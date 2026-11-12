/* Linux-only opt-in experiment. No SDK linkage, payload logging or send hooks.
 * Re-arm TCP_QUICKACK after successful receives on one exact IPv4 TCP peer.
 * SDK shutdown must join receive threads before closing their descriptors.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>

static pthread_once_t once = PTHREAD_ONCE_INIT;
static _Thread_local bool initializing;
static bool enabled, invalid_configuration;
static struct in_addr target_address;
static unsigned short target_port;
static char target_host[INET_ADDRSTRLEN] = "192.168.1.5";
static _Atomic unsigned long matched_receives, successful_options, failed_options;
static _Atomic unsigned long recv_hits, read_hits, recvfrom_hits, recvmsg_hits, readv_hits;
static _Atomic int first_target_fd = -1;
static _Atomic int last_option_errno;
static _Atomic bool first_reported;
static ssize_t (*next_recv)(int, void *, size_t, int);
static ssize_t (*next_read)(int, void *, size_t);
static ssize_t (*next_recvfrom)(int, void *, size_t, int, struct sockaddr *, socklen_t *);
static ssize_t (*next_recvmsg)(int, struct msghdr *, int);
static ssize_t (*next_readv)(int, const struct iovec *, int);

static void log_status(const char *event) {
    char line[1024];
    int n = snprintf(line, sizeof(line),
        "AUBO_QUICKACK {\"event\":\"%s\",\"enabled\":%s,"
        "\"invalid_configuration\":%s,\"host\":\"%s\",\"port\":%u,"
        "\"matched_receives\":%lu,\"setsockopt_success\":%lu,\"setsockopt_failed\":%lu,"
        "\"first_target_fd\":%d,\"last_option_errno\":%d,\"recv\":%lu,\"read\":%lu,\"recvfrom\":%lu,"
        "\"recvmsg\":%lu,\"readv\":%lu}\n",
        event, enabled ? "true" : "false", invalid_configuration ? "true" : "false",
        target_host, (unsigned)target_port, atomic_load(&matched_receives),
        atomic_load(&successful_options), atomic_load(&failed_options),
        atomic_load(&first_target_fd), atomic_load(&last_option_errno), atomic_load(&recv_hits), atomic_load(&read_hits),
        atomic_load(&recvfrom_hits), atomic_load(&recvmsg_hits), atomic_load(&readv_hits));
    if (n > 0 && (size_t)n < sizeof(line)) {
        /* Direct syscall avoids recursion into intercepted libc I/O. */
        (void)syscall(SYS_write, STDERR_FILENO, line, (size_t)n);
    }
}

static void initialize_once(void) {
    initializing = true;
    next_recv = dlsym(RTLD_NEXT, "recv");
    next_read = dlsym(RTLD_NEXT, "read");
    next_recvfrom = dlsym(RTLD_NEXT, "recvfrom");
    next_recvmsg = dlsym(RTLD_NEXT, "recvmsg");
    next_readv = dlsym(RTLD_NEXT, "readv");
    const char *switch_value = getenv("AUBO_QUICKACK");
    const char *host = getenv("AUBO_QUICKACK_HOST");
    const char *port = getenv("AUBO_QUICKACK_PORT");
    enabled = switch_value && strcmp(switch_value, "1") == 0;
    if (switch_value && strcmp(switch_value, "0") != 0 && strcmp(switch_value, "1") != 0)
        invalid_configuration = true;
    if (!host) host = "192.168.1.5";
    if (!port) port = "8899";
    char *end = NULL;
    errno = 0;
    long parsed = strtol(port, &end, 10);
    if (errno || !*port || *end || parsed < 1 || parsed > USHRT_MAX)
        invalid_configuration = true;
    else target_port = (unsigned short)parsed;
    if (inet_pton(AF_INET, host, &target_address) != 1)
        invalid_configuration = true;
    else (void)inet_ntop(AF_INET, &target_address, target_host, sizeof(target_host));
    if (!next_recv || !next_read || !next_recvfrom || !next_recvmsg || !next_readv)
        invalid_configuration = true;
    if (invalid_configuration) enabled = false;
    initializing = false;
    log_status("loaded");
}

static void initialize(void) {
    if (!initializing) (void)pthread_once(&once, initialize_once);
}

__attribute__((constructor)) static void loaded(void) {
    int saved = errno;
    initialize();
    errno = saved;
}

__attribute__((destructor)) static void unloaded(void) { log_status("summary"); }

static void received(int fd, ssize_t count, _Atomic unsigned long *path_hits) {
    if (!enabled || count <= 0 || initializing) return;
    int saved = errno;
    struct sockaddr_in peer;
    socklen_t length = sizeof(peer);
    int type = 0, protocol = 0;
    socklen_t option_length = sizeof(type);
    if (getpeername(fd, (struct sockaddr *)&peer, &length) == 0 &&
            length == sizeof(peer) && peer.sin_family == AF_INET &&
            peer.sin_addr.s_addr == target_address.s_addr && ntohs(peer.sin_port) == target_port &&
            getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &option_length) == 0 && type == SOCK_STREAM) {
        option_length = sizeof(protocol);
        if (getsockopt(fd, SOL_SOCKET, SO_PROTOCOL, &protocol, &option_length) == 0 && protocol == IPPROTO_TCP) {
            atomic_fetch_add(&matched_receives, 1);
            atomic_fetch_add(path_hits, 1);
            int unset = -1;
            (void)atomic_compare_exchange_strong(&first_target_fd, &unset, fd);
            int one = 1;
            if (setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &one, sizeof(one)) == 0)
                atomic_fetch_add(&successful_options, 1);
            else {
                atomic_store(&last_option_errno, errno);
                atomic_fetch_add(&failed_options, 1);
            }
            bool expected = false;
            if (atomic_compare_exchange_strong(&first_reported, &expected, true))
                log_status("first_target_receive");
        }
    }
    errno = saved;
}

ssize_t recv(int fd, void *buffer, size_t length, int flags) {
    initialize();
    ssize_t n = next_recv ? next_recv(fd, buffer, length, flags)
                         : syscall(SYS_recvfrom, fd, buffer, length, flags, NULL, NULL);
    received(fd, n, &recv_hits);
    return n;
}

ssize_t read(int fd, void *buffer, size_t length) {
    initialize();
    ssize_t n = next_read ? next_read(fd, buffer, length) : syscall(SYS_read, fd, buffer, length);
    received(fd, n, &read_hits);
    return n;
}

ssize_t recvfrom(int fd, void *buffer, size_t length, int flags, struct sockaddr *address, socklen_t *address_length) {
    initialize();
    ssize_t n = next_recvfrom ? next_recvfrom(fd, buffer, length, flags, address, address_length)
                             : syscall(SYS_recvfrom, fd, buffer, length, flags, address, address_length);
    received(fd, n, &recvfrom_hits);
    return n;
}

ssize_t recvmsg(int fd, struct msghdr *message, int flags) {
    initialize();
    ssize_t n = next_recvmsg ? next_recvmsg(fd, message, flags) : syscall(SYS_recvmsg, fd, message, flags);
    received(fd, n, &recvmsg_hits);
    return n;
}

ssize_t readv(int fd, const struct iovec *vectors, int count) {
    initialize();
    ssize_t n = next_readv ? next_readv(fd, vectors, count) : syscall(SYS_readv, fd, vectors, count);
    received(fd, n, &readv_hits);
    return n;
}
