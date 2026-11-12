"""Loopback-only LD_PRELOAD tests. Never load the SDK or contact a cabinet.

Build with scripts/build_aubo_quickack.sh first. All TCP/UDP destinations are
ephemeral loopback ports, including deliberate target/non-target comparisons.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import tempfile
import unittest

LIBRARY=Path(os.environ.get('AUBO_RUNTIME_DIR','/data/VR_teleoperation_runtime'))/'quickack/libaubo_quickack.so'


class QuickackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not LIBRARY.is_file():
            raise RuntimeError('Build scripts/build_aubo_quickack.sh before this test')

    def run_client(self, code, *, hook='recv', enabled='1', host='127.0.0.1',
                   different_port=False, clients=1, udp=False, extra_preload=None):
        kind=socket.SOCK_DGRAM if udp else socket.SOCK_STREAM
        server=socket.socket(socket.AF_INET,kind);self.addCleanup(server.close)
        server.bind(('127.0.0.1',0));server.settimeout(5)
        port=server.getsockname()[1]
        if not udp:server.listen(clients)
        errors=[]
        def serve():
            try:
                for _ in range(clients):
                    if udp:
                        data,peer=server.recvfrom(32);server.sendto(data,peer)
                    else:
                        conn,_=server.accept()
                        with conn:
                            conn.sendall(b'hello')
            except Exception as exc:errors.append(repr(exc))
        thread=threading.Thread(target=serve,daemon=True);thread.start()
        preload=str(LIBRARY)+((':'+str(extra_preload)) if extra_preload else '')
        env=dict(os.environ,LD_PRELOAD=preload,AUBO_QUICKACK=enabled,
            AUBO_QUICKACK_HOST=host,AUBO_QUICKACK_PORT=str(port+1 if different_port and port<65535 else port-1 if different_port else port))
        proc=subprocess.run([sys.executable,'-c',code,str(port),hook],env=env,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=8)
        thread.join(timeout=6)
        self.assertFalse(thread.is_alive(),'Loopback server did not finish')
        self.assertFalse(errors,errors)
        self.assertEqual(proc.returncode,0,proc.stderr)
        reports=[json.loads(line.split(' ',1)[1]) for line in proc.stderr.splitlines()
                 if line.startswith('AUBO_QUICKACK ')]
        expected=['loaded']+(['first_target_receive'] if reports[-1]['matched_receives'] else [])+['summary']
        self.assertEqual([r['event'] for r in reports],expected)
        self.assertNotIn('hello',proc.stderr)
        return proc,reports[-1]

    CLIENT='''
import ctypes,errno,os,socket,sys
port=int(sys.argv[1]);kind=sys.argv[2]
with socket.create_connection(('127.0.0.1',port),timeout=3) as sock:
    sock.setblocking(True)
    if kind=='recv': data=sock.recv(5)
    elif kind=='read': data=os.read(sock.fileno(),5)
    elif kind=='recvfrom': data=sock.recvfrom(5)[0]
    elif kind=='recvmsg': data=sock.recvmsg(5)[0]
    elif kind=='readv':
        a,b=bytearray(2),bytearray(3)
        assert os.readv(sock.fileno(),[a,b])==5
        data=bytes(a+b)
    assert data==b'hello',repr(data)
    assert sock.recv(1)==b''
print('payload and EOF preserved')
'''

    def test_each_receive_path_sets_option_only_after_positive_receive(self):
        for hook in ('recv','read','recvfrom','recvmsg','readv'):
            with self.subTest(hook=hook):
                _,r=self.run_client(self.CLIENT,hook=hook)
                self.assertTrue(r['enabled'])
                self.assertEqual(r['matched_receives'],1)
                self.assertEqual(r['setsockopt_success'],1)
                self.assertEqual(r['setsockopt_failed'],0)
                self.assertEqual(r[hook],1)

    def test_disabled_library_preserves_payload_without_changing_socket(self):
        _,r=self.run_client(self.CLIENT,enabled='0')
        self.assertFalse(r['enabled'])
        self.assertEqual(r['matched_receives'],0)

    @unittest.skipUnless(hasattr(os,'RTLD_DEEPBIND'),'Linux DEEPBIND required')
    def test_deepbind_bypasses_preload_hooks_without_changing_received_data(self):
        # Reproduce the SDK loader's symbol scope using a tiny libc-linked
        # receiver, never the vendor SDK or a controller connection.
        with tempfile.TemporaryDirectory(dir=LIBRARY.parent) as directory:
            source=Path(directory)/'isolated_receiver.c'
            library=Path(directory)/'isolated_receiver.so'
            source.write_text('''
#include <sys/socket.h>
#include <sys/types.h>
ssize_t isolated_receive(int fd, void *buffer, size_t count) {
    return recv(fd,buffer,count,0);
}
''')
            subprocess.run(['gcc','-shared','-fPIC','-O2',str(source),'-o',str(library)],check=True)
            for deepbind in (False,True):
                with self.subTest(deepbind=deepbind):
                    code='''
import ctypes,os,socket,sys
mode=os.RTLD_NOW|os.RTLD_LOCAL
if DEEPBIND: mode|=os.RTLD_DEEPBIND
receiver=ctypes.CDLL(LIBRARY,mode=mode)
receiver.isolated_receive.argtypes=(ctypes.c_int,ctypes.c_void_p,ctypes.c_size_t)
receiver.isolated_receive.restype=ctypes.c_ssize_t
with socket.create_connection(('127.0.0.1',int(sys.argv[1])),timeout=3) as sock:
    sock.setblocking(True)
    buffer=ctypes.create_string_buffer(5)
    assert receiver.isolated_receive(sock.fileno(),buffer,5)==5
    assert buffer.raw==b'hello'
'''.replace('DEEPBIND',repr(deepbind),1).replace('LIBRARY',repr(str(library)),1)
                    _,report=self.run_client(code)
                    self.assertEqual(report['matched_receives'],0 if deepbind else 1)
                    self.assertEqual(report['setsockopt_success'],0 if deepbind else 1)

    def test_other_tcp_port_is_not_modified(self):
        _,r=self.run_client(self.CLIENT,different_port=True)
        self.assertTrue(r['enabled'])
        self.assertEqual(r['matched_receives'],0)
        self.assertEqual(r['first_target_fd'],-1)

    def test_other_ipv4_peer_is_not_modified(self):
        _,r=self.run_client(self.CLIENT,host='127.0.0.2')
        self.assertEqual(r['matched_receives'],0)

    def test_invalid_configuration_is_visible_and_does_not_modify_sockets(self):
        _,r=self.run_client(self.CLIENT,host='invalid-host')
        self.assertTrue(r['invalid_configuration'])
        self.assertFalse(r['enabled'])
        self.assertEqual(r['matched_receives'],0)

    def test_udp_with_same_peer_is_not_modified(self):
        code='''
import socket,sys
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
    sock.settimeout(3);sock.connect(('127.0.0.1',int(sys.argv[1])))
    sock.send(b'hello');assert sock.recv(5)==b'hello'
'''
        _,r=self.run_client(code,udp=True)
        self.assertEqual(r['matched_receives'],0)

    def test_descriptor_reuse_as_pipe_is_not_treated_as_old_tcp_socket(self):
        code='''
import os,socket,sys
sock=socket.create_connection(('127.0.0.1',int(sys.argv[1])),timeout=3)
sock.setblocking(True)
fd=sock.fileno();assert os.read(fd,5)==b'hello';sock.close()
reader,writer=os.pipe()
if reader!=fd:os.dup2(reader,fd);os.close(reader)
os.write(writer,b'file');assert os.read(fd,4)==b'file'
os.close(fd);os.close(writer)
'''
        _,r=self.run_client(code)
        self.assertEqual(r['matched_receives'],1)
        self.assertEqual(r['read'],1)

    def test_success_and_failure_errno_and_return_values_are_preserved(self):
        code='''
import ctypes,errno,socket,sys
libc=ctypes.CDLL(None,use_errno=True);libc.read.restype=ctypes.c_ssize_t
buffer=ctypes.create_string_buffer(5)
with socket.create_connection(('127.0.0.1',int(sys.argv[1])),timeout=3) as sock:
    sock.setblocking(True);ctypes.set_errno(errno.E2BIG)
    assert libc.read(sock.fileno(),buffer,5)==5
    assert buffer.raw==b'hello'
    assert ctypes.get_errno()==errno.E2BIG
ctypes.set_errno(0)
assert libc.read(-1,buffer,5)==-1
assert ctypes.get_errno()==errno.EBADF
'''
        _,r=self.run_client(code)
        self.assertEqual(r['matched_receives'],1)

    def test_parallel_receivers_keep_complete_counters(self):
        code='''
import socket,sys,threading
errors=[]
def client():
    try:
        with socket.create_connection(('127.0.0.1',int(sys.argv[1])),timeout=3) as sock:
            assert sock.recv(5)==b'hello'
    except Exception as exc:errors.append(repr(exc))
threads=[threading.Thread(target=client) for _ in range(12)]
for thread in threads:thread.start()
for thread in threads:thread.join()
assert not errors,errors
'''
        _,r=self.run_client(code,clients=12)
        self.assertEqual(r['matched_receives'],12)
        self.assertEqual(r['setsockopt_success'],12)
        self.assertEqual(r['setsockopt_failed'],0)

    def test_option_failure_is_reported_without_changing_receive_result(self):
        import errno
        with tempfile.TemporaryDirectory(dir=LIBRARY.parent) as directory:
            source=Path(directory)/'reject_option.c';library=Path(directory)/'reject_option.so'
            source.write_text('''
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
int setsockopt(int fd,int level,int option,const void *value,socklen_t length) {
    if(level==IPPROTO_TCP && option==TCP_QUICKACK) {errno=EPERM;return -1;}
    int (*original)(int,int,int,const void*,socklen_t)=dlsym(RTLD_NEXT,"setsockopt");
    return original(fd,level,option,value,length);
}
''')
            subprocess.run(['gcc','-shared','-fPIC','-O2',str(source),'-o',str(library),'-ldl'],check=True)
            _,r=self.run_client(self.CLIENT,extra_preload=library)
            self.assertEqual(r['matched_receives'],1)
            self.assertEqual(r['setsockopt_success'],0)
            self.assertEqual(r['setsockopt_failed'],1)
            self.assertEqual(r['last_option_errno'],errno.EPERM)


if __name__=='__main__':unittest.main(verbosity=2)
