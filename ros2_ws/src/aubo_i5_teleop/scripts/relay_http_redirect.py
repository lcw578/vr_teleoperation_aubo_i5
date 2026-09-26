#!/usr/bin/env python3
"""8080 端口的 HTTP→HTTPS 跳转服务（relay 的"纠错入口"）。

为什么存在：relay 主服务（8443）是纯 TLS 的，浏览器用 http:// 访问 8443 会
得到 502（经 Clash 时）或连接重置（直连）——2026-09-26 用户实测踩坑。
本服务监听 8080 纯 HTTP，把任何请求 301 跳转到 https://<host>:8443/，
让"漏掉 s"的输入也能到达正确页面。

与主 relay 分开进程是安全的：本服务无状态（只跳转），不参与 WebSocket
广播——头显浏览器跳转后自然连接 8443 的主 hub。

用法：python3 scripts/relay_http_redirect.py &   （跟主 relay 一起启动）
"""
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HTTPS_PORT = 8443


class Redirect(BaseHTTPRequestHandler):
    def do_GET(self):
        host = self.headers.get("Host", "").split(":")[0] or "10.26.185.27"
        self.send_response(301)
        self.send_header("Location", f"https://{host}:{HTTPS_PORT}{self.path}")
        self.end_headers()

    def log_message(self, fmt, *args):
        print("[redirect] " + fmt % args, flush=True)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    srv = ThreadingHTTPServer(("0.0.0.0", port), Redirect)
    print(f"HTTP→HTTPS 跳转服务：http://<本机IP>:{port}/ → https://<本机IP>:{HTTPS_PORT}/", flush=True)
    srv.serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
