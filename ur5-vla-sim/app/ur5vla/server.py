"""Web GUI + JSON API (standard library only).
   python3 -m ur5vla.server          env: SIM_BACKEND=gazebo|mock PLANNER=vlm|rules
                                          VLM_URL=http://vlm:8081 PORT=8080 SIM_SPEED=1"""
from __future__ import annotations
import json, os, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .agent import Agent
from .planner import Planner, VLMClient
from .backends import make_backend

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "static")


def make_handler(agent: Agent):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode())

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                with open(os.path.join(STATIC, "index.html"), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                return self._json(agent.snapshot())
            if path == "/api/frame.jpg":
                try:
                    jpg = agent.be.frame_jpeg()
                except Exception:
                    jpg = None
                return self._send(200, jpg, "image/jpeg") if jpg else self._send(204, b"", "text/plain")
            if path == "/api/health":
                return self._json({"ok": True, "backend": agent.be.name, "ready": agent.ready})
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                data = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                return self._json({"error": "bad json"}, 400)
            path = self.path.split("?")[0]
            if path == "/api/chat":
                text = str(data.get("text", ""))[:500]
                agent.submit(text)
                return self._json({"ok": True})
            if path == "/api/stop":
                agent.stop()
                return self._json({"ok": True})
            if path == "/api/reset":
                agent.reset_scene()
                return self._json({"ok": True})
            if path == "/api/planner":
                agent.set_planner_mode(data.get("mode"))
                return self._json({"ok": True, "mode": agent.planner.mode})
            self._json({"error": "unknown"}, 404)

    return H


def main():
    kind = os.environ.get("SIM_BACKEND", "gazebo")
    mode = os.environ.get("PLANNER", "vlm")
    url = os.environ.get("VLM_URL", "http://vlm:8081")
    port = int(os.environ.get("PORT", "8080"))
    speed = float(os.environ.get("SIM_SPEED", "1"))
    planner = Planner(mode, VLMClient(url))
    agent = Agent(make_backend(kind), planner, time_scale=speed)
    agent.start()
    srv = ThreadingHTTPServer(("0.0.0.0", port), make_handler(agent))
    srv.daemon_threads = True
    print(f"[ur5vla] backend={kind} planner={mode} vlm={url} -> http://localhost:{port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
