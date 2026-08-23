#!/usr/bin/env python3
"""Static server for marketting_page, plus a tiny ratings API.

`python3 -m http.server` only does GET, so there was no way for a rating made in the browser to
reach disk — and localStorage is invisible to anything outside the browser. This adds two routes so
keep/discard decisions land in ratings/*.json where they can be read directly:

  POST /api/ratings/<name>   body = JSON   -> writes ratings/<name>.json
  GET  /api/ratings/<name>                 -> reads it back (or {} if absent)

Everything else is served from disk exactly as before.

  python3 serve.py [port]        (default 8912)
"""
import json
import os
import sys
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.abspath(__file__))
RATINGS = os.path.join(ROOT, "ratings")
os.makedirs(RATINGS, exist_ok=True)
_lock = threading.Lock()


def _safe(name):
    """ratings names are simple slugs — never a path."""
    keep = "".join(c for c in name if c.isalnum() or c in "-_")
    return keep or "unnamed"


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def log_message(self, fmt, *args):
        pass  # quiet; this runs in the background all night

    def _json(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path.startswith("/api/ratings/"):
            name = _safe(self.path[len("/api/ratings/"):].split("?")[0])
            path = os.path.join(RATINGS, name + ".json")
            try:
                with open(path, encoding="utf-8") as fh:
                    return self._json(200, json.load(fh))
            except Exception:
                return self._json(200, {})
        # never let the browser cache the pages while they are being iterated on
        self.send_header_hook = True
        return super().do_GET()

    def end_headers(self):
        if getattr(self, "send_header_hook", False):
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.send_header_hook = False
        super().end_headers()

    def do_POST(self):
        if not self.path.startswith("/api/ratings/"):
            return self._json(404, {"error": "no such endpoint"})
        name = _safe(self.path[len("/api/ratings/"):].split("?")[0])
        try:
            n = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(n) or b"{}")
        except Exception as e:
            return self._json(400, {"error": "bad json: %s" % e})
        path = os.path.join(RATINGS, name + ".json")
        with _lock:
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2, ensure_ascii=False)
            os.replace(tmp, path)
        return self._json(200, {"ok": True, "saved": os.path.relpath(path, ROOT),
                                "count": len(data) if hasattr(data, "__len__") else 1})


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8912
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print("serving %s on http://127.0.0.1:%d  (ratings -> %s)" % (ROOT, port, RATINGS))
    srv.serve_forever()
