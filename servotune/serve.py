#!/usr/bin/env python3
"""Serve ServoTune locally; no dependencies, motor or internet connection needed."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import webbrowser

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', type=int, default=0, help='Local port; default 0 selects a free port')
parser.add_argument('--no-browser', action='store_true')
args = parser.parse_args()
handler = partial(SimpleHTTPRequestHandler, directory=str(Path(__file__).resolve().parent))
with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
    url = f'http://127.0.0.1:{server.server_port}/?demo=1'
    print(f'ServoTune: {url}\nDemo mode uses simulated data. Press Ctrl-C to stop.', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
