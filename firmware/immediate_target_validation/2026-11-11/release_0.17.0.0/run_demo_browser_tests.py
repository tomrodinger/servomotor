from functools import partial
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import os,subprocess,threading
root=Path.cwd()
server=ThreadingHTTPServer(('127.0.0.1',0),partial(SimpleHTTPRequestHandler,directory=str(root/'servotune')))
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
env=os.environ.copy()
env['PLAYWRIGHT_MODULE']='/Users/tom/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright'
env['SERVOTUNE_URL']=f'http://127.0.0.1:{server.server_port}/?demo=1'
try:
    print('Demo-only browser tests at',env['SERVOTUNE_URL'],flush=True)
    result=subprocess.run(['node','servotune/test_browser.cjs'],env=env,timeout=90)
    assert result.returncode==0,'Browser tests failed'
finally:
    server.shutdown();server.server_close();thread.join()
