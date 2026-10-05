"""Serve deterministic baseline/current browser WebGL qualification, without backend/device access."""
import argparse
import functools
import http.server
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MODULES = ('pointcloud3d.js', 'pointcloud-incremental.js', 'pointcloud-pages.js', 'three.module.min.js',
           'semantic-regions3d.js', 'scene-graph3d.js', 'OrbitControls.js', 'pointcloud-processing.js', 'dense-pointcloud.js')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', default='67c29be27a3800241b3cdf14dc7ec3a11d619206')
    parser.add_argument('--candidate', default='a56d0cfaad71c1d0ddb0ec8df02364ce3329b753')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix='neh-forbidden-webgl-'))
    output.mkdir(parents=True, exist_ok=True)
    refs = {name: subprocess.check_output(['git', 'rev-parse', ref], cwd=ROOT, text=True).strip()
            for name, ref in [('baseline', args.baseline), ('candidate', args.candidate)]}
    for name, ref in refs.items():
        folder = output / name
        folder.mkdir(exist_ok=True)
        for module in MODULES:
            (folder / module).write_bytes(subprocess.check_output(['git', 'show', f'{ref}:gateways/http/static/js/{module}'], cwd=ROOT))
        (folder / 'index.html').write_bytes((ROOT / 'scripts/browser_checks/forbidden-overlay/viewer.html').read_bytes())
    (output / 'index.html').write_bytes((ROOT / 'scripts/browser_checks/forbidden-overlay/index.html').read_bytes())
    (output / 'refs.json').write_text(json.dumps(refs))

    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_POST(self):
            if self.path != '/result':
                self.send_error(404)
                return
            origin = self.headers.get('Origin')
            if origin not in {f'http://127.0.0.1:{args.port}', f'http://localhost:{args.port}'}:
                self.send_error(403)
                return
            if self.headers.get_content_type() != 'application/json':
                self.send_error(415)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                self.send_error(400)
                return
            if not 0 < size <= 8 * 1024 * 1024:
                self.send_error(413)
                return
            try:
                payload = json.loads(self.rfile.read(size))
            except (ValueError, UnicodeDecodeError):
                self.send_error(400)
                return
            if not isinstance(payload, dict):
                self.send_error(400)
                return
            payload['refs'] = refs
            (output / 'result.json').write_text(json.dumps(payload, indent=2))
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'OK')
    server = http.server.ThreadingHTTPServer(('127.0.0.1', args.port), functools.partial(Handler, directory=str(output)))
    print(f'Open http://127.0.0.1:{args.port}/ ; results {output}/result.json', flush=True)
    server.serve_forever()

if __name__ == '__main__':
    main()
