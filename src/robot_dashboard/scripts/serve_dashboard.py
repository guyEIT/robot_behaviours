"""Static-file server for the React dashboard with cache-busting headers.

Drop-in replacement for `python -m http.server` that adds:

- `Cache-Control: no-store` on `index.html` and `env-config.js` (the entry
  points). Vite already content-hashes the JS/CSS bundles under /assets/
  so those cache aggressively (1 year), but `index.html` references the
  current bundle hash and MUST be re-fetched after a rebuild — otherwise
  the browser shows stale UI even after `pixi run dashboard-build`.
- `Cache-Control: public, max-age=31536000, immutable` on hashed assets
  so repeat visits are instant.

Bind / port match python -m http.server's CLI: ``python serve_dashboard.py
<port> --directory <dir> --bind <host>``.
"""

from __future__ import annotations

import argparse
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from functools import partial


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        path = self.path.split("?", 1)[0]
        # Hashed bundles in /assets/ never change for a given hash —
        # cache them forever. Everything else (index.html, env-config.js,
        # / itself) must revalidate every load so the browser picks up
        # new bundle hashes after a `pixi run dashboard-build`.
        if path.startswith("/assets/"):
            self.send_header(
                "Cache-Control", "public, max-age=31536000, immutable"
            )
        else:
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
        super().end_headers()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("port", type=int)
    p.add_argument("--directory", required=True)
    p.add_argument("--bind", default="0.0.0.0")
    args = p.parse_args()

    handler = partial(NoCacheHandler, directory=args.directory)
    with ThreadingHTTPServer((args.bind, args.port), handler) as httpd:
        os.chdir(args.directory)
        print(f"Serving {args.directory} on http://{args.bind}:{args.port}")
        httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
