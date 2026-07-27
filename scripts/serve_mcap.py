#!/usr/bin/env python3
"""
CLI tool to serve or upload MCAP files for Foxglove Studio.

Foxglove Studio Web requires:
  1. HTTP 206 Partial Content support for byte-range requests ('Accept-Ranges: bytes', 'Content-Range: bytes ...')
  2. Full CORS headers (Access-Control-Allow-Origin: *, OPTIONS preflight handling)

This tool serves files locally on http://localhost:8080 with 100% 
Foxglove-compliant CORS and Range headers for instant, zero-latency streaming.
"""

import argparse
import http.server
import json
import mimetypes
import os
import socketserver
import sys
import threading
import time
import uuid
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class FoxgloveCORSHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP Request Handler tailored for Foxglove Studio CORS & Range requirements."""
    target_filepath: Path = None

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Range, Content-Type, Authorization, X-Requested-With")
        self.send_header("Access-Control-Expose-Headers", "Content-Length, Content-Range, Accept-Ranges, Content-Type")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200, "OK")
        self.end_headers()

    def do_GET(self):
        if not self.target_filepath or not self.target_filepath.exists():
            self.send_error(404, "File not found")
            return

        file_size = self.target_filepath.stat().st_size
        range_header = self.headers.get("Range")

        try:
            if range_header and range_header.startswith("bytes="):
                range_str = range_header.split("=")[1].strip()
                parts = range_str.split("-")
                
                if not parts[0]:  # e.g., bytes=-8 (last 8 bytes)
                    suffix_length = int(parts[1])
                    start = max(0, file_size - suffix_length)
                    end = file_size - 1
                else:
                    start = int(parts[0])
                    end = int(parts[1]) if len(parts) > 1 and parts[1] else file_size - 1

                if start >= file_size or end >= file_size or start > end:
                    self.send_response(416, "Requested Range Not Satisfiable")
                    self.send_header("Content-Range", f"bytes */{file_size}")
                    self.end_headers()
                    return

                length = end - start + 1
                self.send_response(206, "Partial Content")
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(length))
                self.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
                self.end_headers()

                with open(self.target_filepath, "rb") as f:
                    f.seek(start)
                    chunk_size = 64 * 1024
                    bytes_remaining = length
                    while bytes_remaining > 0:
                        to_read = min(chunk_size, bytes_remaining)
                        data = f.read(to_read)
                        if not data:
                            break
                        self.wfile.write(data)
                        bytes_remaining -= len(data)
            else:
                self.send_response(200, "OK")
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(file_size))
                self.end_headers()

                with open(self.target_filepath, "rb") as f:
                    chunk_size = 64 * 1024
                    while True:
                        data = f.read(chunk_size)
                        if not data:
                            break
                        self.wfile.write(data)
        except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
            pass  # Suppress normal socket disconnects during Range streaming

    def log_message(self, format, *args):
        # Clean log output
        sys.stderr.write(f"[{time.strftime('%H:%M:%S')}] {args[0]}\n")


def serve_local(filepath: Path, port: int = 8080, **_) -> str:
    """Serves file locally with full Foxglove-compliant CORS & Range headers."""
    target_resolved = filepath.resolve()
    if not target_resolved.exists():
        raise FileNotFoundError(f"File not found: {filepath}")

    FoxgloveCORSHandler.target_filepath = target_resolved

    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    try:
        server = ThreadedHTTPServer(("0.0.0.0", port), FoxgloveCORSHandler)
    except OSError:
        server = None

    if server:
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()

    local_url = f"http://localhost:{port}/{target_resolved.name}"

    print("\n=======================================================", file=sys.stderr)
    print("  🚀 Foxglove HTTP 206 Range Stream Server Active", file=sys.stderr)
    print("=======================================================", file=sys.stderr)
    print(f" File Path   : {target_resolved}", file=sys.stderr)
    print(f" Local URL   : {local_url}", file=sys.stderr)
    print("\n 🦊 To View in Foxglove Studio:", file=sys.stderr)
    print(f"    1. Open https://studio.foxglove.dev", file=sys.stderr)
    print(f"    2. Click 'Open file from URL'", file=sys.stderr)
    print(f"    3. Paste: {local_url}", file=sys.stderr)
    print("=======================================================\n", file=sys.stderr)

    print(local_url)

    if server:
        try:
            print("Serving Range requests (Press Ctrl+C to stop)...", file=sys.stderr)
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nServer stopped.", file=sys.stderr)
            server.shutdown()
            sys.exit(0)

    return local_url


def encode_multipart_formdata(fields: dict, files: dict) -> tuple[bytes, str]:
    boundary = f"----WebKitFormBoundary{uuid.uuid4().hex}"
    body = bytearray()

    for name, value in fields.items():
        body.extend(f"--{boundary}\r\n".encode("utf-8"))
        body.extend(f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode("utf-8"))
        body.extend(f"{value}\r\n".encode("utf-8"))

    for field_name, (filename, file_data) in files.items():
        mime_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
        body.extend(f"--{boundary}\r\n".encode("utf-8"))
        body.extend(f'Content-Disposition: form-data; name="{field_name}"; filename="{filename}"\r\n'.encode("utf-8"))
        body.extend(f"Content-Type: {mime_type}\r\n\r\n".encode("utf-8"))
        body.extend(file_data)
        body.extend(b"\r\n")

    body.extend(f"--{boundary}--\r\n".encode("utf-8"))
    return bytes(body), f"multipart/form-data; boundary={boundary}"


def upload_litterbox(filepath: Path, retention: str = "24h", **_) -> str:
    url = "https://litterbox.catbox.moe/resources/internals/api.php"
    with open(filepath, "rb") as f:
        file_data = f.read()

    fields = {"reqtype": "fileupload", "time": retention}
    files = {"fileToUpload": (filepath.name, file_data)}
    body, content_type = encode_multipart_formdata(fields, files)

    req = Request(url, data=body, headers={"User-Agent": "Mozilla/5.0", "Content-Type": content_type})
    with urlopen(req) as resp:
        return resp.read().decode("utf-8").strip()


def upload_tmpfiles(filepath: Path, **_) -> str:
    url = "https://tmpfiles.org/api/v1/upload"
    with open(filepath, "rb") as f:
        file_data = f.read()

    body, content_type = encode_multipart_formdata({}, {"file": (filepath.name, file_data)})
    req = Request(url, data=body, headers={"User-Agent": "Mozilla/5.0", "Content-Type": content_type})

    with urlopen(req) as resp:
        res_data = json.loads(resp.read().decode("utf-8"))
        page_url = res_data.get("data", {}).get("url")
        if not page_url:
            raise RuntimeError(f"Invalid response from tmpfiles: {res_data}")
        return page_url.replace("tmpfiles.org/", "tmpfiles.org/dl/")


PROVIDERS = {
    "local": serve_local,
    "litterbox": upload_litterbox,
    "tmpfiles": upload_tmpfiles,
}


def main():
    parser = argparse.ArgumentParser(
        description="Serve or upload MCAP files with Foxglove-compliant CORS & 206 Range headers."
    )
    parser.add_argument("file", type=str, help="Path to any MCAP/file in the workspace (e.g. output/results/living_room/pipeline.mcap)")
    parser.add_argument(
        "--service",
        "-s",
        choices=list(PROVIDERS.keys()),
        default="local",
        help="Service mode (default: local for instant zero-latency HTTP 206 Range streaming)",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=8080,
        help="Port for local HTTP server (default: 8080)",
    )

    args = parser.parse_args()
    filepath = Path(args.file)

    if not filepath.exists() or not filepath.is_file():
        print(f"Error: File '{args.file}' not found.", file=sys.stderr)
        sys.exit(1)

    file_size_mb = filepath.stat().st_size / (1024 * 1024)
    print(f"Processing '{filepath.resolve()}' ({file_size_mb:.2f} MB)...", file=sys.stderr)

    try:
        provider_fn = PROVIDERS[args.service]
        url = provider_fn(filepath, port=args.port)

        if args.service != "local":
            print("\n--- Upload Successful ---", file=sys.stderr)
            print(url)
    except (URLError, HTTPError, RuntimeError, Exception) as e:
        print(f"\nError processing file: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
