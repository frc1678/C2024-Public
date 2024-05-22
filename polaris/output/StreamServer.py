import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

from configuration.Configuration import Device


class StreamServer:
    """Interface for outputing camera frames."""

    def start(self, config: Device) -> None:
        """Starts the output stream."""
        raise NotImplementedError

    def set_frame(self, frame: cv2.Mat) -> None:
        """Sets the frame to serve."""
        raise NotImplementedError


class MjpegServer(StreamServer):
    _frame: cv2.Mat
    _has_frame: bool = False

    def _make_handler(self_mjpeg):  # type: ignore
        class StreamingHandler(BaseHTTPRequestHandler):
            HTML = """
    <html>
        <head>
            <title>Polaris Debug</title>
            <style>
                body {
                    background-color: black;
                }

                img {
                    position: absolute;
                    left: 50%;
                    top: 50%;
                    transform: translate(-50%, -50%);
                    max-width: 100%;
                    max-height: 100%;
                }
            </style>
        </head>
        <body>
            <img src="stream.mjpg" />
        </body>
    </html>
            """

            def do_GET(self):
                if self.path == "/":
                    content = self.HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                elif self.path == "/stream.mjpg":
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header(
                        "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
                    )
                    self.end_headers()
                    try:
                        while True:
                            if not self_mjpeg._has_frame:
                                print("No frames?")
                                time.sleep(0.1)
                            else:
                                frame = cv2.resize(self_mjpeg._frame, (400, 300))
                                _, enc = cv2.imencode(".jpg", frame)

                                frame_data = np.array(enc).tobytes()

                                self.wfile.write(b"--FRAME\r\n")
                                self.send_header("Content-Type", "image/jpeg")
                                self.send_header("Content-Length", str(len(frame_data)))
                                self.end_headers()
                                self.wfile.write(frame_data)
                                self.wfile.write(b"\r\n")
                    except Exception as e:
                        print(
                            "Removed streaming client %s: %s",
                            self.client_address,
                            str(e),
                        )
                else:
                    self.send_error(404)
                    self.end_headers()

        return StreamingHandler

    class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    def _run(self, port: int) -> None:
        server = self.StreamingServer(("", port), self._make_handler())
        server.serve_forever()

    def start(self, config: Device) -> None:
        threading.Thread(
            target=self._run, daemon=True, args=(config.stream_port,)
        ).start()

    def set_frame(self, frame: cv2.Mat) -> None:
        self._frame = frame.copy()
        self._has_frame = True
