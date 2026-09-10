#!/usr/bin/env python3
"""
Local dashboard backend for MicroMouse PID tuning + maze algorithm
validation. Serves the frontend (./static) and bridges it to:
  - the real robot over serial (telemetry in, PID/RUN commands out -
    see ../../include/comms.h for the exact line protocol)
  - tools/maze_cli (runs the actual robot algorithm - ../../src/maze.cpp -
    against a maze, for search/speed-run validation without scripting
    the mms simulator's GUI)
  - the mms simulator AppImage (launched as a plain subprocess, for
    visual/manual reference alongside the above)

Run with a python3 that has flask + pyserial installed. If `python3` on
your PATH is shadowed by PlatformIO's virtualenv (it is, on at least one
of this project's dev machines), use the system one explicitly:
    /usr/bin/python3 server.py
Then open http://127.0.0.1:5055 in a browser.
"""
import json
import os
import queue
import subprocess
import threading

from flask import Flask, Response, jsonify, request, send_from_directory
import serial
import serial.tools.list_ports

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAZE_CLI_PATH = os.path.normpath(os.path.join(BASE_DIR, "..", "maze_cli", "maze_cli"))
MMS_APPIMAGE_PATH = "/media/yasiru/New Volume/Praxis/MicroMouse/Simulator/mms-x86_64.AppImage"

app = Flask(__name__, static_folder=os.path.join(BASE_DIR, "static"), static_url_path="")


class SerialBridge:
    """Owns the one serial connection to the robot and fans its lines out
    to any number of SSE listeners (see /api/telemetry/stream)."""

    def __init__(self):
        self.ser = None
        self.reader_thread = None
        self.stop_flag = threading.Event()
        self.listeners = []
        self.lock = threading.Lock()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud):
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=1)
        self.stop_flag.clear()
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def disconnect(self):
        self.stop_flag.set()
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
            self.reader_thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send_line(self, line):
        if not self.is_connected():
            raise RuntimeError("not connected")
        self.ser.write((line.strip() + "\n").encode("utf-8"))

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self.lock:
            self.listeners.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.listeners:
                self.listeners.remove(q)

    def _broadcast(self, line):
        with self.lock:
            for q in self.listeners:
                try:
                    q.put_nowait(line)
                except queue.Full:
                    pass

    def _read_loop(self):
        buf = b""
        while not self.stop_flag.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    self._broadcast(text)


bridge = SerialBridge()


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/serial/ports")
def serial_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return jsonify({"ports": ports, "connected": bridge.is_connected()})


@app.route("/api/serial/connect", methods=["POST"])
def serial_connect():
    data = request.get_json(force=True)
    port = data.get("port")
    baud = int(data.get("baud", 115200))
    try:
        bridge.connect(port, baud)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/serial/disconnect", methods=["POST"])
def serial_disconnect():
    bridge.disconnect()
    return jsonify({"ok": True})


@app.route("/api/serial/send", methods=["POST"])
def serial_send():
    data = request.get_json(force=True)
    line = data.get("line", "")
    try:
        bridge.send_line(line)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route("/api/telemetry/stream")
def telemetry_stream():
    q = bridge.subscribe()

    def gen():
        try:
            while True:
                try:
                    line = q.get(timeout=15)
                    yield f"data: {line}\n\n"
                except queue.Empty:
                    yield ": keepalive\n\n"
        finally:
            bridge.unsubscribe(q)

    return Response(gen(), mimetype="text/event-stream")


@app.route("/api/maze/run", methods=["POST"])
def maze_run():
    data = request.get_json(force=True)
    mode = data.get("mode", "search")

    lines = [f"MODE {mode}"]

    start = data.get("start", {"x": 0, "y": 0})
    lines.append(f"START {start['x']} {start['y']}")

    goals = data.get("goals")
    if goals:
        lines.append("GOALS %d %s" % (len(goals), " ".join(f"{g['x']} {g['y']}" for g in goals)))

    if data.get("random"):
        lines.append("RANDOM 1")
        lines.append(f"SEED {int(data.get('seed', 42))}")

    walls = data.get("walls")
    if walls is not None:
        lines.append("WALLS %d %s" % (len(walls), " ".join(str(int(w)) for w in walls)))

    stdin_text = "\n".join(lines) + "\n"

    if not os.path.isfile(MAZE_CLI_PATH):
        return jsonify({
            "ok": False,
            "error": f"maze_cli not built - run tools/maze_cli/build.sh (looked for {MAZE_CLI_PATH})",
        }), 500

    try:
        proc = subprocess.run([MAZE_CLI_PATH], input=stdin_text, capture_output=True,
                               text=True, timeout=15)
    except subprocess.TimeoutExpired:
        return jsonify({"ok": False, "error": "maze_cli timed out"}), 500

    try:
        result = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return jsonify({
            "ok": False,
            "error": "maze_cli produced invalid output",
            "stdout": proc.stdout,
            "stderr": proc.stderr,
        }), 500

    return jsonify(result)


@app.route("/api/mms/launch", methods=["POST"])
def mms_launch():
    if not os.path.isfile(MMS_APPIMAGE_PATH):
        return jsonify({"ok": False, "error": f"AppImage not found at {MMS_APPIMAGE_PATH}"}), 404
    try:
        subprocess.Popen([MMS_APPIMAGE_PATH], start_new_session=True)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


if __name__ == "__main__":
    print(f"maze_cli: {MAZE_CLI_PATH} ({'found' if os.path.isfile(MAZE_CLI_PATH) else 'MISSING - run tools/maze_cli/build.sh'})")
    print(f"mms AppImage: {MMS_APPIMAGE_PATH} ({'found' if os.path.isfile(MMS_APPIMAGE_PATH) else 'MISSING'})")
    print("Dashboard: http://127.0.0.1:5055")
    app.run(host="127.0.0.1", port=5055, threaded=True, debug=False)
