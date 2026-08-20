import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


REFRESH_MS = 60
PLOT_MAX_S = 30


class SerialLink:
    def __init__(self, on_message):
        self.on_message = on_message
        self.ser = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()

    def ports(self):
        if list_ports is None:
            return []
        return [p.device for p in list_ports.comports()]

    def open(self, port, baud=115200):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install it with: pip install pyserial")
        self.close(send_disconnect=False)
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def close(self, send_disconnect=True):
        if self.ser and send_disconnect:
            try:
                self.send({"cmd": "disconnect"})
            except Exception:
                pass
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.thread = None

    def send(self, obj):
        if not self.ser or not self.ser.is_open:
            return
        line = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
        with self.lock:
            self.ser.write(line)

    def _reader(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            try:
                text = raw.decode("utf-8", errors="ignore").strip()
                if not text.startswith("{"):
                    continue
                self.on_message(json.loads(text))
            except Exception:
                continue


class Plot:
    def __init__(self, canvas, color, label):
        self.canvas = canvas
        self.color = color
        self.label = label
        self.data = []
        self.lost = []
        self.plot_start = time.monotonic()
        self.last_t = 0.0

    def reset(self):
        self.data.clear()
        self.lost.clear()
        self.plot_start = time.monotonic()
        self.last_t = 0.0

    def add(self, value, lost=False):
        t = time.monotonic() - self.plot_start
        self.last_t = t
        self.data.append((t, value, lost))
        min_t = max(0, t - PLOT_MAX_S)
        self.data = [p for p in self.data if p[0] >= min_t]

    def draw(self, state):
        c = self.canvas
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        c.delete("all")
        p = 42
        top = 18
        x0 = max(0, self.last_t - PLOT_MAX_S)
        x1 = max(PLOT_MAX_S, self.last_t)

        vals = [v for t, v, lost in self.data if not lost and isinstance(v, (int, float)) and math.isfinite(v)]
        if self.label == "pos":
            ymin, ymax = 0, max(1, state.get("table_length", 290))
        elif self.label == "speed":
            m = max([100] + [abs(v) for v in vals])
            ymax = math.ceil(m / 50) * 50
            ymin = -ymax
        else:
            ymin = state.get("servo_theoretical_min", 0)
            ymax = state.get("servo_theoretical_max", 180)
            if ymax <= ymin:
                ymin, ymax = 0, 180

        def xp(t):
            return p + (t - x0) / max(0.001, x1 - x0) * (w - p - 16)

        def yp(v):
            return h - p - (v - ymin) / max(0.001, ymax - ymin) * (h - p - top)

        c.create_line(p, top, p, h - p, w - 12, h - p, width=3)

        ref = None
        if self.label == "pos":
            ref = (state.get("ref", 145), "x0")
        elif self.label == "speed":
            ref = (0, "0")
        elif self.label == "angle":
            ref = (state.get("servo_neutral", 45), "neutral")
        if ref and ymin <= ref[0] <= ymax:
            y = yp(ref[0])
            c.create_line(p, y, w - 12, y, fill="#666", dash=(7, 5), width=2)
            c.create_text(p + 8, y - 8, text=f"{ref[1]} {ref[0]}", anchor="w", fill="#666")

        last = None
        for t, v, lost in self.data:
            if lost or not isinstance(v, (int, float)) or not math.isfinite(v):
                last = None
                continue
            x, y = xp(t), yp(v)
            if last:
                c.create_line(last[0], last[1], x, y, fill=self.color, width=3)
            last = (x, y)

        c.create_text(p + 8, top + 12, text=f"{self.label} | {round(x0)}-{round(x1)}s", anchor="w")
        window_vals = [v for t, v, lost in self.data if x0 <= t <= x1 and not lost and isinstance(v, (int, float))]
        if window_vals:
            mn, mx = min(window_vals), max(window_vals)
            mean = sum(window_vals) / len(window_vals)
            std = math.sqrt(sum((v - mean) ** 2 for v in window_vals) / len(window_vals))
            unit = "deg" if self.label == "angle" else "mm" if self.label == "pos" else "mm/s"
            parts = [f"min {mn:.1f} {unit}", f"max {mx:.1f} {unit}", f"mean {mean:.1f} {unit}", f"std {std:.1f} {unit}"]
            span = (w - p - 20) / 4
            for i, text in enumerate(parts):
                c.create_text(p + 4 + i * span, h - 12, text=text, anchor="w", fill=self.color, font=("Arial", 9, "bold"))


class PIDTableApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robovaly PID Table - USB Interface")
        self.inbox = queue.Queue()
        self.link = SerialLink(lambda msg: self.inbox.put(msg))
        self.connected = False
        self.state = {}
        self.plot_running = False
        self.tune_draft = None
        self.first_run = True

        self._build()
        self._refresh_ports()
        self._poll()

    def _build(self):
        self.root.configure(bg="#f4f1e8")
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="COM").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.pack(side="left", padx=5)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=5)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=12)

        self.welcome = ttk.LabelFrame(self.root, text="Bienvenue", padding=10)
        self.welcome.pack(fill="x", padx=8, pady=4)
        ttk.Label(
            self.welcome,
            text="Merci d'avoir participe au workshop Balancier PID! Connectez-vous par USB pour piloter la table.",
            wraplength=900,
        ).pack(anchor="w")

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        self.scene = tk.Canvas(main, height=260, bg="#fffdf6", highlightthickness=2, highlightbackground="#202020")
        self.scene.pack(fill="x", pady=(0, 8))

        controls = ttk.Frame(main)
        controls.pack(fill="x")
        self.plot_btn = ttk.Button(controls, text="Go", command=self._toggle_plot)
        self.plot_btn.pack(side="left")

        self.plots_frame = ttk.Frame(main)
        self.plots_frame.pack(fill="both", expand=True, pady=8)
        self.angle_canvas = tk.Canvas(self.plots_frame, height=230, bg="#fffdf6", highlightthickness=2, highlightbackground="#202020")
        self.pos_canvas = tk.Canvas(self.plots_frame, height=230, bg="#fffdf6", highlightthickness=2, highlightbackground="#202020")
        self.speed_canvas = tk.Canvas(self.plots_frame, height=230, bg="#fffdf6", highlightthickness=2, highlightbackground="#202020")
        for i, canvas in enumerate((self.angle_canvas, self.pos_canvas, self.speed_canvas)):
            canvas.grid(row=0, column=i, sticky="nsew", padx=4)
            self.plots_frame.columnconfigure(i, weight=1)

        self.angle_plot = Plot(self.angle_canvas, "#c43131", "angle")
        self.pos_plot = Plot(self.pos_canvas, "#2457b8", "pos")
        self.speed_plot = Plot(self.speed_canvas, "#208444", "speed")

        bottom = ttk.Frame(main)
        bottom.pack(fill="x")
        pid = ttk.LabelFrame(bottom, text="PID", padding=8)
        pid.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.ref = self._entry(pid, "Ref X0", 0)
        self.kp = self._entry(pid, "Kp", 1)
        self.ki = self._entry(pid, "Ki", 2)
        self.kd = self._entry(pid, "Kd", 3)
        ttk.Button(pid, text="Apply", command=self._apply_pid).grid(row=4, column=0, pady=6)
        ttk.Button(pid, text="Save", command=self._save_pid).grid(row=4, column=1, pady=6)

        manual = ttk.LabelFrame(bottom, text="Manual Ctrl", padding=8)
        manual.pack(side="left", fill="x", expand=True, padx=4)
        self.stab_btn = ttk.Button(manual, text="Stabilization", command=self._toggle_stabilization)
        self.stab_btn.grid(row=0, column=0, columnspan=3, pady=4)
        ttk.Button(manual, text="-", command=lambda: self._manual_step(-1)).grid(row=1, column=0)
        self.angle_var = tk.StringVar(value="--")
        ttk.Label(manual, textvariable=self.angle_var, width=8, anchor="center").grid(row=1, column=1)
        ttk.Button(manual, text="+", command=lambda: self._manual_step(1)).grid(row=1, column=2)

        tune = ttk.LabelFrame(bottom, text="Alpha / Beta", padding=8)
        tune.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.tune_vars = {}
        self._tune_row(tune, "min alpha", "ab_min_alpha", 0)
        self._tune_row(tune, "max alpha", "ab_max_alpha", 1)
        self._tune_row(tune, "min beta", "ab_min_beta", 2)
        self._tune_row(tune, "max beta", "ab_max_beta", 3)
        ttk.Button(tune, text="Save alpha/beta", command=self._save_tune).grid(row=4, column=0, columnspan=4, pady=5)

        wifi = ttk.LabelFrame(main, text="WiFi name", padding=8)
        wifi.pack(fill="x", pady=(8, 0))
        self.ssid_var = tk.StringVar()
        ttk.Entry(wifi, textvariable=self.ssid_var, width=32).pack(side="left")
        ttk.Button(wifi, text="Save WiFi name", command=self._save_wifi_name).pack(side="left", padx=6)

        more = ttk.LabelFrame(main, text="More", padding=8)
        more.pack(fill="x", pady=8)
        ttk.Label(
            more,
            text=(
                "Auteur: Jerome ESSOLA ELANGA - etudiant en master de robotique a l'EPFL - 2026 - 2027\n"
                "Release version: Robovaly 2026 - 20.08.2026\n"
                "Github: Jess_lnga\n\n"
                "Merci d'avoir participe au workshop Balancier PID! Ce projet aurait encore besoin "
                "d'ameliorations. Si vous avez des retours ou des suggestions, n'hesitez pas a passer sur le github!"
            ),
            wraplength=1000,
        ).pack(anchor="w")

    def _entry(self, parent, label, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        var = tk.StringVar()
        ttk.Entry(parent, textvariable=var, width=12).grid(row=row, column=1, padx=4)
        return var

    def _tune_row(self, parent, label, key, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        ttk.Button(parent, text="-", width=3, command=lambda: self._step_tune(key, -0.01)).grid(row=row, column=1)
        var = tk.StringVar(value="--")
        self.tune_vars[key] = var
        ttk.Label(parent, textvariable=var, width=6, anchor="center").grid(row=row, column=2)
        ttk.Button(parent, text="+", width=3, command=lambda: self._step_tune(key, 0.01)).grid(row=row, column=3)

    def _refresh_ports(self):
        ports = self.link.ports()
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.connected:
            self.link.send({"cmd": "disconnect"})
            self.link.close(send_disconnect=False)
            self.connected = False
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("COM", "Select a COM port first.")
            return
        try:
            self.link.open(port)
            time.sleep(0.2)
            self.link.send({"cmd": "connect"})
            self.status_var.set("Connecting...")
        except Exception as exc:
            messagebox.showerror("USB", str(exc))

    def _send(self, obj):
        if self.link.ser:
            self.link.send(obj)

    def _poll(self):
        while True:
            try:
                msg = self.inbox.get_nowait()
            except queue.Empty:
                break
            self._handle(msg)
        if self.connected:
            self._send({"cmd": "state"})
        self.root.after(REFRESH_MS, self._poll)

    def _handle(self, msg):
        if msg.get("busy") == "wifi":
            self.status_var.set("WiFi client already connected")
            self.link.close(send_disconnect=False)
            return
        if msg.get("connected") is True:
            self.connected = True
            self.connect_btn.configure(text="Disconnect")
            self.status_var.set("Connected")
            if self.first_run:
                self.first_run = False
                self.welcome.pack_forget()
        if "alpha_beta_min_alpha" in msg and "x" not in msg:
            self.tune_draft = None
            self._apply_tune_state(msg)
        if "x" in msg:
            self.state = msg
            self._update_from_state()

    def _update_from_state(self):
        s = self.state
        self.ref.set(str(s.get("ref", "")))
        self.kp.set(f"{float(s.get('kp', 0)):.3f}")
        self.ki.set(f"{float(s.get('ki', 0)):.3f}")
        self.kd.set(f"{float(s.get('kd', 0)):.3f}")
        self.angle_var.set(str(s.get("servo_angle", "--")))
        self.ssid_var.set(s.get("wifi_ssid", self.ssid_var.get()))
        mapping = {
            "ab_min_alpha": "alpha_beta_min_alpha",
            "ab_max_alpha": "alpha_beta_max_alpha",
            "ab_min_beta": "alpha_beta_min_beta",
            "ab_max_beta": "alpha_beta_max_beta",
        }
        if self.tune_draft is None:
            self._apply_tune_state(s)
        self._draw_scene()
        if self.plot_running:
            lost = s.get("x", -1) < 0 or s.get("controller_valid") is False
            self.angle_plot.add(s.get("servo_angle", math.nan), False)
            self.pos_plot.add(math.nan if lost else s.get("x", math.nan), lost)
            self.speed_plot.add(math.nan if lost or not s.get("speed_valid") else s.get("v", math.nan), lost)
        self.angle_plot.draw(s)
        self.pos_plot.draw(s)
        self.speed_plot.draw(s)

    def _apply_tune_state(self, state):
        mapping = {
            "ab_min_alpha": "alpha_beta_min_alpha",
            "ab_max_alpha": "alpha_beta_max_alpha",
            "ab_min_beta": "alpha_beta_min_beta",
            "ab_max_beta": "alpha_beta_max_beta",
        }
        for key, state_key in mapping.items():
            if state_key in state:
                self.tune_vars[key].set(f"{float(state.get(state_key, 0)):.2f}")

    def _draw_scene(self):
        c = self.scene
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        c.delete("all")
        s = self.state
        table_len = max(1, s.get("table_length", 290))
        neutral = s.get("servo_neutral", 45)
        table_deg = s.get("servo_angle", neutral) - neutral
        a = -math.radians(table_deg)
        cx, cy, length = w * 0.5, h * 0.56, w * 0.70
        ux, uy = math.cos(a), math.sin(a)
        nx, ny = uy, -ux
        x1, y1 = cx - ux * length / 2, cy - uy * length / 2
        x2, y2 = cx + ux * length / 2, cy + uy * length / 2
        c.create_line(x1, y1, x2, y2, width=4)
        c.create_polygon(cx, cy + 8, cx - w * 0.035, cy + h * 0.19, cx + w * 0.035, cy + h * 0.19, outline="#171717", fill="", width=3)
        x = s.get("x", -1)
        p = 0.5 if x < 0 else max(0, min(table_len, x)) / table_len
        r = max(15, min(w, h) * 0.04)
        bx = x1 + (x2 - x1) * p + nx * r
        by = y1 + (y2 - y1) * p + ny * r
        c.create_oval(bx - r, by - r, bx + r, by + r, width=3, dash=(8, 5) if x < 0 else None, outline="#c43131" if x < 0 else "#171717")
        c.create_line(bx - r * 0.65, by - r * 0.65, bx + r * 0.65, by + r * 0.65, width=3)
        c.create_line(bx + r * 0.65, by - r * 0.65, bx - r * 0.65, by + r * 0.65, width=3)
        c.create_text(18, h - 76, text=f"x={x} mm", anchor="w", font=("Arial", 12))
        c.create_text(18, h - 24, text=f"servo={s.get('servo_angle', '--')} deg / table={table_deg:.1f} deg", anchor="w", font=("Arial", 12))

    def _toggle_plot(self):
        self.plot_running = not self.plot_running
        self.plot_btn.configure(text="Stop" if self.plot_running else "Go")
        self.angle_plot.reset()
        self.pos_plot.reset()
        self.speed_plot.reset()

    def _toggle_stabilization(self):
        self._send({"cmd": "control", "stabilization": not self.state.get("stabilization", False)})

    def _manual_step(self, direction):
        step = self.state.get("manual_angle_step", 5)
        angle = self.state.get("servo_angle", self.state.get("servo_neutral", 45)) + direction * step
        self._send({"cmd": "control", "angle": angle})

    def _apply_pid(self):
        self._send({"cmd": "params", "ref": int(float(self.ref.get())), "kp": float(self.kp.get()), "ki": float(self.ki.get()), "kd": float(self.kd.get())})

    def _save_pid(self):
        self._send({"cmd": "params_save", "ref": int(float(self.ref.get())), "kp": float(self.kp.get()), "ki": float(self.ki.get()), "kd": float(self.kd.get())})

    def _step_tune(self, key, delta):
        current = float(self.tune_vars[key].get() if self.tune_vars[key].get() != "--" else 0)
        next_value = round(current + delta, 4)
        if "alpha" in key:
            next_value = max(0, min(1, next_value))
        else:
            next_value = max(0, min(2, next_value))
        self.tune_vars[key].set(f"{next_value:.2f}")
        self.tune_draft = self._current_tune()
        self._reset_plots_after_tune()
        self._send({"cmd": "advanced_set", **self.tune_draft})

    def _current_tune(self):
        return {
            "ab_min_alpha": float(self.tune_vars["ab_min_alpha"].get()),
            "ab_max_alpha": float(self.tune_vars["ab_max_alpha"].get()),
            "ab_min_beta": float(self.tune_vars["ab_min_beta"].get()),
            "ab_max_beta": float(self.tune_vars["ab_max_beta"].get()),
        }

    def _save_tune(self):
        self._send({"cmd": "advanced_save", **self._current_tune()})
        self.tune_draft = None

    def _reset_plots_after_tune(self):
        self.angle_plot.reset()
        self.pos_plot.reset()
        self.speed_plot.reset()

    def _save_wifi_name(self):
        ssid = self.ssid_var.get().strip()
        if not ssid:
            messagebox.showwarning("WiFi", "SSID cannot be empty.")
            return
        self._send({"cmd": "wifi_set", "ssid": ssid})


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1180x880")
    app = PIDTableApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.link.close(), root.destroy()))
    root.mainloop()
