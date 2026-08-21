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
PLOT_DEFAULT_MAX_S = 30
CAL_RETRY_INTERVAL_S = 2.0
CAL_ACTION_TIMEOUT_S = 4.0
CAL_MAX_RETRIES = 10


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
        self.max_seconds = PLOT_DEFAULT_MAX_S
        self.data = []
        self.lost = []
        self.plot_start = time.monotonic()
        self.last_t = 0.0

    def set_max_seconds(self, seconds):
        self.max_seconds = max(1, int(seconds))

    def reset(self):
        self.data.clear()
        self.lost.clear()
        self.plot_start = time.monotonic()
        self.last_t = 0.0

    def add(self, value, lost=False):
        t = time.monotonic() - self.plot_start
        self.last_t = t
        self.data.append((t, value, lost))
        min_t = max(0, t - self.max_seconds)
        self.data = [p for p in self.data if p[0] >= min_t]

    def draw(self, state):
        c = self.canvas
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        c.delete("all")
        p = 42
        top = 18
        x0 = max(0, self.last_t - self.max_seconds)
        x1 = max(self.max_seconds, self.last_t)

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


class ScrollableFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, highlightthickness=0, bg="#f4f1e8")
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.window_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.inner.bind("<Configure>", self._on_inner_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _on_inner_configure(self, _event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfigure(self.window_id, width=event.width)

    def _on_mousewheel(self, event):
        if self.winfo_toplevel().focus_get() is not None:
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")


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
        self.current_page = "dashboard"
        self.calibration_active = False
        self.servo_calibration_active = False
        self.cal_state = {}
        self.cal_last_step = ""
        self.cal_button_layout_key = None
        self.verify_view = "both"
        self.verify_arrows_visible = False
        self.cal_pending_action_until = 0.0
        self.cal_pending_action = None
        self.cal_pending_req = 0
        self.cal_next_req = 1
        self.cal_pending_payload = None
        self.cal_pending_start_step = ""
        self.cal_pending_last_send = 0.0
        self.cal_pending_attempts = 0
        self.cal_last_action_response = None
        self.servo_state = {}
        self.plots_layout_narrow = None

        self._build()
        self._refresh_ports()
        self._poll()

    def _build(self):
        self.root.configure(bg="#f4f1e8")
        style = ttk.Style()
        style.configure("Run.TButton", foreground="#0b6b2b")
        style.configure("Stop.TButton", foreground="#9b1c1c")
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

        nav = ttk.Frame(self.root, padding=(8, 0, 8, 4))
        nav.pack(fill="x")
        self.nav_buttons = {}
        for name, label in (
            ("dashboard", "Dashboard"),
            ("calibration", "Calibrations"),
            ("advanced", "Advanced parameters"),
            ("more", "More"),
        ):
            btn = ttk.Button(nav, text=label, command=lambda n=name: self._show_page(n))
            btn.pack(side="left", padx=(0, 6))
            self.nav_buttons[name] = btn

        self.welcome = ttk.LabelFrame(self.root, text="Bienvenue", padding=10)
        self.welcome.pack(fill="x", padx=8, pady=4)
        ttk.Label(
            self.welcome,
            text="Merci d'avoir participe au workshop Balancier PID! Connectez-vous par USB pour piloter la table.",
            wraplength=900,
        ).pack(anchor="w")

        self.scroller = ScrollableFrame(self.root)
        self.scroller.pack(fill="both", expand=True)
        self.body = ttk.Frame(self.scroller.inner, padding=8)
        self.body.pack(fill="both", expand=True)
        self.pages = {}
        self._build_dashboard_page()
        self._build_calibration_page()
        self._build_advanced_page()
        self._build_more_page()
        self._show_page("dashboard")

    def _build_dashboard_page(self):
        main = ttk.Frame(self.body)
        self.pages["dashboard"] = main

        wifi = ttk.LabelFrame(main, text="WiFi name", padding=8)
        wifi.pack(fill="x", pady=(0, 8))
        self.ssid_var = tk.StringVar()
        ttk.Entry(wifi, textvariable=self.ssid_var, width=32).pack(side="left")
        ttk.Button(wifi, text="Save WiFi name", command=self._save_wifi_name).pack(side="left", padx=6)

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
            canvas.grid(row=0, column=i, sticky="nsew", padx=4, pady=2)
            self.plots_frame.columnconfigure(i, weight=1)
        self.plots_frame.rowconfigure(0, weight=1)
        self.root.bind("<Configure>", self._layout_plots)

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
        self.neutral_btn = ttk.Button(manual, text="Neutral pos", command=self._neutral_position)
        self.neutral_btn.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(0, 5))
        self.manual_minus_btn = ttk.Button(manual, text="-", command=lambda: self._manual_step(-1))
        self.manual_minus_btn.grid(row=2, column=0)
        self.angle_var = tk.StringVar(value="--")
        ttk.Label(manual, textvariable=self.angle_var, width=8, anchor="center").grid(row=2, column=1)
        self.manual_plus_btn = ttk.Button(manual, text="+", command=lambda: self._manual_step(1))
        self.manual_plus_btn.grid(row=2, column=2)

        tune = ttk.LabelFrame(bottom, text="Alpha / Beta", padding=8)
        tune.pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.tune_vars = {}
        self._tune_row(tune, "min alpha", "ab_min_alpha", 0)
        self._tune_row(tune, "max alpha", "ab_max_alpha", 1)
        self._tune_row(tune, "min beta", "ab_min_beta", 2)
        self._tune_row(tune, "max beta", "ab_max_beta", 3)
        ttk.Button(tune, text="Save alpha/beta", command=self._save_tune).grid(row=4, column=0, columnspan=4, pady=5)

    def _build_calibration_page(self):
        page = ttk.Frame(self.body)
        self.pages["calibration"] = page

        self.calibration_views = {}

        menu = ttk.Frame(page)
        self.calibration_views["menu"] = menu
        intro = ttk.LabelFrame(menu, text="Calibrations", padding=10)
        intro.pack(fill="x")
        ttk.Label(
            intro,
            text="Choisissez le type de calibration a effectuer.",
            wraplength=1000,
        ).pack(anchor="w", pady=(0, 8))
        ttk.Button(intro, text="Calibrer les TOFs", command=lambda: self._show_calibration_view("tof_menu")).pack(side="left", padx=(0, 8))
        ttk.Button(intro, text="Calibrer le servomoteur", command=self._servo_calibration_start).pack(side="left")

        tof_menu = ttk.Frame(page)
        self.calibration_views["tof_menu"] = tof_menu
        tof = ttk.LabelFrame(tof_menu, text="Calibration TOF et bruit", padding=10)
        tof.pack(fill="x")
        ttk.Label(
            tof,
            text="Selectionnez une procedure TOF ou noise rejection.",
            wraplength=1000,
        ).pack(anchor="w", pady=(0, 8))
        for label, cmd in (
            ("Calibrate TOF1", lambda: self._calibration_start(target=1)),
            ("Calibrate TOF2", lambda: self._calibration_start(target=2)),
            ("Verify calibration", lambda: self._calibration_start(mode="verify")),
            ("Noise rejection", lambda: self._calibration_start(mode="noise")),
            ("Verify noise rejection", lambda: self._calibration_start(mode="noise_result")),
        ):
            ttk.Button(tof, text=label, command=cmd).pack(side="left", padx=(0, 6), pady=(0, 8))
        ttk.Button(tof_menu, text="Retour", command=lambda: self._show_calibration_view("menu")).pack(anchor="w", pady=8)

        tof_process = ttk.Frame(page)
        self.calibration_views["tof_process"] = tof_process
        ttk.Button(tof_process, text="Retour", command=self._back_to_tof_menu).pack(anchor="w", pady=(0, 8))
        scene_frame = ttk.Frame(tof_process)
        scene_frame.pack(fill="x", pady=8)
        self.verify_prev_btn = ttk.Button(scene_frame, text="<", width=4, command=lambda: self._cycle_verify_view(-1))
        self.verify_prev_btn.pack(side="left", fill="y", padx=(0, 6))
        self.cal_scene = tk.Canvas(scene_frame, height=260, bg="#fffdf6", highlightthickness=2, highlightbackground="#202020")
        self.cal_scene.pack(side="left", fill="x", expand=True)
        self.verify_next_btn = ttk.Button(scene_frame, text=">", width=4, command=lambda: self._cycle_verify_view(1))
        self.verify_next_btn.pack(side="left", fill="y", padx=(6, 0))
        self.verify_prev_btn.pack_forget()
        self.verify_next_btn.pack_forget()
        self.cal_title = tk.StringVar(value="No calibration running")
        self.cal_instruction = tk.StringVar(value="Choose a calibration mode.")
        ttk.Label(tof_process, textvariable=self.cal_title, font=("Arial", 13, "bold")).pack(anchor="w")
        ttk.Label(tof_process, textvariable=self.cal_instruction, wraplength=1000).pack(anchor="w", pady=(2, 8))
        self.cal_value = tk.StringVar()
        value_row = ttk.Frame(tof_process)
        self.cal_action_row = value_row
        value_row.pack(fill="x", pady=(0, 8))
        self.cal_value_label = ttk.Label(value_row, text="FOV")
        self.cal_value_entry = ttk.Entry(value_row, textvariable=self.cal_value, width=10)
        self.cal_buttons = {
            "done": ttk.Button(value_row, text="Done", command=lambda: self._calibration_action("done")),
            "submit": ttk.Button(value_row, text="Submit FOV", command=lambda: self._calibration_action("real_fov", value=self.cal_value.get())),
            "accept": ttk.Button(value_row, text="Accept", command=self._calibration_accept_or_done),
            "tof1": ttk.Button(value_row, text="Calibrate TOF1", command=lambda: self._calibration_action("calibrate_tof", value=1)),
            "tof2": ttk.Button(value_row, text="Calibrate TOF2", command=lambda: self._calibration_action("calibrate_tof", value=2)),
            "noise": ttk.Button(value_row, text="Go to noise rejection", command=lambda: self._calibration_action("go_noise")),
            "restart": ttk.Button(value_row, text="Restart", command=lambda: self._calibration_action("restart")),
            "cancel": ttk.Button(value_row, text="Cancel", command=self._back_to_tof_menu),
        }
        self.cal_status = tk.StringVar(value="")
        ttk.Label(tof_process, textvariable=self.cal_status, wraplength=1000).pack(anchor="w")

        servo_page = ttk.Frame(page)
        self.calibration_views["servo"] = servo_page
        ttk.Button(servo_page, text="Retour", command=self._back_to_calibration_menu).pack(anchor="w", pady=(0, 8))
        servo = ttk.LabelFrame(servo_page, text="Servo calibration", padding=8)
        servo.pack(fill="x", pady=(12, 0))
        self.servo_pwm = tk.StringVar(value="1500")
        pwm_row = ttk.Frame(servo)
        pwm_row.pack(fill="x", pady=(0, 8))
        ttk.Button(pwm_row, text="-", width=4, command=lambda: self._servo_pwm_step(-1)).pack(side="left")
        ttk.Label(pwm_row, textvariable=self.servo_pwm, width=8, anchor="center", font=("Arial", 14, "bold")).pack(side="left", padx=6)
        ttk.Button(pwm_row, text="+", width=4, command=lambda: self._servo_pwm_step(1)).pack(side="left")
        self.servo_vars = {}
        servo_fields = [
            ("min", "Angle at 1000 us"),
            ("max", "Angle at 2000 us"),
            ("limit_min", "Min allowed angle"),
            ("limit_max", "Max allowed angle"),
            ("offset", "Neutral offset us"),
            ("step", "PWM step us"),
        ]
        form = ttk.Frame(servo)
        form.pack(fill="x")
        for i, (key, label) in enumerate(servo_fields):
            row, col = divmod(i, 2)
            ttk.Label(form, text=label).grid(row=row, column=col * 2, sticky="w", padx=(0, 5), pady=2)
            var = tk.StringVar()
            ttk.Entry(form, textvariable=var, width=12).grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 20), pady=2)
            self.servo_vars[key] = var
        servo_buttons = ttk.Frame(servo)
        servo_buttons.pack(fill="x", pady=(8, 0))
        ttk.Button(servo_buttons, text="Neutral pos", command=lambda: self._servo_action("neutral")).pack(side="left", padx=(0, 6))
        ttk.Button(servo_buttons, text="Set pos offset", command=lambda: self._servo_action("offset")).pack(side="left", padx=(0, 6))
        ttk.Button(servo_buttons, text="Preview", command=lambda: self._servo_action("preview")).pack(side="left", padx=(0, 6))
        ttk.Button(servo_buttons, text="Save servo", command=lambda: self._servo_action("save")).pack(side="left")
        self.servo_status = tk.StringVar(value="Click Servo calibration to start.")
        ttk.Label(servo, textvariable=self.servo_status, wraplength=1000).pack(anchor="w", pady=(6, 0))

        self._show_calibration_view("menu")

    def _build_advanced_page(self):
        page = ttk.Frame(self.body)
        self.pages["advanced"] = page
        grid = ttk.LabelFrame(page, text="Advanced parameters", padding=8)
        grid.pack(fill="x")
        self.advanced_vars = {}
        fields = [
            ("max_speed", "Max control speed"),
            ("ab_min_alpha", "Min alpha"),
            ("ab_max_alpha", "Max alpha"),
            ("ab_min_beta", "Min beta"),
            ("ab_max_beta", "Max beta"),
            ("ctrl_period", "Controller period ms"),
            ("max_step", "Max servo step deg"),
            ("pos_db", "Position deadband mm"),
            ("speed_db", "Speed deadband mm/s"),
            ("stable_time", "Stable time ms"),
            ("idle_exit", "Idle exit percent"),
            ("lost_delay", "Lost ball delay ms"),
            ("lost_iter", "Lost ball iter"),
            ("servo_min", "Servo min deg"),
            ("servo_max", "Servo max deg"),
            ("servo_step", "Servo PWM step us"),
            ("manual_step", "Manual angle step deg"),
            ("table_len", "Table length mm"),
            ("plot_max", "Plot max s"),
        ]
        for i, (key, label) in enumerate(fields):
            row, col = divmod(i, 2)
            ttk.Label(grid, text=label).grid(row=row, column=col * 2, sticky="w", padx=(0, 5), pady=2)
            var = tk.StringVar()
            ttk.Entry(grid, textvariable=var, width=14).grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 20), pady=2)
            self.advanced_vars[key] = var
        buttons = ttk.Frame(page)
        buttons.pack(fill="x", pady=8)
        ttk.Button(buttons, text="Load", command=self._load_advanced).pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Apply", command=lambda: self._send_advanced(save=False)).pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Save", command=lambda: self._send_advanced(save=True)).pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Reset to default values", command=self._reset_advanced).pack(side="left")
        self.advanced_status = tk.StringVar(value="")
        ttk.Label(page, textvariable=self.advanced_status).pack(anchor="w")

    def _build_more_page(self):
        page = ttk.Frame(self.body)
        self.pages["more"] = page
        more = ttk.LabelFrame(page, text="More", padding=8)
        more.pack(fill="x", pady=0)
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

    def _show_page(self, name):
        self.current_page = name
        for page in self.pages.values():
            page.pack_forget()
        self.pages[name].pack(fill="both", expand=True)
        if name == "calibration":
            self._show_calibration_view("menu")
        if name == "advanced" and self.connected:
            self._load_advanced()

    def _show_calibration_view(self, name):
        for view in self.calibration_views.values():
            view.pack_forget()
        self.calibration_views[name].pack(fill="both", expand=True)

    def _entry(self, parent, label, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        var = tk.StringVar()
        ttk.Entry(parent, textvariable=var, width=12).grid(row=row, column=1, padx=4)
        return var

    def _layout_plots(self, _event=None):
        if not hasattr(self, "plots_frame"):
            return
        canvases = (self.angle_canvas, self.pos_canvas, self.speed_canvas)
        narrow = self.root.winfo_width() < 900
        if self.plots_layout_narrow == narrow:
            return
        self.plots_layout_narrow = narrow
        for canvas in canvases:
            canvas.grid_forget()
        for i in range(3):
            self.plots_frame.columnconfigure(i, weight=0)
            self.plots_frame.rowconfigure(i, weight=0)
        if narrow:
            for i, canvas in enumerate(canvases):
                canvas.grid(row=i, column=0, sticky="nsew", padx=4, pady=4)
                self.plots_frame.rowconfigure(i, weight=1)
            self.plots_frame.columnconfigure(0, weight=1)
        else:
            for i, canvas in enumerate(canvases):
                canvas.grid(row=0, column=i, sticky="nsew", padx=4, pady=2)
                self.plots_frame.columnconfigure(i, weight=1)
            self.plots_frame.rowconfigure(0, weight=1)

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
            if self.current_page == "calibration" and self.calibration_active:
                self._send({"cmd": "calibration_state"})
                self._retry_pending_calibration_action()
            if self.current_page == "calibration" and self.servo_calibration_active:
                self._send({"cmd": "servo_calibration_state"})
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
        if msg.get("advanced"):
            self._apply_advanced_state(msg)
        if msg.get("calibration"):
            if self.cal_pending_action:
                if msg.get("action_response") and int(msg.get("req", -1)) == self.cal_pending_req:
                    self._finish_pending_calibration_action()
                elif str(msg.get("step", "")) != self.cal_pending_start_step:
                    self._finish_pending_calibration_action()
                else:
                    self._apply_calibration_waiting_status(msg)
                    return
            self._apply_calibration_state(msg)
        if msg.get("servo_calibration"):
            self._apply_servo_calibration_state(msg)
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
        plot_max = int(s.get("plot_max_s", PLOT_DEFAULT_MAX_S))
        self.angle_plot.set_max_seconds(plot_max)
        self.pos_plot.set_max_seconds(plot_max)
        self.speed_plot.set_max_seconds(plot_max)
        stabilization = bool(s.get("stabilization", False))
        self.stab_btn.configure(
            text="Stop stabilization" if stabilization else "Start stabilization",
            style="Stop.TButton" if stabilization else "Run.TButton",
        )
        state = "disabled" if stabilization else "normal"
        self.manual_minus_btn.configure(state=state)
        self.manual_plus_btn.configure(state=state)
        self.neutral_btn.configure(state=state)
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

    def _neutral_position(self):
        if self.state.get("stabilization", False):
            return
        self._send({"cmd": "control", "angle": self.state.get("servo_neutral", 45)})

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

    def _load_advanced(self):
        self.advanced_status.set("Loading...")
        self._send({"cmd": "advanced"})

    def _advanced_payload(self):
        numeric = {}
        for key, var in self.advanced_vars.items():
            text = var.get().strip()
            if text == "":
                continue
            numeric[key] = float(text) if "." in text else int(text)
        return numeric

    def _send_advanced(self, save=False):
        try:
            payload = self._advanced_payload()
        except ValueError:
            messagebox.showwarning("Advanced", "All advanced values must be numeric.")
            return
        payload["cmd"] = "advanced_save" if save else "advanced_set"
        self.advanced_status.set("Saving..." if save else "Applying...")
        self._send(payload)

    def _reset_advanced(self):
        self.advanced_status.set("Resetting...")
        self._send({"cmd": "advanced_reset"})

    def _apply_advanced_state(self, msg):
        mapping = {
            "max_speed": "max_control_speed",
            "ab_min_alpha": "alpha_beta_min_alpha",
            "ab_max_alpha": "alpha_beta_max_alpha",
            "ab_min_beta": "alpha_beta_min_beta",
            "ab_max_beta": "alpha_beta_max_beta",
            "ctrl_period": "controller_period",
            "max_step": "max_step",
            "pos_db": "position_deadband",
            "speed_db": "speed_deadband",
            "stable_time": "stable_time",
            "idle_exit": "idle_exit_percent",
            "lost_delay": "lost_delay",
            "lost_iter": "lost_iter",
            "servo_min": "servo_min",
            "servo_max": "servo_max",
            "servo_step": "servo_step_us",
            "manual_step": "manual_angle_step",
            "table_len": "table_length",
            "plot_max": "plot_max_s",
        }
        for ui_key, state_key in mapping.items():
            if state_key not in msg:
                continue
            value = msg[state_key]
            if isinstance(value, float):
                self.advanced_vars[ui_key].set(f"{value:.4f}" if "ab_" in ui_key else f"{value:.2f}")
            else:
                self.advanced_vars[ui_key].set(str(value))
        if msg.get("ok") is False:
            self.advanced_status.set("Some values were rejected.")
        elif msg.get("saved"):
            self.advanced_status.set("Saved.")
        else:
            self.advanced_status.set("Loaded / applied.")

    def _calibration_start(self, mode=None, target=None):
        self.calibration_active = True
        self.servo_calibration_active = False
        self._show_calibration_view("tof_process")
        self.cal_title.set("Starting calibration...")
        self.cal_instruction.set("Waiting for ESP32 calibration state.")
        self.cal_status.set("")
        msg = {"cmd": "calibration_start"}
        if mode:
            msg["mode"] = mode
        if target:
            msg["target"] = target
        self._send_calibration_command("start", msg)

    def _calibration_action(self, action, value=None):
        self.calibration_active = True
        self.servo_calibration_active = False
        self._show_calibration_view("tof_process")
        msg = {"cmd": "calibration_action", "action": action}
        if value not in (None, ""):
            try:
                msg["value"] = int(float(value))
            except ValueError:
                messagebox.showwarning("Calibration", "Value must be numeric.")
                return
        self._send_calibration_command(action, msg)

    def _send_calibration_command(self, action, msg):
        request_id = self.cal_next_req
        self.cal_next_req += 1
        msg["req"] = request_id
        self.cal_pending_action = action
        self.cal_pending_req = request_id
        self.cal_pending_payload = dict(msg)
        self.cal_pending_start_step = str(self.cal_state.get("step", ""))
        self.cal_pending_last_send = time.monotonic()
        self.cal_pending_action_until = self.cal_pending_last_send + CAL_ACTION_TIMEOUT_S
        self.cal_pending_attempts = 1
        for button in self.cal_buttons.values():
            button.configure(state="disabled")
        self._send(msg)

    def _retry_pending_calibration_action(self):
        if not self.cal_pending_action or not self.cal_pending_payload:
            return
        now = time.monotonic()
        if now - self.cal_pending_last_send < CAL_RETRY_INTERVAL_S:
            return
        if self.cal_pending_attempts >= CAL_MAX_RETRIES:
            self.cal_status.set("No ESP32 acknowledgement. You can try the action again.")
            self._finish_pending_calibration_action(mark_success=False)
            return
        self.cal_pending_last_send = now
        self.cal_pending_action_until = now + CAL_ACTION_TIMEOUT_S
        self.cal_pending_attempts += 1
        self.cal_status.set(f"Waiting for ESP32... retry {self.cal_pending_attempts}")
        self._send(dict(self.cal_pending_payload))

    def _finish_pending_calibration_action(self, mark_success=True):
        if mark_success:
            self.cal_last_action_response = self.cal_pending_action
        self.cal_pending_action = None
        self.cal_pending_req = 0
        self.cal_pending_payload = None
        self.cal_pending_start_step = ""
        self.cal_pending_last_send = 0.0
        self.cal_pending_action_until = 0.0
        self.cal_pending_attempts = 0

    def _apply_calibration_waiting_status(self, msg):
        self.cal_state = msg
        step = str(msg.get("step", ""))
        self.cal_title.set(msg.get("title", self.cal_title.get()))
        self.cal_instruction.set(msg.get("instruction", self.cal_instruction.get()))
        self.cal_status.set(f"Waiting for ESP32 to apply '{self.cal_pending_action}' on step {step}...")

    def _calibration_accept_or_done(self):
        step = str(self.cal_state.get("step", ""))
        just_calibrated = int(self.cal_state.get("default_verify_tof", 0) or 0)
        if step == "verify" and just_calibrated == 0:
            self.calibration_active = False
            self.servo_calibration_active = False
            self._show_calibration_view("tof_menu")
            return
        self._calibration_action("accept")

    def _cycle_verify_view(self, delta):
        views = ["both", "tof1", "tof2"]
        current = self.verify_view if self.verify_view in views else "both"
        self.verify_view = views[(views.index(current) + delta) % len(views)]
        if self.cal_state:
            self._draw_calibration_scene(self.cal_state)

    def _servo_calibration_start(self):
        self.servo_calibration_active = True
        self.calibration_active = False
        self._show_calibration_view("servo")
        self.cal_title.set("Servo calibration")
        self.cal_instruction.set("Placez le servo a 1500 us, ajustez le PWM, puis enregistrez l'offset et les limites.")
        self.cal_status.set("")
        self._send({"cmd": "servo_calibration_start", "initial": False})

    def _back_to_tof_menu(self):
        if self.calibration_active:
            self._send({"cmd": "calibration_action", "action": "cancel"})
        self.calibration_active = False
        self.servo_calibration_active = False
        self.cal_title.set("No calibration running")
        self.cal_instruction.set("Choose a calibration mode.")
        self.cal_status.set("")
        self._show_calibration_view("tof_menu")

    def _back_to_calibration_menu(self):
        self.calibration_active = False
        self.servo_calibration_active = False
        self._show_calibration_view("menu")

    def _servo_payload(self):
        payload = {}
        for key, var in self.servo_vars.items():
            text = var.get().strip()
            if text:
                payload[key] = int(float(text))
        return payload

    def _servo_action(self, action):
        self.servo_calibration_active = True
        self.calibration_active = False
        try:
            payload = self._servo_payload()
        except ValueError:
            messagebox.showwarning("Servo", "Servo calibration values must be numeric.")
            return
        payload["cmd"] = "servo_calibration_action"
        payload["action"] = action
        self._send(payload)

    def _servo_pwm_step(self, direction):
        self.servo_calibration_active = True
        self.calibration_active = False
        try:
            step = int(float(self.servo_vars["step"].get() or 10))
            pwm = int(float(self.servo_pwm.get() or 1500)) + direction * step
        except ValueError:
            messagebox.showwarning("Servo", "PWM and step must be numeric.")
            return
        pwm = max(1000, min(2000, pwm))
        self.servo_pwm.set(str(pwm))
        self._send({"cmd": "servo_calibration_action", "action": "pwm", "value": pwm})

    def _apply_calibration_state(self, msg):
        previous_step = str(self.cal_state.get("step", ""))
        self.cal_state = msg
        step = str(msg.get("step", ""))
        if step == "verify" and previous_step != "verify":
            default_tof = int(msg.get("default_verify_tof", 0) or 0)
            self.verify_view = "tof1" if default_tof == 1 else "tof2" if default_tof == 2 else "both"
        elif step != "verify":
            self.verify_view = "both"
        self._set_verify_arrows_visible(step == "verify")
        self.cal_title.set(msg.get("title", "Calibration"))
        self.cal_instruction.set(msg.get("instruction", ""))
        self._update_calibration_buttons(msg)
        status = []
        if "step" in msg:
            status.append(f"Step: {msg['step']}")
        tof_number = int(msg.get("tof", 0) or 0)
        if tof_number in (1, 2):
            raw_text = f"{msg.get('raw_mm', '--')} mm" if msg.get("raw_valid", False) else "invalid"
            status.append(f"TOF {tof_number} measured distance: {raw_text}")
        if "x" in msg:
            status.append(f"x={msg['x']} mm")
        if "d1" in msg and "d2" in msg:
            status.append(f"d1={msg['d1']} mm | d2={msg['d2']} mm")
        if msg.get("error"):
            status.append(str(msg["error"]))
        self.cal_status.set(" | ".join(status))
        self._draw_calibration_scene(msg)
        if msg.get("done") and self.cal_last_action_response == "accept":
            self.calibration_active = False
            self.servo_calibration_active = False
            self._show_calibration_view("tof_menu")
        self.cal_last_action_response = None

    def _set_verify_arrows_visible(self, visible):
        if visible == self.verify_arrows_visible:
            return
        self.verify_arrows_visible = visible
        if visible:
            self.verify_prev_btn.pack(side="left", fill="y", padx=(0, 6), before=self.cal_scene)
            self.verify_next_btn.pack(side="left", fill="y", padx=(6, 0))
        else:
            self.verify_prev_btn.pack_forget()
            self.verify_next_btn.pack_forget()

    def _update_calibration_buttons(self, msg):
        for button in self.cal_buttons.values():
            button.configure(state="normal")

        step = str(msg.get("step", ""))
        needs_real = bool(msg.get("needs_real_input", False)) or step.endswith("_real_fov")
        needs_done = bool(msg.get("needs_done", False))
        verify = step == "verify"
        noise_step = step.startswith("noise")
        noise_done = step == "noise_done"
        step_changed = step != self.cal_last_step
        self.cal_last_step = step
        just_calibrated = int(msg.get("default_verify_tof", 0))
        visible = []

        if needs_real:
            visible.extend(["value_label", "value_entry"])
            if step_changed or not self.cal_value.get():
                self.cal_value.set(str(msg.get("real_fov", 145)))
            visible.append("submit")

        if needs_done:
            self.cal_buttons["done"].configure(text="Capture bruit" if noise_step and not noise_done else "Done")
            visible.append("done")

        if verify:
            self.cal_buttons["accept"].configure(text="Accept" if just_calibrated else "Done")
            visible.append("accept")
            if just_calibrated != 1:
                visible.append("tof1")
            if just_calibrated != 2:
                visible.append("tof2")
            visible.append("noise")

        if noise_done:
            self.cal_buttons["accept"].configure(text="Accept")
            visible.append("accept")

        show_restart_cancel = not verify or just_calibrated != 0
        if show_restart_cancel:
            visible.extend(["restart", "cancel"])

        layout_key = tuple(visible)
        if layout_key == self.cal_button_layout_key:
            return

        self.cal_button_layout_key = layout_key
        for widget in (self.cal_value_label, self.cal_value_entry, *self.cal_buttons.values()):
            widget.pack_forget()

        for name in visible:
            if name == "value_label":
                self.cal_value_label.pack(side="left")
            elif name == "value_entry":
                self.cal_value_entry.pack(side="left", padx=5)
            else:
                self.cal_buttons[name].pack(side="left", padx=(0, 5))

    def _apply_servo_calibration_state(self, msg):
        self.servo_state = msg
        mapping = {
            "min": "theoretical_min_angle",
            "max": "theoretical_max_angle",
            "limit_min": "limit_min_angle",
            "limit_max": "limit_max_angle",
            "offset": "neutral_offset_us",
            "step": "pwm_step_us",
        }
        for ui_key, state_key in mapping.items():
            if state_key in msg:
                self.servo_vars[ui_key].set(str(msg[state_key]))
        self.servo_pwm.set(str(msg.get("current_pwm_us", self.servo_pwm.get())))
        self._draw_servo_scene(msg)
        status = (
            f"Servo={msg.get('current_angle', '--')} deg | "
            f"Neutral={msg.get('neutral_angle', '--')} deg | "
            f"PWM={msg.get('current_pwm_us', '--')} us"
        )
        if msg.get("error"):
            status += f" | {msg['error']}"
        self.servo_status.set(status)

    def _draw_calibration_scene(self, msg):
        c = self.cal_scene
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        c.delete("all")
        table_len = max(1, int(msg.get("table_length", self.state.get("table_length", 290))))
        step = str(msg.get("step", ""))
        verify = step == "verify"
        noise_step = step.startswith("noise")
        tof_number = int(msg.get("tof", 0) or 0)
        focus_tof = 1 if verify and self.verify_view == "tof1" else 2 if verify and self.verify_view == "tof2" else 0
        x1, x2, y = w * 0.15, w * 0.85, h * 0.52
        c.create_line(x1, y, x2, y, width=4)
        c.create_polygon(w * 0.5, y + 8, w * 0.46, y + h * 0.20, w * 0.54, y + h * 0.20, outline="#171717", fill="", width=3)

        if verify:
            label = "TOF 1 + TOF 2" if self.verify_view == "both" else "TOF 1 only" if self.verify_view == "tof1" else "TOF 2 only"
            if focus_tof:
                label_x = 18 if focus_tof == 2 else w - 18
                anchor = "w" if focus_tof == 2 else "e"
                c.create_text(label_x, 18, text=label, anchor=anchor, fill="#2457b8", font=("Arial", 10, "bold"))
            else:
                c.create_text(w * 0.5, 22, text=label, fill="#2457b8", font=("Arial", 13, "bold"))
            for mm in (0, 72, 145, 218, 290):
                p = max(0, min(table_len, mm)) / table_len
                x = x1 + (x2 - x1) * p
                c.create_line(x, y + 24, x, y + 58, fill="#208444", width=3)
                c.create_polygon(x, y + 18, x - 8, y + 36, x + 8, y + 36, fill="#208444", outline="#208444")
                c.create_text(x, y + 78, text=str(mm), fill="#208444")

        if tof_number in (1, 2):
            arrow_x = x2 if tof_number == 1 else x1
            direction = -1 if tof_number == 1 else 1
            c.create_text(arrow_x, y - 92, text=f"TOF {tof_number}", fill="#2457b8", font=("Arial", 13, "bold"))
            c.create_line(arrow_x, y - 78, arrow_x + direction * 70, y - 36, fill="#2457b8", width=4, arrow=tk.LAST)

        if focus_tof == 1:
            live = msg.get("tof1_pos_mm", -1)
        elif focus_tof == 2:
            live = msg.get("tof2_pos_mm", -1)
        else:
            live = msg.get("visual_pos_mm", msg.get("x", -1))
        target = msg.get("visual_target_mm", -1)
        if verify and focus_tof in (1, 2):
            fov = msg.get("tof1_fov_pos_mm" if focus_tof == 1 else "tof2_fov_pos_mm", -1)
            if isinstance(fov, (int, float)) and fov >= 0:
                fx = x1 + (x2 - x1) * (max(0, min(table_len, fov)) / table_len)
                c.create_line(fx, y - 74, fx, y - 16, fill="#8a4f00", dash=(7, 4), width=3)
                c.create_text(fx, y - 88, text=f"FOV {int(fov)} mm", fill="#8a4f00", font=("Arial", 11, "bold"))
            pos_text = f"{int(live)} mm" if isinstance(live, (int, float)) and live >= 0 else "lost"
            label_x = 18 if focus_tof == 2 else w - 18
            anchor = "w" if focus_tof == 2 else "e"
            c.create_text(label_x, 36, text=f"position: {pos_text}", anchor=anchor, fill="#2457b8", font=("Arial", 10, "bold"))
        if noise_step and isinstance(target, (int, float)) and target >= 0:
            tx = x1 + (x2 - x1) * (max(0, min(table_len, target)) / table_len)
            c.create_line(tx, y + 20, tx, y + 60, fill="#2457b8", dash=(6, 4), width=3)
            c.create_text(tx, y + 82, text=f"target {int(target)}", fill="#2457b8")

        raw_valid = bool(msg.get("raw_valid", True))
        ball_lost = not isinstance(live, (int, float)) or live < 0 or (tof_number in (1, 2) and not raw_valid and not verify)
        if not ball_lost:
            bx = x1 + (x2 - x1) * (max(0, min(table_len, live)) / table_len)
            c.create_oval(bx - 16, y - 42, bx + 16, y - 10, outline="#171717", width=3)
        else:
            bx = (x1 + x2) * 0.5
            c.create_oval(bx - 16, y - 42, bx + 16, y - 10, outline="#c43131", dash=(6, 4), width=3)
            c.create_text(w * 0.5, y - 58, text="Ball lost", fill="#c43131", font=("Arial", 13, "bold"))

    def _draw_servo_scene(self, msg):
        c = self.cal_scene
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        c.delete("all")
        current = float(msg.get("current_angle", 45))
        neutral = float(msg.get("neutral_angle", 45))
        table_deg = current - neutral
        a = -math.radians(table_deg)
        cx, cy, length = w * 0.5, h * 0.56, w * 0.72
        ux, uy = math.cos(a), math.sin(a)
        x1, y1 = cx - ux * length / 2, cy - uy * length / 2
        x2, y2 = cx + ux * length / 2, cy + uy * length / 2
        c.create_line(x1, y1, x2, y2, width=4)
        c.create_polygon(cx, cy + 8, cx - w * 0.035, cy + h * 0.22, cx + w * 0.035, cy + h * 0.22, outline="#171717", fill="", width=3)
        c.create_text(18, h - 76, text=f"servo={current:.0f} deg", anchor="w", font=("Arial", 12))
        c.create_text(18, h - 24, text=f"table={table_deg:.1f} deg", anchor="w", font=("Arial", 12))


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1180x880")
    app = PIDTableApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.link.close(), root.destroy()))
    root.mainloop()
