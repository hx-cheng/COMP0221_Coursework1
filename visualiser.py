import json
import math
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.ciphers import algorithms
from cryptography.hazmat.primitives.cmac import CMAC
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from mac_dict import DRONE_NAMES

# =============================================================================

# Configuration

# =============================================================================


BROKER_HOST = "broker.hivemq.com"

BROKER_PORT = 1883

MQTT_TOPIC = "flocksim"


# 100 m cube in millimetres

MAX_POS_MM = 100_000

MAX_SPEED_MM_S = 5000.0


# AES-128 CMAC key (must match publisher)
SHARED_KEY_HEX = ""

SHARED_KEY = bytes.fromhex(SHARED_KEY_HEX)

# =============================================================================

# Helpers for struct packing and CMAC

# =============================================================================


def parse_node_id(node_id_field):
    """

    Convert node_id from JSON into 6 bytes, matching the C struct.



    Accepts:

      - "F0:24:F9:AD:69:F0"

      - "f024f9ad69f0"

      - [240, 36, 249, 173, 105, 240]

    """

    if isinstance(node_id_field, str):

        s = node_id_field.replace(":", "").replace("-", "")

        if len(s) != 12:

            raise ValueError(f"node_id hex string must be 12 characters, got {len(s)}")

        return bytes.fromhex(s)

    elif isinstance(node_id_field, list):

        if len(node_id_field) != 6:

            raise ValueError("node_id list must have 6 elements")

        return bytes(node_id_field)

    else:

        raise TypeError("Unsupported node_id type")


def parse_mac_tag(mac_tag_field):
    """

    Convert mac_tag from JSON into 4 bytes.



    Accepts:

      - "a1b2c3d4"

      - [0xa1, 0xb2, 0xc3, 0xd4]

    """

    if isinstance(mac_tag_field, str):

        s = mac_tag_field.replace(" ", "")

        if len(s) != 8:

            raise ValueError("mac_tag hex string must have 8 characters")

        return bytes.fromhex(s)

    elif isinstance(mac_tag_field, list):

        if len(mac_tag_field) != 4:

            raise ValueError("mac_tag list must have 4 elements")

        return bytes(mac_tag_field)

    else:

        raise TypeError("Unsupported mac_tag type")


def pack_struct_bytes(payload):
    """

    Recreate the byte layout of the C struct up to but excluding mac_tag.



    typedef struct __attribute__((packed)) {

        uint8_t  version;

        uint8_t  team_id;

        uint8_t  node_id[6];

        uint16_t seq_number;

        uint32_t ts_s;

        uint16_t ts_ms;

        uint32_t x_mm;

        uint32_t y_mm;

        uint32_t z_mm;

        int32_t  vx_mm_s;

        int32_t  vy_mm_s;

        int32_t  vz_mm_s;

        uint16_t yaw_cd;

        uint8_t  mac_tag[4];  // excluded from CMAC input

    } YOUR_STRUCT_NAME;

    """

    version = int(payload["version"])

    team_id = int(payload["team_id"])

    node_id_bytes = parse_node_id(payload["node_id"])

    seq_number = int(payload["seq_number"])

    ts_s = int(payload["ts_s"])

    ts_ms = int(payload["ts_ms"])

    x_mm = int(payload["x_mm"])

    y_mm = int(payload["y_mm"])

    z_mm = int(payload["z_mm"])

    vx_mm_s = int(payload["vx_mm_s"])

    vy_mm_s = int(payload["vy_mm_s"])

    vz_mm_s = int(payload["vz_mm_s"])

    yaw_cd = int(payload["yaw_cd"])

    fmt = "<BB6sH I H I I I i i i H"

    packed = struct.pack(

        fmt,

        version,

        team_id,

        node_id_bytes,

        seq_number,

        ts_s,

        ts_ms,

        x_mm,

        y_mm,

        z_mm,

        vx_mm_s,

        vy_mm_s,

        vz_mm_s,

        yaw_cd,

    )

    return packed


def compute_truncated_cmac(data_bytes, key_bytes):
    """

    Compute AES-128 CMAC and return the lower 4 bytes.

    """

    c = CMAC(algorithms.AES(key_bytes), backend=default_backend())

    c.update(data_bytes)

    full_tag = c.finalize()

    return full_tag[-4:]


def verify_cmac(payload):
    """

    Return (ok, reason) given a parsed payload.

    """

    try:

        expected_tag = parse_mac_tag(payload["mac_tag"])

    except Exception as e:

        return False, f"CMAC parse error: {e}"

    try:

        struct_bytes = pack_struct_bytes(payload)

    except Exception as e:

        return False, f"Struct packing error: {e}"

    try:

        computed_tag = compute_truncated_cmac(struct_bytes, SHARED_KEY)

    except Exception as e:

        return False, f"CMAC compute error: {e}"

    if computed_tag != expected_tag:

        return False, "CMAC mismatch"

    return True, ""


def check_position(payload):
    """

    Check if x_mm, y_mm, z_mm are inside the 100 m cube.

    """

    try:

        x = int(payload["x_mm"])

        y = int(payload["y_mm"])

        z = int(payload["z_mm"])

    except Exception as e:

        return False, f"Position error: {e}"

    for axis, value in (("x", x), ("y", y), ("z", z)):

        if value < 0 or value > MAX_POS_MM:

            return False, f"{axis}_mm out of bounds: {value} mm"

    return True, ""


def check_speed(payload):
    """

    Check speed magnitude does not exceed MAX_SPEED_MM_S.

    """

    try:

        vx = float(payload["vx_mm_s"])

        vy = float(payload["vy_mm_s"])

        vz = float(payload["vz_mm_s"])

    except Exception as e:

        return False, f"Velocity error: {e}"

    speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    if speed > MAX_SPEED_MM_S:

        return False, f"Speed {speed:.1f} mm/s exceeds limit {MAX_SPEED_MM_S:.1f} mm/s"

    return True, ""


# =============================================================================

# GUI with 3D visualisation

# =============================================================================


class FlockSim3DGUI:

    def __init__(self, root):

        self.root = root

        self.root.title("COMP0221 FlockSim Visualiser (3D)")

        # Queue from MQTT thread

        self.event_queue = queue.Queue()

        # node_id -> latest state

        # state fields: x_mm, y_mm, z_mm, yaw_cd, vx_mm_s, vy_mm_s, vz_mm_s

        self.node_states = {}

        # node_id -> name

        self.node_names = {}

        # node_id -> colour

        self.node_colours = {}

        # node_id -> text artist

        self.node_annotations = {}

        self.colour_palette = [

            "tab:blue",

            "tab:orange",

            "tab:green",

            "tab:red",

            "tab:purple",

            "tab:brown",

            "tab:pink",

            "tab:gray",

            "tab:olive",

            "tab:cyan",

        ]

        self.colour_index = 0

        # Track flock toggle

        self.track_flock = tk.BooleanVar(value=False)

        # Arrow mode: "yaw" or "velocity"

        self.arrow_mode = tk.StringVar(value="yaw")

        main_frame = ttk.Frame(root)

        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left: 3D figure

        fig_frame = ttk.Frame(main_frame)

        fig_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(5, 5))

        self.ax = self.fig.add_subplot(111, projection="3d")

        self.ax.set_xlabel("X (m)")

        self.ax.set_ylabel("Y (m)")

        self.ax.set_zlabel("Z (m)")

        max_m = MAX_POS_MM / 1000.0

        self.max_m = max_m

        self.ax.set_xlim(0.0, max_m)

        self.ax.set_ylim(0.0, max_m)

        self.ax.set_zlim(0.0, max_m)

        # Save home view and limits

        self.home_limits = (0.0, max_m, 0.0, max_m, 0.0, max_m)

        self.ax.view_init(elev=25.0, azim=-60.0)

        self.home_view = (self.ax.elev, self.ax.azim)

        self.canvas = FigureCanvasTkAgg(self.fig, master=fig_frame)

        self.canvas_widget = self.canvas.get_tk_widget()

        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        # Enable scroll wheel zoom

        self.canvas.mpl_connect("scroll_event", self.on_scroll)

        # Handle to current quiver artist

        self.quiver = None

        # Right: controls and text logs

        side_frame = ttk.Frame(main_frame)

        side_frame.pack(side=tk.RIGHT, fill=tk.BOTH)

        # Controls row (buttons, track tick box, arrow mode dropdown)

        controls_frame = ttk.Frame(side_frame)

        controls_frame.pack(fill=tk.X, pady=(4, 4))

        home_button = ttk.Button(controls_frame, text="Home", command=self.reset_view)

        home_button.grid(row=0, column=0, padx=2, pady=2, sticky="w")

        top_button = ttk.Button(controls_frame, text="Top", command=self.set_top_view)

        top_button.grid(row=0, column=1, padx=2, pady=2, sticky="w")

        front_button = ttk.Button(controls_frame, text="Front", command=self.set_front_view)

        front_button.grid(row=0, column=2, padx=2, pady=2, sticky="w")

        side_button = ttk.Button(controls_frame, text="Side", command=self.set_side_view)

        side_button.grid(row=0, column=3, padx=2, pady=2, sticky="w")

        track_cb = ttk.Checkbutton(

            controls_frame,

            text="Track flock",

            variable=self.track_flock,

            command=self.on_track_toggle,

        )

        track_cb.grid(row=0, column=4, padx=(10, 2), pady=2, sticky="w")

        mode_label = ttk.Label(controls_frame, text="Arrow:")

        mode_label.grid(row=0, column=5, padx=(10, 2), pady=2, sticky="e")

        self.mode_combo = ttk.Combobox(

            controls_frame,

            state="readonly",

            values=["Yaw", "Velocity"],

            width=9,

        )

        self.mode_combo.grid(row=0, column=6, padx=2, pady=2, sticky="w")

        self.mode_combo.set("Yaw")

        def on_mode_selected(event=None):

            selection = self.mode_combo.get()

            self.arrow_mode.set(selection.lower())

            self.redraw_3d()

        self.mode_combo.bind("<<ComboboxSelected>>", on_mode_selected)

        # Recent valid messages

        recent_label = ttk.Label(side_frame, text="Recent valid messages")

        recent_label.pack(anchor=tk.W)

        recent_frame = ttk.Frame(side_frame)

        recent_frame.pack(fill=tk.BOTH, expand=True)

        self.recent_text = tk.Text(recent_frame, height=15, state=tk.DISABLED, wrap=tk.NONE)

        recent_scroll = ttk.Scrollbar(recent_frame, orient=tk.VERTICAL, command=self.recent_text.yview)

        self.recent_text.configure(yscrollcommand=recent_scroll.set)

        self.recent_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        recent_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # Error messages

        error_label = ttk.Label(side_frame, text="Erroneous messages")

        error_label.pack(anchor=tk.W, pady=(8, 0))

        error_frame = ttk.Frame(side_frame)

        error_frame.pack(fill=tk.BOTH, expand=True)

        self.error_text = tk.Text(error_frame, height=15, state=tk.DISABLED, wrap=tk.NONE)

        error_scroll = ttk.Scrollbar(error_frame, orient=tk.VERTICAL, command=self.error_text.yview)

        self.error_text.configure(yscrollcommand=error_scroll.set)

        self.error_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        error_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # Periodic processing of incoming MQTT events

        self.root.after(10, self.process_events)

        # Periodic check for stale drone positions

        self.root.after(1000, self.check_stale_nodes)

    # ------------------------------------------------------------------

    # Logging helpers

    # ------------------------------------------------------------------

    def append_text(self, widget, line):
        # Check if scrollbar is at the bottom before appending
        at_bottom = widget.yview()[1] >= 1.0

        widget.configure(state=tk.NORMAL)
        widget.insert(tk.END, line + "\n")
        if at_bottom:
            widget.see(tk.END)
        widget.configure(state=tk.DISABLED)

    def log_recent(self, line):

        self.append_text(self.recent_text, line)

    def log_error(self, line):

        self.append_text(self.error_text, line)

    # ------------------------------------------------------------------

    # View control helpers

    # ------------------------------------------------------------------

    def _set_full_cube_limits(self):

        x_min, x_max, y_min, y_max, z_min, z_max = self.home_limits

        self.ax.set_xlim(x_min, x_max)

        self.ax.set_ylim(y_min, y_max)

        self.ax.set_zlim(z_min, z_max)

    def reset_view(self):
        """

        Reset the 3D view to the initial home position and limits.

        Also disables tracking.

        """

        self.track_flock.set(False)

        self._set_full_cube_limits()

        elev, azim = self.home_view

        self.ax.view_init(elev=elev, azim=azim)

        self.canvas.draw_idle()

    def set_top_view(self):
        """

        Top view: look down the +Z axis at the X Y plane, full 100 m cube.

        """

        self.track_flock.set(False)

        self._set_full_cube_limits()

        self.ax.view_init(elev=90.0, azim=-90.0)

        self.canvas.draw_idle()

    def set_front_view(self):
        """

        Front view: look along +X towards the Y Z plane, full 100 m cube.

        """

        self.track_flock.set(False)

        self._set_full_cube_limits()

        self.ax.view_init(elev=0.0, azim=-90.0)

        self.canvas.draw_idle()

    def set_side_view(self):
        """

        Side view: look along +Y towards the X Z plane, full 100 m cube.

        """

        self.track_flock.set(False)

        self._set_full_cube_limits()

        self.ax.view_init(elev=0.0, azim=0.0)

        self.canvas.draw_idle()

    def on_track_toggle(self):
        """
        Called when the Track flock tick box is toggled.
        """
        if self.track_flock.get():
            self.redraw_3d()

    def check_stale_nodes(self):
        """
        Remove nodes that have not sent an update for more than 5 seconds.
        """
        now = time.time()
        stale_nodes = []
        for node_id, state in self.node_states.items():
            if now - state.get("last_updated", now) > 5.0:
                stale_nodes.append(node_id)

        if stale_nodes:
            for node_id in stale_nodes:
                del self.node_states[node_id]
                # Also remove from other mappings
                self.node_colours.pop(node_id, None)
                self.node_names.pop(node_id, None)
                if node_id in self.node_annotations:
                    self.node_annotations.pop(node_id).remove()
            self.redraw_3d()

        # Schedule the next check
        self.root.after(1000, self.check_stale_nodes)

    # ------------------------------------------------------------------

    # Scroll wheel zoom

    # ------------------------------------------------------------------

    def on_scroll(self, event):
        """

        Zoom the 3D view with the mouse wheel.



        Scroll up   - zoom in

        Scroll down - zoom out

        """

        if event.inaxes is not self.ax:

            return

        scale_factor = 1.2 if event.button == "up" else 1.0 / 1.2

        x_min, x_max = self.ax.get_xlim()

        y_min, y_max = self.ax.get_ylim()

        z_min, z_max = self.ax.get_zlim()

        def zoom_axis(a_min, a_max, factor):

            centre = 0.5 * (a_min + a_max)

            half_range = 0.5 * (a_max - a_min)

            half_range /= factor

            return centre - half_range, centre + half_range

        x_min, x_max = zoom_axis(x_min, x_max, scale_factor)

        y_min, y_max = zoom_axis(y_min, y_max, scale_factor)

        z_min, z_max = zoom_axis(z_min, z_max, scale_factor)

        self.ax.set_xlim(x_min, x_max)

        self.ax.set_ylim(y_min, y_max)

        self.ax.set_zlim(z_min, z_max)

        self.canvas.draw_idle()

    # ------------------------------------------------------------------

    # 3D drawing

    # ------------------------------------------------------------------

    def get_colour_for_node(self, node_id):
        """

        Assign a stable colour per node_id from the palette.

        """

        if node_id in self.node_colours:

            return self.node_colours[node_id]

        colour = self.colour_palette[self.colour_index % len(self.colour_palette)]

        self.node_colours[node_id] = colour

        self.colour_index += 1

        return colour

    def get_name_for_node(self, node_id):
        """
        Assign a stable name per node_id from the DRONE_NAMES map.
        """
        if node_id in self.node_names:
            return self.node_names[node_id]

        name = DRONE_NAMES.get(node_id, node_id)
        self.node_names[node_id] = name
        return name

    def _update_tracking_limits(self, xs, ys, zs):
        """

        If tracking is enabled, update axis limits so that the centre of mass

        is centred and roughly the closest 90 percent of birds fit in view.

        """

        if not self.track_flock.get():

            return

        n = len(xs)

        if n == 0:

            return

        cx = sum(xs) / n

        cy = sum(ys) / n

        cz = sum(zs) / n

        dists = []

        for x, y, z in zip(xs, ys, zs):

            dx = x - cx

            dy = y - cy

            dz = z - cz

            d2 = dx * dx + dy * dy + dz * dz

            dists.append(d2)

        dists.sort()

        if n == 1:

            r = self.max_m * 0.1

        else:

            idx = int(0.9 * (n - 1))

            r2 = dists[idx]

            r = math.sqrt(r2)

            r *= 1.1

        min_r = self.max_m * 0.05

        if r < min_r:

            r = min_r

        max_m = self.max_m

        cx = max(r, min(max_m - r, cx))

        cy = max(r, min(max_m - r, cy))

        cz = max(r, min(max_m - r, cz))

        self.ax.set_xlim(cx - r, cx + r)

        self.ax.set_ylim(cy - r, cy + r)

        self.ax.set_zlim(cz - r, cz + r)

    def redraw_3d(self):
        """

        Redraw the 3D arrows from self.node_states. If tracking is enabled,

        recalc axis limits around the flock; otherwise preserve the current view.



        Arrow directions are chosen according to self.arrow_mode:

          - "yaw": from yaw_cd in the horizontal plane

          - "velocity": from (vx_mm_s, vy_mm_s, vz_mm_s)

        """

        if self.quiver is not None:

            try:

                self.quiver.remove()

            except Exception:

                pass

            self.quiver = None

        for artist in self.node_annotations.values():
            artist.remove()
        self.node_annotations.clear()

        if not self.node_states:

            self.canvas.draw_idle()

            return

        mode = self.arrow_mode.get()

        xs = []

        ys = []

        zs = []

        us = []

        vs = []

        ws = []

        colours = []

        for node_id, state in self.node_states.items():

            x_mm = state["x_mm"]

            y_mm = state["y_mm"]

            z_mm = state["z_mm"]

            yaw_cd = state["yaw_cd"]

            vx_mm_s = state.get("vx_mm_s", 0)

            vy_mm_s = state.get("vy_mm_s", 0)

            vz_mm_s = state.get("vz_mm_s", 0)

            x_m = x_mm / 1000.0

            y_m = y_mm / 1000.0

            z_m = z_mm / 1000.0

            if mode == "velocity":

                # Use full 3D velocity as direction

                u = vx_mm_s

                v = vy_mm_s

                w = vz_mm_s

            else:

                # Use yaw in the horizontal plane

                yaw_deg = yaw_cd / 100.0

                yaw_rad = math.radians(yaw_deg)

                u = math.cos(yaw_rad)

                v = math.sin(yaw_rad)

                w = 0.0

            xs.append(x_m)

            ys.append(y_m)

            zs.append(z_m)

            us.append(u)

            vs.append(v)

            ws.append(w)

            colours.append(self.get_colour_for_node(node_id))

        self._update_tracking_limits(xs, ys, zs)

        self.quiver = self.ax.quiver(
            xs, ys, zs,
            us, vs, ws,
            length=5.0,
            normalize=True,
            arrow_length_ratio=0.3,
            colors=colours,
        )

        for i, node_id in enumerate(self.node_states.keys()):
            name = self.get_name_for_node(node_id)
            colour = self.get_colour_for_node(node_id)
            x_m, y_m, z_m = xs[i], ys[i], zs[i]
            text = self.ax.text(x_m, y_m, z_m, f"  {name}", color=colour, zorder=10)
            self.node_annotations[node_id] = text

        self.canvas.draw_idle()

    # ------------------------------------------------------------------

    # Event handling

    # ------------------------------------------------------------------

    def enqueue_message(self, event):

        self.event_queue.put(event)

    def process_events(self):

        try:

            while True:

                event = self.event_queue.get_nowait()

                self.handle_event(event)

        except queue.Empty:

            pass

        self.root.after(100, self.process_events)

    def handle_event(self, event):

        etype = event.get("type")

        if etype == "parse_error":

            raw = event.get("raw", "")

            error = event.get("error", "Unknown parse error")

            self.log_error(f"[PARSE ERROR] {error}. Raw: {raw!r}")

            return

        if etype != "message":

            return

        payload = event["payload"]

        errors = event["errors"]

        raw = event["raw"]

        node_repr = str(payload.get("node_id", "unknown"))

        try:

            x_mm = int(payload["x_mm"])

            y_mm = int(payload["y_mm"])

            z_mm = int(payload["z_mm"])

            yaw_cd = int(payload["yaw_cd"])

            vx_mm_s = int(payload["vx_mm_s"])

            vy_mm_s = int(payload["vy_mm_s"])

            vz_mm_s = int(payload["vz_mm_s"])

            ts_s = int(payload.get("ts_s", 0))

            ts_ms = int(payload.get("ts_ms", 0))

        except Exception as e:

            errors.append(f"Missing or invalid fields: {e}")

            self.log_error(f"[INVALID] node={node_repr}: {errors} Raw={raw!r}")

            return

        x_m = x_mm / 1000.0

        y_m = y_mm / 1000.0

        z_m = z_mm / 1000.0

        time_str = f"{ts_s}.{ts_ms:03d}" if ts_s or ts_ms else "n/a"

        base_line = (

            f"[OK] t={time_str} node={node_repr} "

            f"pos=({x_m:.2f}, {y_m:.2f}, {z_m:.2f}) m "

            f"yaw={yaw_cd / 100.0:.1f} deg "

            f"v=({vx_mm_s},{vy_mm_s},{vz_mm_s}) mm/s"

        )

        # Update 3D state including velocity

        self.node_states[node_repr] = {

            "x_mm": x_mm,

            "y_mm": y_mm,

            "z_mm": z_mm,

            "yaw_cd": yaw_cd,

            "vx_mm_s": vx_mm_s,

            "vy_mm_s": vy_mm_s,

            "vz_mm_s": vz_mm_s,

            "last_updated": time.time(),

        }

        # Redraw arrows with the new data

        self.redraw_3d()

        if errors:

            err_line = f"[ERROR] node={node_repr}: " + "; ".join(errors)

            self.log_error(err_line + f" Raw={raw!r}")

        else:

            self.log_recent(base_line)


# =============================================================================

# MQTT client

# =============================================================================


def make_mqtt_client(gui: FlockSim3DGUI):

    client = mqtt.Client()

    def on_connect(client, userdata, flags, rc, properties=None):

        if rc == 0:

            gui.log_recent(f"Connected to MQTT broker {BROKER_HOST}:{BROKER_PORT}")

            client.subscribe(MQTT_TOPIC)

            gui.log_recent(f"Subscribed to topic '{MQTT_TOPIC}'")

        else:

            gui.log_error(f"Failed to connect to MQTT broker: rc={rc}")

    def on_message(client, userdata, msg):

        raw = msg.payload.decode("utf-8", errors="replace").strip()

        try:

            payload = json.loads(raw)

        except json.JSONDecodeError as e:

            gui.enqueue_message(

                {

                    "type": "parse_error",

                    "raw": raw,

                    "error": f"JSON decode error: {e}",

                }

            )

            return

        errors = []

        ok, reason = verify_cmac(payload)

        if not ok:

            errors.append(reason)

        ok, reason = check_position(payload)

        if not ok:

            errors.append(reason)

        ok, reason = check_speed(payload)

        if not ok:

            errors.append(reason)

        gui.enqueue_message(

            {

                "type": "message",

                "payload": payload,

                "raw": raw,

                "errors": errors,

            }

        )

    client.on_connect = on_connect

    client.on_message = on_message

    return client


# =============================================================================

# Main

# =============================================================================


def main():

    root = tk.Tk()

    gui = FlockSim3DGUI(root)

    client = make_mqtt_client(gui)

    def mqtt_thread():

        client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)

        client.loop_forever()

    t = threading.Thread(target=mqtt_thread, daemon=True)

    t.start()

    root.mainloop()


if __name__ == "__main__":

    main()
