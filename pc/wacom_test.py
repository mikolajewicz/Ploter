import os

import pyglet

from pyglet import shapes

from pyglet.gl import glClearColor

import threading

import time

import serial
import socket

from pyglet.window import mouse

from pyglet.text.document import UnformattedDocument
from pyglet.text.layout import IncrementalTextLayout
from pyglet.text.caret import Caret



print("================================")

print("WACOM TEST - USB / WIFI")

print("RUNNING:", os.path.abspath(__file__))

print("================================")





# --------------------------------------------------

# Kartka A4

# --------------------------------------------------



A4_WIDTH_MM = 297.0

A4_HEIGHT_MM = 210.0

MARGIN_MM = 10.0



WINDOW_WIDTH = 1485 #990

WINDOW_HEIGHT = 900 #700


A4_RATIO = A4_WIDTH_MM / A4_HEIGHT_MM

page_x = 0.0
page_y = 0.0
page_width = WINDOW_WIDTH
page_height = WINDOW_HEIGHT

page_scale = 1.0

# --------------------------------------------------
# Panel G-code
# --------------------------------------------------

GCODE_PANEL_WIDTH = 420
GCODE_PANEL_MARGIN = 15

GCODE_TOGGLE_WIDTH = 30
GCODE_TOGGLE_HEIGHT = 30

gcode_panel_open = True
tablet_control_enabled = True
motors_enabled = True

PLOTTER_DRAW_MIN_DISTANCE_MM = 0.3

def update_page_layout(width, height):
    global page_x, page_y
    global page_width, page_height
    global page_scale
    global left, right, bottom, top

    if gcode_panel_open:
        drawing_width = width - GCODE_PANEL_WIDTH
    else:
        drawing_width = width

    window_ratio = drawing_width / height

    if window_ratio > A4_RATIO:
        page_height = height
        page_width = page_height * A4_RATIO
    else:
        page_width = drawing_width
        page_height = page_width / A4_RATIO

    # Wyśrodkowanie kartki tylko w obszarze rysowania
    page_x = (drawing_width - page_width) / 2.0
    page_y = (height - page_height) / 2.0

    page_scale = page_width / A4_WIDTH_MM

    left = page_x + MARGIN_MM * page_scale
    right = page_x + page_width - MARGIN_MM * page_scale

    bottom = page_y + MARGIN_MM * page_scale
    top = page_y + page_height - MARGIN_MM * page_scale


def screen_to_page_mm(x, y):
    x_mm = (x - page_x) / page_scale
    y_mm = (y - page_y) / page_scale

    return x_mm, y_mm


def page_mm_to_screen(x_mm, y_mm):
    screen_x = page_x + x_mm * page_scale
    screen_y = page_y + y_mm * page_scale

    return screen_x, screen_y
# --------------------------------------------------

# Serial

# --------------------------------------------------



# Wybierz połączenie: "wifi" lub "usb".
CONNECTION_MODE = "wifi"
ESP_HOST = "ploter-esp32.local"
ESP_PORT = 5000
SERIAL_PORT = "COM5"
BAUD_RATE = 115200


class PlotterConnection:
    RECONNECT_INTERVAL = 1.0
    CONNECT_TIMEOUT = 0.5

    def _start_reconnect_thread(self):

        if CONNECTION_MODE != "wifi":
            return

        if (
            self.reconnect_thread is not None
            and self.reconnect_thread.is_alive()
        ):
            return

        self.reconnect_thread = threading.Thread(
            target=self._reconnect_loop,
            daemon=True
        )

        self.reconnect_thread.start()

        print("TCP reconnect thread started")

    def __init__(self):
        self.lock = threading.RLock()

        self.last_rx_time = time.monotonic()
        self.last_ping_time = 0.0

        self.serial_port = None
        self.socket = None

        self.last_ip = None
        self.last_reconnect_log = 0.0
        self.last_rx_time = time.monotonic()

        self.stop_event = threading.Event()
        self.reconnect_thread = None

        if CONNECTION_MODE == "usb":

            self.serial_port = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0.01
            )

            print(
                "USB connected:",
                SERIAL_PORT
            )

        elif CONNECTION_MODE == "wifi":

            # Pierwsza próba połączenia.
            if not self._connect_wifi():
                print(
                    "TCP: ESP niedostępne, "
                    "czekam na połączenie..."
                )

            # Automatyczne ponowne łączenie.

            self._start_reconnect_thread()

        else:
            raise ValueError(
                "CONNECTION_MODE musi być "
                "'usb' albo 'wifi'"
            )

    # --------------------------------------------------
    # WiFi
    # --------------------------------------------------

    def _connect_wifi(self):

        if CONNECTION_MODE != "wifi":
            return False

        with self.lock:
            if self.socket is not None:
                return True

        targets = []

        # Najpierw próbujemy ostatniego znanego IP.
        if self.last_ip is not None:
            targets.append(self.last_ip)

        # Potem hostname .local.
        if ESP_HOST not in targets:
            targets.append(ESP_HOST)

        for host in targets:

            try:
                new_socket = socket.create_connection(
                    (host, ESP_PORT),
                    timeout=1.0
                )

                new_socket.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1
                )

                new_socket.setblocking(False)

                # Zapamiętaj faktyczne IP ESP.
                self.last_ip = (
                    new_socket.getpeername()[0]
                )

            except OSError:
                continue

            with self.lock:

                if self.socket is not None:
                    new_socket.close()
                    return True

                self.socket = new_socket
                self.last_rx_time = time.monotonic()
                self.last_ping_time = 0.0

            print(
                "TCP connected:",
                self.last_ip,
                ESP_PORT
            )

            return True

        return False

    def _disconnect_wifi(
        self,
        broken_socket=None
    ):

        with self.lock:

            if broken_socket is not None:
                if self.socket is not broken_socket:
                    return

            sock = self.socket
            self.socket = None

        if sock is not None:
            try:
                sock.shutdown(
                    socket.SHUT_RDWR
                )
            except OSError:
                pass

            try:
                sock.close()
            except OSError:
                pass

    def _reconnect_loop(self):

        while not self.stop_event.is_set():

            try:

                with self.lock:
                    connected = (
                        self.socket is not None
                    )

                if not connected:

                    if self._connect_wifi():

                        print(
                            "TCP: reconnect OK"
                        )

                    else:

                        now = time.monotonic()

                        if (
                            now
                            - self.last_reconnect_log
                            >= 2.0
                        ):
                            self.last_reconnect_log = now

                            print(
                                "TCP: czekam na ESP32..."
                            )

                else:

                    now = time.monotonic()

                    # Heartbeat co 1 s
                    if (
                        now - self.last_ping_time
                        >= 1.0
                    ):
                        self.last_ping_time = now

                        try:
                            self.send("PING\n")

                        except OSError:
                            self._disconnect_wifi()
                            continue

                    # Brak odpowiedzi przez 5 s
                    if (
                        now - self.last_rx_time
                        >= 5.0
                    ):
                        print(
                            "TCP: brak odpowiedzi ESP32"
                        )

                        self._disconnect_wifi()

            except Exception as e:

                print(
                    "TCP reconnect error:",
                    repr(e)
                )

                try:
                    self._disconnect_wifi()
                except Exception:
                    pass

            self.stop_event.wait(0.25)

    def ensure_reconnect(self):

        if CONNECTION_MODE != "wifi":
            return

        self._start_reconnect_thread()
    # --------------------------------------------------
    # SEND
    # --------------------------------------------------

    def send(self, message):

        payload = message.encode("ascii")

        self.send_bytes(payload)

    def send_bytes(self, payload):

        with self.lock:

            if self.serial_port is not None:

                self.serial_port.write(payload)
                self.serial_port.flush()
                return

            sock = self.socket

            if sock is None:
                raise ConnectionError(
                    "TCP: brak połączenia z ESP32"
                )

            offset = 0

            try:

                while offset < len(payload):

                    try:
                        sent = sock.send(
                            payload[offset:]
                        )

                    except BlockingIOError:
                        time.sleep(0.001)
                        continue

                    if sent <= 0:
                        raise ConnectionError(
                            "TCP: połączenie zerwane "
                            "podczas wysyłania"
                        )

                    offset += sent

            except OSError:

                self._disconnect_wifi(sock)
                raise

    # --------------------------------------------------
    # RECEIVE
    # --------------------------------------------------

    def receive(self):

        if self.serial_port is not None:

            count = self.serial_port.in_waiting

            if count:
                return self.serial_port.read(count)

            return b""

        with self.lock:
            sock = self.socket

        if sock is None:
            return b""

        try:

            data = sock.recv(4096)

            if not data:

                self._disconnect_wifi(sock)

                raise ConnectionError(
                    "TCP: ESP32 rozłączyło połączenie"
                )

            self.last_rx_time = time.monotonic()

            return data

        except BlockingIOError:

            # ESP normalnie regularnie wysyła POS.
            # Jeśli przez 3 s nie przyszło absolutnie nic,
            # traktujemy połączenie jako martwe.
            if (
                time.monotonic()
                - self.last_rx_time
                > 3.0
            ):

                print(
                    "TCP: brak danych przez 3 s, "
                    "resetuję połączenie"
                )

                self._disconnect_wifi(sock)

                raise ConnectionError(
                    "TCP: timeout odbioru"
                )

            return b""

        except OSError:

            self._disconnect_wifi(sock)

            raise

    # --------------------------------------------------
    # CLOSE
    # --------------------------------------------------

    def close(self):

        self.stop_event.set()

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except OSError:
                pass

        self._disconnect_wifi()


connection = PlotterConnection()

SEND_INTERVAL = 0.01

last_send_time = 0.0



tip_position = None

tip_position_lock = threading.Lock()



rx_buffer = ""

last_pos_debug_time = 0.0



DEBUG_POS = True





def plotter_to_screen(x_mm, y_mm):

    screen_x = (
        page_x
        + page_width / 2.0
        + x_mm * page_scale
    )

    screen_y = (
        page_y
        + page_height / 2.0
        + y_mm * page_scale
    )

    return screen_x, screen_y




def send_command(command):

    message = command.strip() + "\n"



    try:

        connection.send(message)



        print("CMD:", command)



    except (serial.SerialException, OSError) as e:

        print("TX error:", e)



def upload_gcode(gcode):

    try:
        # G-code trzymamy jako ASCII.
        payload = (
            gcode.rstrip()
            + "\n"
        ).encode("ascii")

        header = (
            f"GCODE_UPLOAD {len(payload)}\n"
        ).encode("ascii")

        print(
            f"Wysyłanie G-code: "
            f"{len(payload)} bajtów"
        )

        connection.send_bytes(
            header + payload
        )

        print("G-code wysłany do ESP")

    except UnicodeEncodeError:
        print(
            "G-code zawiera znaki spoza ASCII"
        )

    except (
        serial.SerialException,
        OSError,
        ConnectionError
    ) as e:
        print(
            "Błąd wysyłania G-code:",
            e
        )

def console_input():

    print()

    print("================================")

    print("Terminal plotera")

    print("Wpisuj komendy i naciskaj ENTER")

    print("================================")

    print()



    while True:

        try:

            command = input("> ").strip()



            if command:

                send_command(command)



        except EOFError:

            break



        except Exception as e:

            print("Terminal error:", e)





# --------------------------------------------------

# Okno

# --------------------------------------------------



window = pyglet.window.Window(

    width=WINDOW_WIDTH,

    height=WINDOW_HEIGHT,

    caption="Ploter - Wacom / Terminal",

    resizable=True

)



glClearColor(1.0, 1.0, 1.0, 1.0)





# --------------------------------------------------

# Tablet

# --------------------------------------------------



tablet = None

tablet_canvas = None

tablet_available = False





def connect_tablet():

    global tablet, tablet_canvas, tablet_available



    if tablet_canvas is not None:

        return True



    try:

        tablets = pyglet.input.get_tablets()



        if not tablets:

            print("Nie znaleziono tabletu")

            position_label.text = "Tablet: BRAK"

            return False



        tablet = tablets[0]

        tablet_canvas = tablet.open(window)



        setup_tablet_handlers(tablet_canvas)



        tablet_available = True

        position_label.text = "Tablet: OK"



        print("Tablet podłączony:", tablet)

        return True



    except Exception as e:

        print("Tablet error:", e)



        tablet = None

        tablet_canvas = None

        tablet_available = False



        return False



# --------------------------------------------------

# Rysowanie

# --------------------------------------------------



drawing_batch = pyglet.graphics.Batch()

ui_batch = pyglet.graphics.Batch()

# --------------------------------------------------
# Edytor G-code
# --------------------------------------------------

gcode_batch = pyglet.graphics.Batch()

gcode_document = UnformattedDocument("")

gcode_document.set_style(
    0,
    len(gcode_document.text),
    {
        "font_name": "Consolas",
        "font_size": 11,
        "color": (20, 20, 20, 255),
    }
)

gcode_layout = IncrementalTextLayout(
    gcode_document,
    x=0,
    y=0,
    width=100,
    height=100,
    multiline=True,
    wrap_lines=False,
    batch=gcode_batch
)

gcode_caret = Caret(
    gcode_layout,
    window=window
)

gcode_caret.visible = False

gcode_editor_focused = False


gcode_panel = shapes.Rectangle(
    0,
    0,
    GCODE_PANEL_WIDTH,
    WINDOW_HEIGHT,
    color=(235, 235, 235)
)

gcode_editor_bg = shapes.Rectangle(
    0,
    0,
    100,
    100,
    color=(255, 255, 255)
)


gcode_title = pyglet.text.Label(
    "G-CODE",
    font_size=14,
    weight="bold",
    color=(0, 0, 0, 255)
)


SEND_GCODE_WIDTH = 170
SEND_GCODE_HEIGHT = 35

send_gcode_button = shapes.Rectangle(
    0,
    0,
    SEND_GCODE_WIDTH,
    SEND_GCODE_HEIGHT,
    color=(70, 160, 80)
)

send_gcode_label = pyglet.text.Label(
    "WYŚLIJ G-CODE",
    anchor_x="center",
    anchor_y="center",
    color=(255, 255, 255, 255)
)

RUN_GCODE_WIDTH = 170
RUN_GCODE_HEIGHT = 35

run_gcode_button = shapes.Rectangle(
    0,
    0,
    RUN_GCODE_WIDTH,
    RUN_GCODE_HEIGHT,
    color=(60, 110, 200)
)

# --------------------------------------------------
# Przycisk ENABLE / DISABLE
# --------------------------------------------------

ENABLE_BUTTON_WIDTH = 100
ENABLE_BUTTON_HEIGHT = 25
ENABLE_BUTTON_GAP = 10

enable_button = shapes.Rectangle(
    0,
    0,
    ENABLE_BUTTON_WIDTH,
    ENABLE_BUTTON_HEIGHT,
    color=(100, 100, 100)
)

enable_label = pyglet.text.Label(
    "DISABLE",
    anchor_x="center",
    anchor_y="center",
    color=(255, 255, 255, 255),
    font_size=12
)

# --------------------------------------------------
# Przycisk STOP
# --------------------------------------------------

STOP_BUTTON_WIDTH = 100
STOP_BUTTON_HEIGHT = 25
STOP_BUTTON_GAP = 10

stop_button = shapes.Rectangle(
    0,
    0,
    STOP_BUTTON_WIDTH,
    STOP_BUTTON_HEIGHT,
    color=(190, 50, 50)
)

stop_label = pyglet.text.Label(
    "STOP",
    anchor_x="center",
    anchor_y="center",
    color=(255, 255, 255, 255),
    font_size=12
)

run_gcode_label = pyglet.text.Label(
    "RUN G-CODE",
    anchor_x="center",
    anchor_y="center",
    color=(255, 255, 255, 255)
)

gcode_toggle_button = shapes.Rectangle(
    0,
    0,
    GCODE_TOGGLE_WIDTH,
    GCODE_TOGGLE_HEIGHT,
    color=(190, 190, 190)
)

gcode_toggle_label = pyglet.text.Label(
    ">",
    anchor_x="center",
    anchor_y="center",
    font_size=16,
    color=(0, 0, 0, 255)
)

lines = []



last_x = None

last_y = None



last_x_mm = 0.0

last_y_mm = 0.0



PRESSURE_THRESHOLD = 0.01





# --------------------------------------------------

# Margines

# --------------------------------------------------



def update_margin_lines():

    margin_lines[0].x = left
    margin_lines[0].y = bottom
    margin_lines[0].x2 = right
    margin_lines[0].y2 = bottom

    margin_lines[1].x = right
    margin_lines[1].y = bottom
    margin_lines[1].x2 = right
    margin_lines[1].y2 = top

    margin_lines[2].x = right
    margin_lines[2].y = top
    margin_lines[2].x2 = left
    margin_lines[2].y2 = top

    margin_lines[3].x = left
    margin_lines[3].y = top
    margin_lines[3].x2 = left
    margin_lines[3].y2 = bottom



update_page_layout(
    WINDOW_WIDTH,
    WINDOW_HEIGHT
)


margin_lines = [

    shapes.Line(

        left, bottom, right, bottom,

        thickness=1,

        color=(150, 150, 150),

        batch=ui_batch

    ),

    shapes.Line(

        right, bottom, right, top,

        thickness=1,

        color=(150, 150, 150),

        batch=ui_batch

    ),

    shapes.Line(

        right, top, left, top,

        thickness=1,

        color=(150, 150, 150),

        batch=ui_batch

    ),

    shapes.Line(

        left, top, left, bottom,

        thickness=1,

        color=(150, 150, 150),

        batch=ui_batch

    )

]





# --------------------------------------------------

# Tekst

# --------------------------------------------------



initial_text = "Tablet: niepodłączony | kliknij obszar rysowania"



position_label = pyglet.text.Label(
    initial_text,
    x=5,
    y=4,
    anchor_y="bottom",
    color=(0, 0, 0, 255)
)

pos_label = pyglet.text.Label(
    "POS: brak",
    x=5,
    y=22,
    anchor_y="bottom",
    color=(0, 120, 0, 255)
)

# --------------------------------------------------

# Przycisk WYCZYŚĆ

# --------------------------------------------------



BUTTON_X = 10

BUTTON_Y = WINDOW_HEIGHT - 30

BUTTON_WIDTH = 100

BUTTON_HEIGHT = 25



clear_button = shapes.Rectangle(

    BUTTON_X,

    BUTTON_Y,

    BUTTON_WIDTH,

    BUTTON_HEIGHT,

    color=(220, 220, 220),

    batch=ui_batch

)



clear_label = pyglet.text.Label(

    "WYCZYŚĆ",

    x=BUTTON_X + BUTTON_WIDTH / 2,

    y=BUTTON_Y + BUTTON_HEIGHT / 2,

    anchor_x="center",

    anchor_y="center",

    color=(0, 0, 0, 255)

)

PEN_UP_COLOR = (0, 200, 0)
PEN_DOWN_COLOR = (0, 80, 220)

pen_down = False

plotter_draw_x = None
plotter_draw_y = None   

tip_marker = shapes.Circle(
    0,
    0,
    radius=7,
    color=PEN_UP_COLOR
)

# --------------------------------------------------

# Przycisk HOME

# --------------------------------------------------



homing_active = False



HOME_BUTTON_WIDTH = 100

HOME_BUTTON_HEIGHT = 25

HOME_BUTTON_MARGIN = 10



HOME_IDLE = 0

HOME_HOMING = 1

HOME_READY = 2

HOME_MOVED = 3



home_state = HOME_MOVED



HOME_COLORS = {

    HOME_IDLE: (100, 100, 100),

    HOME_HOMING: (200, 40, 40),

    HOME_READY: (40, 170, 70),

    HOME_MOVED: (230, 190, 30),

}



def set_home_state(state):

    global home_state

    global homing_active



    home_state = state

    homing_active = (state == HOME_HOMING)



    home_button.color = HOME_COLORS[state]



home_button = shapes.Rectangle(

    WINDOW_WIDTH - HOME_BUTTON_WIDTH - HOME_BUTTON_MARGIN,

    WINDOW_HEIGHT - HOME_BUTTON_HEIGHT - HOME_BUTTON_MARGIN,

    HOME_BUTTON_WIDTH,

    HOME_BUTTON_HEIGHT,

    color=HOME_COLORS[home_state]

)



home_label = pyglet.text.Label(

    "HOME",

    x=home_button.x + home_button.width / 2,

    y=home_button.y + home_button.height / 2,

    anchor_x="center",

    anchor_y="center",

    font_size=12

)



# --------------------------------------------------

# Czyszczenie

# --------------------------------------------------



def clear_drawing():

    global last_x, last_y



    for segment in lines:
        segment["shape"].delete()

    lines.clear()
    last_x = None
    last_y = None
    print("Wyczyszczono kartkę")


@window.event
def on_mouse_press(x, y, button, modifiers):

    global tablet_control_enabled
    global gcode_panel_open
    global gcode_editor_focused
    global motors_enabled

    if button != mouse.LEFT:
        return


    # --------------------------------------------------
    # Otwieranie / zamykanie panelu G-code
    # --------------------------------------------------

    if is_inside_rectangle(
        x,
        y,
        gcode_toggle_button
    ):
        gcode_panel_open = not gcode_panel_open

        gcode_editor_focused = False
        gcode_caret.visible = False

        if gcode_panel_open:
            gcode_toggle_label.text = ">"
        else:
            gcode_toggle_label.text = "<"

        on_resize(
            window.width,
            window.height
        )

        return


    # --------------------------------------------------
    # Edytor G-code
    # --------------------------------------------------

    if (
        gcode_panel_open
        and is_inside_rectangle(
            x,
            y,
            gcode_editor_bg
        )
    ):
        gcode_editor_focused = True
        gcode_caret.visible = True

        gcode_caret.on_mouse_press(
            x,
            y,
            button,
            modifiers
        )

        return

    gcode_editor_focused = False
    gcode_caret.visible = False


    # --------------------------------------------------
    # WYŚLIJ G-CODE
    # --------------------------------------------------

    if (
        gcode_panel_open
        and is_inside_rectangle(
            x,
            y,
            send_gcode_button
        )
    ):
        gcode = gcode_document.text.strip()

        if not gcode:
            print("G-code jest pusty")
            return

        threading.Thread(
            target=upload_gcode,
            args=(gcode,),
            daemon=True
        ).start()
        return

    # --------------------------------------------------
    # RUN G-CODE
    # --------------------------------------------------

    if (
        gcode_panel_open
        and is_inside_rectangle(
            x,
            y,
            run_gcode_button
        )
    ):
        send_command(
            "GCODE_RUN"
        )

        return

    # --------------------------------------------------
    # STOP
    # --------------------------------------------------

    if is_inside_rectangle(
        x,
        y,
        stop_button
    ):
        tablet_control_enabled = False

        send_command("stop")

        return

    # --------------------------------------------------
    # ENABLE / DISABLE
    # --------------------------------------------------

    if is_inside_rectangle(
        x,
        y,
        enable_button
    ):
        if motors_enabled:
            # DISABLE po stronie ESP robi też STOP.
            tablet_control_enabled = False

            send_command("DISABLE")
        else:
            send_command("ENABLE")

        return

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------

    if is_inside_rectangle(
        x,
        y,
        home_button
    ):
        if not motors_enabled:
            print("PC: HOME blocked - motors disabled")
            return

        if home_state != HOME_HOMING:
            set_home_state(HOME_HOMING)
            send_command("home")

        return

    # --------------------------------------------------
    # WYCZYŚĆ
    # --------------------------------------------------

    if is_inside_rectangle(x, y, clear_button):

        clear_drawing()

        return


    # --------------------------------------------------
    # Obszar rysowania
    # --------------------------------------------------

    inside_drawing_area = (
        left <= x <= right
        and bottom <= y <= top
    )

    if inside_drawing_area:

        tablet_control_enabled = True

        connect_tablet()

        print("PC: Tablet control enabled")

        return

@window.event
def on_text(text):

    if (
        gcode_panel_open
        and gcode_editor_focused
    ):
        return gcode_caret.on_text(text)


@window.event
def on_text_motion(motion):

    if (
        gcode_panel_open
        and gcode_editor_focused
    ):
        return gcode_caret.on_text_motion(motion)


@window.event
def on_text_motion_select(motion):

    if (
        gcode_panel_open
        and gcode_editor_focused
    ):
        return gcode_caret.on_text_motion_select(motion)


@window.event
def on_mouse_drag(
    x,
    y,
    dx,
    dy,
    buttons,
    modifiers
):

    if (
        gcode_panel_open
        and gcode_editor_focused
    ):
        return gcode_caret.on_mouse_drag(
            x,
            y,
            dx,
            dy,
            buttons,
            modifiers
        )


@window.event
def on_mouse_scroll(
    x,
    y,
    scroll_x,
    scroll_y
):

    if (
        gcode_panel_open
        and is_inside_rectangle(
            x,
            y,
            gcode_editor_bg
        )
    ):
        return gcode_caret.on_mouse_scroll(
            x,
            y,
            scroll_x,
            scroll_y
        )


# --------------------------------------------------

# Wysyłanie pozycji tabletu

# --------------------------------------------------



def send_tablet_position(

    x_mm,

    y_mm,

    pressure,

    inside_page

):

    global last_send_time
    global tablet_control_enabled

    if homing_active:
        return

    if not tablet_control_enabled:
        return



    now = time.monotonic()



    if now - last_send_time < SEND_INTERVAL:

        return



    last_send_time = now



    inside = 1 if inside_page else 0



    plotter_x = x_mm - A4_WIDTH_MM / 2.0

    plotter_y = y_mm - A4_HEIGHT_MM / 2.0



    message = (

        f"tablet "

        f"{plotter_x:.2f} "

        f"{plotter_y:.2f} "

        f"{pressure:.3f} "

        f"{inside}\n"

    )



    try:

        connection.send(message)



    except (serial.SerialException, OSError) as e:

        print("TX error:", e)





# --------------------------------------------------

# Obsługa tabletu

# --------------------------------------------------



def setup_tablet_handlers(canvas):



    @canvas.event

    def on_enter(cursor):

        print("Piórko wykryte")



    @canvas.event

    def on_leave(cursor):

        global last_x, last_y



        last_x = None

        last_y = None



        send_tablet_position(

            last_x_mm,

            last_y_mm,

            0.0,

            False

        )



        print("Piórko poza zasięgiem")





    @canvas.event

    def on_motion(

        cursor,

        x,

        y,

        pressure,

        *args

    ):

        global last_x, last_y

        global last_x_mm, last_y_mm



        x_mm, y_mm = screen_to_page_mm(x, y)



        last_x_mm = x_mm

        last_y_mm = y_mm



        inside_page = (

            MARGIN_MM

            <= x_mm

            <= A4_WIDTH_MM - MARGIN_MM

            and

            MARGIN_MM

            <= y_mm

            <= A4_HEIGHT_MM - MARGIN_MM

        )



        send_tablet_position(

            x_mm,

            y_mm,

            pressure,

            inside_page

        )



        status = "OK" if inside_page else "POZA OBSZAREM"



        position_label.text = (

            f"X={x_mm:6.1f} mm   "

            f"Y={y_mm:6.1f} mm   "

            f"P={pressure:.3f}   "

            f"{status}"

        )



        if (
            pressure > PRESSURE_THRESHOLD
            and inside_page
        ):

            if (
                last_x is not None
                and last_y is not None
            ):

                x1_screen, y1_screen = page_mm_to_screen(
                    last_x,
                    last_y
                )

                x2_screen, y2_screen = page_mm_to_screen(
                    x_mm,
                    y_mm
                )

                line = shapes.Line(
                    x1_screen,
                    y1_screen,
                    x2_screen,
                    y2_screen,
                    thickness=2,
                    color=(0, 0, 0),
                    batch=drawing_batch
                )

                lines.append(
                    {
                        "shape": line,
                        "x1_mm": last_x,
                        "y1_mm": last_y,
                        "x2_mm": x_mm,
                        "y2_mm": y_mm
                    }
                )

            # UWAGA:
            # od teraz last_x i last_y przechowują milimetry,
            # a nie piksele
            last_x = x_mm
            last_y = y_mm

        else:

            last_x = None
            last_y = None



def update_drawing_lines():

    for segment in lines:

        x1, y1 = page_mm_to_screen(
            segment["x1_mm"],
            segment["y1_mm"]
        )

        x2, y2 = page_mm_to_screen(
            segment["x2_mm"],
            segment["y2_mm"]
        )

        segment["shape"].x = x1
        segment["shape"].y = y1

        segment["shape"].x2 = x2
        segment["shape"].y2 = y2

# --------------------------------------------------

# Jedyny odbiornik ESP32

# --------------------------------------------------



def handle_esp_line(line):

    global motors_enabled
    global tablet_control_enabled
    global pen_down
    global plotter_draw_x
    global plotter_draw_y

    global tip_position

    global last_pos_debug_time

    global homing_active



    if not line:
        return

    if line == "PEN DOWN":
        pen_down = True
        tip_marker.color = PEN_DOWN_COLOR

        with tip_position_lock:
            if tip_position is not None:
                plotter_draw_x = tip_position[0]
                plotter_draw_y = tip_position[1]
            else:
                plotter_draw_x = None
                plotter_draw_y = None

        return


    if line == "PEN UP":
        pen_down = False
        tip_marker.color = PEN_UP_COLOR

        plotter_draw_x = None
        plotter_draw_y = None

        return

    if line in ("HOME sequence started", "HOME ACCEPTED"):

        set_home_state(HOME_HOMING)

        print("PC: Homing active")

        return



    if line == "HOME sequence finished":

        set_home_state(HOME_READY)

        print("PC: Homing finished")

        return



    if line == "ERR HOME BUSY":

        # Nie wyłączamy blokady, bo HOME nadal trwa

        set_home_state(HOME_HOMING)

        return



    if (
        line.startswith("HOME ERROR")
        or line == "HOME ABORTED"
    ):
        set_home_state(HOME_IDLE)

        print("PC: Homing interrupted")

        return


    if line == "ALL STOPPED":

        if homing_active:
            set_home_state(HOME_IDLE)

            print("PC: Homing interrupted")
        else:
            print("PC: Motion stopped")

        return


    parts = line.split()

    if line == "MOTORS DISABLED":
        motors_enabled = False
        tablet_control_enabled = False

        enable_label.text = "ENABLE"

        set_home_state(HOME_IDLE)

        print("PC: Motors disabled")

        return


    if line == "MOTORS ENABLED":
        motors_enabled = True

        enable_label.text = "DISABLE"

        print("PC: Motors enabled")

        return

    if len(parts) == 3 and parts[0] == "POS":

        try:
            x = float(parts[1])
            y = float(parts[2])

            if home_state == HOME_READY:
                if abs(x) > 0.5 or abs(y) > 0.5:
                    set_home_state(HOME_MOVED)

        except ValueError:
            print("BAD POS:", repr(line))
            return

        if not (-10000.0 < x < 10000.0):
            print("BAD POS X:", repr(line))
            return

        if not (-10000.0 < y < 10000.0):
            print("BAD POS Y:", repr(line))
            return

        if pen_down:

            if (
                plotter_draw_x is not None
                and plotter_draw_y is not None
            ):
                dx = x - plotter_draw_x
                dy = y - plotter_draw_y

                distance_sq = dx * dx + dy * dy

                if distance_sq >= (
                    PLOTTER_DRAW_MIN_DISTANCE_MM
                    * PLOTTER_DRAW_MIN_DISTANCE_MM
                ):
                    x1_mm = (
                        plotter_draw_x
                        + A4_WIDTH_MM / 2.0
                    )

                    y1_mm = (
                        plotter_draw_y
                        + A4_HEIGHT_MM / 2.0
                    )

                    x2_mm = (
                        x
                        + A4_WIDTH_MM / 2.0
                    )

                    y2_mm = (
                        y
                        + A4_HEIGHT_MM / 2.0
                    )

                    x1_screen, y1_screen = page_mm_to_screen(
                        x1_mm,
                        y1_mm
                    )

                    x2_screen, y2_screen = page_mm_to_screen(
                        x2_mm,
                        y2_mm
                    )

                    shape = shapes.Line(
                        x1_screen,
                        y1_screen,
                        x2_screen,
                        y2_screen,
                        thickness=2,
                        color=(0, 80, 220),
                        batch=drawing_batch
                    )

                    lines.append(
                        {
                            "shape": shape,
                            "x1_mm": x1_mm,
                            "y1_mm": y1_mm,
                            "x2_mm": x2_mm,
                            "y2_mm": y2_mm
                        }
                    )

                    plotter_draw_x = x
                    plotter_draw_y = y

        with tip_position_lock:
            tip_position = (x, y)

        pos_label.text = (
            f"POS: X={x:8.3f}  Y={y:8.3f}"
        )

        now = time.monotonic()

        if (
            DEBUG_POS
            and now - last_pos_debug_time >= 0.5
        ):
            last_pos_debug_time = now

        return

    if line == "PONG":
        return

    print("ESP:", line)





def read_esp(dt):

    global rx_buffer

    connection.ensure_reconnect()

    try:

        data = connection.receive().decode(
            "ascii",
            errors="ignore"
        )

        if not data:
            return

        rx_buffer += data

        while "\n" in rx_buffer:

            line, rx_buffer = rx_buffer.split(
                "\n",
                1
            )

            handle_esp_line(
                line.strip()
            )

    except (
        serial.SerialException,
        OSError
    ) as e:

        print(
            "RX error:",
            e
        )


# --------------------------------------------------

# Rysowanie okna

# --------------------------------------------------



def is_inside_rectangle(x, y, rect):

    return (

        rect.x <= x <= rect.x + rect.width

        and rect.y <= y <= rect.y + rect.height

    )



@window.event
def on_resize(width, height):

    update_page_layout(width, height)

    update_margin_lines()


    # --------------------------------------------------
    # WYCZYŚĆ
    # --------------------------------------------------

    clear_button.y = height - 30

    clear_label.x = (
        clear_button.x
        + clear_button.width / 2
    )

    clear_label.y = (
        clear_button.y
        + clear_button.height / 2
    )


    # --------------------------------------------------
    # HOME + STOP + ENABLE/DISABLE
    # --------------------------------------------------

    if gcode_panel_open:
        drawing_width = width - GCODE_PANEL_WIDTH

        enable_button.x = (
            drawing_width
            - ENABLE_BUTTON_WIDTH
            - HOME_BUTTON_MARGIN
        )
    else:
        drawing_width = width

        enable_button.x = (
            width
            - GCODE_TOGGLE_WIDTH
            - ENABLE_BUTTON_WIDTH
            - HOME_BUTTON_MARGIN
            - 20
        )


    enable_button.y = (
        height
        - ENABLE_BUTTON_HEIGHT
        - HOME_BUTTON_MARGIN
    )


    # STOP po lewej od ENABLE/DISABLE

    stop_button.x = (
        enable_button.x
        - STOP_BUTTON_WIDTH
        - ENABLE_BUTTON_GAP
    )

    stop_button.y = enable_button.y


    # HOME po lewej od STOP

    home_button.x = (
        stop_button.x
        - HOME_BUTTON_WIDTH
        - STOP_BUTTON_GAP
    )

    home_button.y = enable_button.y


    # HOME label

    home_label.x = (
        home_button.x
        + home_button.width / 2
    )

    home_label.y = (
        home_button.y
        + home_button.height / 2
    )


    # STOP label

    stop_label.x = (
        stop_button.x
        + stop_button.width / 2
    )

    stop_label.y = (
        stop_button.y
        + stop_button.height / 2
    )


    # ENABLE / DISABLE label

    enable_label.x = (
        enable_button.x
        + enable_button.width / 2
    )

    enable_label.y = (
        enable_button.y
        + enable_button.height / 2
    )
    
    # --------------------------------------------------
    # Panel G-code
    # --------------------------------------------------

    if gcode_panel_open:

        panel_x = width - GCODE_PANEL_WIDTH

        gcode_panel.x = panel_x
        gcode_panel.y = 0

        gcode_panel.width = GCODE_PANEL_WIDTH
        gcode_panel.height = height


        editor_x = (
            panel_x
            + GCODE_PANEL_MARGIN
        )

        editor_y = 75

        editor_width = (
            GCODE_PANEL_WIDTH
            - 2 * GCODE_PANEL_MARGIN    
        )

        editor_height = height - 130


        gcode_editor_bg.x = editor_x
        gcode_editor_bg.y = editor_y
        gcode_editor_bg.width = editor_width
        gcode_editor_bg.height = editor_height


        gcode_layout.x = editor_x + 8
        gcode_layout.y = editor_y + 8

        gcode_layout.width = int(
            editor_width - 16
        )

        gcode_layout.height = int(
            editor_height - 16
        )


        gcode_title.x = panel_x + 55
        gcode_title.y = height - 30


        gcode_toggle_button.x = (
            panel_x + 10
        )

        gcode_toggle_button.y = (
            height
            - GCODE_TOGGLE_HEIGHT
            - 10
        )


        BUTTON_GAP = 10

        total_width = (
            SEND_GCODE_WIDTH
            + BUTTON_GAP
            + RUN_GCODE_WIDTH
        )

        buttons_x = (
            panel_x
            + GCODE_PANEL_WIDTH / 2
            - total_width / 2
        )

        send_gcode_button.x = buttons_x
        send_gcode_button.y = 20

        run_gcode_button.x = (
            send_gcode_button.x
            + SEND_GCODE_WIDTH
            + BUTTON_GAP
        )

        run_gcode_button.y = 20

    else:

        gcode_toggle_button.x = (
            width
            - GCODE_TOGGLE_WIDTH
            - 10
        )

        gcode_toggle_button.y = (
            height
            - GCODE_TOGGLE_HEIGHT
            - 10
        )

    position_label.x = 5
    position_label.y = 4

    pos_label.x = 5
    pos_label.y = 22

    # --------------------------------------------------
    # Napisy przycisków
    # --------------------------------------------------

    gcode_toggle_label.x = (
        gcode_toggle_button.x
        + gcode_toggle_button.width / 2
    )

    gcode_toggle_label.y = (
        gcode_toggle_button.y
        + gcode_toggle_button.height / 2
    )


    send_gcode_label.x = (
        send_gcode_button.x
        + send_gcode_button.width / 2
    )

    send_gcode_label.y = (
        send_gcode_button.y
        + send_gcode_button.height / 2
    )

    run_gcode_label.x = (
        run_gcode_button.x
        + run_gcode_button.width / 2
    )

    run_gcode_label.y = (
        run_gcode_button.y
        + run_gcode_button.height / 2
    )

    update_drawing_lines()



@window.event
def on_draw():

    window.clear()


    # --------------------------------------------------
    # Kartka
    # --------------------------------------------------

    drawing_batch.draw()

    ui_batch.draw()


    # --------------------------------------------------
    # Panel G-code
    # --------------------------------------------------

    if gcode_panel_open:

        gcode_panel.draw()

        gcode_editor_bg.draw()

        gcode_batch.draw()

        gcode_title.draw()

        send_gcode_button.draw()

        send_gcode_label.draw()

        run_gcode_button.draw()
        run_gcode_label.draw()


    # Przycisk otwierania panelu jest zawsze widoczny
    gcode_toggle_button.draw()

    gcode_toggle_label.draw()


    # --------------------------------------------------
    # Informacje
    # --------------------------------------------------

    position_label.draw()

    pos_label.draw()

    clear_label.draw()


    # --------------------------------------------------
    # Pozycja plotera
    # --------------------------------------------------

    with tip_position_lock:

        position = tip_position


    if position is not None:

        x_mm, y_mm = position

        screen_x, screen_y = plotter_to_screen(
            x_mm,
            y_mm
        )

        tip_marker.x = screen_x
        tip_marker.y = screen_y

        tip_marker.draw()


    home_button.draw()

    home_label.draw()

    stop_button.draw()
    stop_label.draw()

    home_button.draw()
    home_label.draw()

    stop_button.draw()
    stop_label.draw()

    enable_button.draw()
    enable_label.draw()




# --------------------------------------------------

# Zamknięcie

# --------------------------------------------------



@window.event

def on_close():

    try:

        connection.close()

    except Exception:

        pass



    pyglet.app.exit()





# --------------------------------------------------

# Start

# --------------------------------------------------



threading.Thread(

    target=console_input,

    daemon=True

).start()



pyglet.clock.schedule_interval(

    read_esp,

    0.01

)

on_resize(
    window.width,
    window.height
)

pyglet.app.run()
