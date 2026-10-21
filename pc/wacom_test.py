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

WINDOW_HEIGHT = 1000 #700


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
    def __init__(self):
        self.lock = threading.Lock()
        self.serial_port = None
        self.socket = None
        if CONNECTION_MODE == "usb":
            self.serial_port = serial.Serial(
                SERIAL_PORT, BAUD_RATE, timeout=0.01
            )
            print("USB connected:", SERIAL_PORT)
        elif CONNECTION_MODE == "wifi":
            self.socket = socket.create_connection(
                (ESP_HOST, ESP_PORT), timeout=5
            )

            self.socket.setsockopt(
                socket.IPPROTO_TCP,
                socket.TCP_NODELAY,
                1
            )

            self.socket.setblocking(False)
            print("TCP connected:", ESP_HOST, ESP_PORT)
        else:
            raise ValueError("CONNECTION_MODE musi być 'usb' albo 'wifi'")

    def send(self, message):
        payload = message.encode("ascii")
        with self.lock:
            if self.serial_port is not None:
                self.serial_port.write(payload)
            else:
                # socket jest nieblokujący; ramki są niewielkie.
                # W przypadku przepełnienia bufora nie blokujemy Pyglet.
                count = self.socket.send(payload)
                if count != len(payload):
                    raise OSError("TCP: nie wysłano całej komendy")

    def send_bytes(self, payload):
        with self.lock:

            if self.serial_port is not None:
                self.serial_port.write(payload)
                self.serial_port.flush()
                return

            offset = 0

            while offset < len(payload):

                try:
                    sent = self.socket.send(
                        payload[offset:]
                    )

                except BlockingIOError:
                    time.sleep(0.001)
                    continue

                if sent <= 0:
                    raise ConnectionError(
                        "TCP: połączenie zerwane podczas wysyłania"
                    )

                offset += sent

    

    def receive(self):
        if self.serial_port is not None:
            count = self.serial_port.in_waiting
            return self.serial_port.read(count) if count else b""
        try:
            data = self.socket.recv(4096)
            if not data:
                raise ConnectionError("TCP: ESP32 rozłączyło połączenie")
            return data
        except BlockingIOError:
            return b""

    def close(self):
        if self.serial_port is not None:
            self.serial_port.close()
        if self.socket is not None:
            self.socket.close()


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

gcode_document = UnformattedDocument(
    """M5
G0 X0 Y0
"""
)

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

    x=45,

    y=15,

    color=(0, 0, 0, 255)

)



pos_label = pyglet.text.Label(

    "POS: brak",

    x=45,

    y=35,

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



tip_marker = shapes.Circle(

    0,

    0,

    radius=7,

    color=(0, 200, 0)

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

    global gcode_panel_open
    global gcode_editor_focused

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
    # HOME
    # --------------------------------------------------

    if is_inside_rectangle(x, y, home_button):

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

        connect_tablet()

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



    if homing_active:

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

    global tip_position

    global last_pos_debug_time

    global homing_active



    if not line:

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



    if line.startswith("HOME ERROR") or line in (

        "HOME ABORTED",

        "ALL STOPPED",

    ):

        set_home_state(HOME_IDLE)

        print("PC: Homing interrupted")

        return



    parts = line.split()



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



        # Filtr tylko diagnostyczny:

        # odrzucamy NaN / inf / absurdalne wartości.

        if not (-10000.0 < x < 10000.0):

            print("BAD POS X:", repr(line))

            return



        if not (-10000.0 < y < 10000.0):

            print("BAD POS Y:", repr(line))

            return



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

            # print(

            #     f"POS OK: "

            #     f"{x:9.3f} "

            #     f"{y:9.3f}"

            # )



        return



    print("ESP:", line)





def read_esp(dt):

    global rx_buffer



    try:

        data = connection.receive().decode("ascii", errors="ignore")

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



    except (serial.SerialException, OSError) as e:

        print("RX error:", e)





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
    # HOME
    # --------------------------------------------------


    if gcode_panel_open:
        drawing_width = width - GCODE_PANEL_WIDTH
        home_button.x = drawing_width - HOME_BUTTON_WIDTH - HOME_BUTTON_MARGIN
    else:
        drawing_width = width
        home_button.x = (
            width
            - GCODE_TOGGLE_WIDTH
            - HOME_BUTTON_WIDTH
            - HOME_BUTTON_MARGIN
            - 20
        )

    home_button.y = (
        height
        - HOME_BUTTON_HEIGHT
        - HOME_BUTTON_MARGIN
    )

    home_label.x = (
        home_button.x
        + home_button.width / 2
    )

    home_label.y = (
        home_button.y
        + home_button.height / 2
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


        send_gcode_button.x = (
            panel_x
            + GCODE_PANEL_WIDTH / 2
            - SEND_GCODE_WIDTH / 2
        )

        send_gcode_button.y = 20


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
