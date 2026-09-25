import pyglet
from pyglet import shapes
from pyglet.gl import glClearColor
import threading
import time
import serial


# --------------------------------------------------
# Kartka A4
# --------------------------------------------------

A4_WIDTH_MM = 297.0
A4_HEIGHT_MM = 210.0

MARGIN_MM = 10.0

WINDOW_WIDTH = 990
WINDOW_HEIGHT = 700


# --------------------------------------------------
# Serial
# --------------------------------------------------

ser = serial.Serial(
    "COM5",
    115200,
    timeout=0.01
)

serial_lock = threading.Lock()

SEND_INTERVAL = 0.01
last_send_time = 0.0


def send_command(command):
    message = command.strip() + "\n"

    with serial_lock:
        ser.write(message.encode("ascii"))

    print("CMD:", command)


def read_esp(dt):
    while ser.in_waiting > 0:

        line = ser.readline().decode(
            "utf-8",
            errors="ignore"
        ).strip()

        if line:
            print("ESP:", line)


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


threading.Thread(
    target=console_input,
    daemon=True
).start()


# --------------------------------------------------
# Okno
# --------------------------------------------------

window = pyglet.window.Window(
    width=WINDOW_WIDTH,
    height=WINDOW_HEIGHT,
    caption="Ploter - Wacom / Terminal",
    resizable=False
)

glClearColor(1.0, 1.0, 1.0, 1.0)


# --------------------------------------------------
# Tablet - OPCJONALNY
# --------------------------------------------------

tablets = pyglet.input.get_tablets()

tablet = None
canvas = None
tablet_available = False


if tablets:
    tablet = tablets[0]

    print("Znaleziono tablet:")
    print(tablet)

    canvas = tablet.open(window)

    tablet_available = True

else:
    print()
    print("Nie znaleziono tabletu.")
    print("Terminal nadal działa.")
    print()


# --------------------------------------------------
# Rysowanie
# --------------------------------------------------

drawing_batch = pyglet.graphics.Batch()
ui_batch = pyglet.graphics.Batch()

lines = []

last_x = None
last_y = None

last_x_mm = 0.0
last_y_mm = 0.0

PRESSURE_THRESHOLD = 0.01


# --------------------------------------------------
# Przeliczenie marginesu mm -> piksele
# --------------------------------------------------

margin_x_px = (
    MARGIN_MM / A4_WIDTH_MM
    * WINDOW_WIDTH
)

margin_y_px = (
    MARGIN_MM / A4_HEIGHT_MM
    * WINDOW_HEIGHT
)


# --------------------------------------------------
# Ramka marginesu
# --------------------------------------------------

left = margin_x_px
right = WINDOW_WIDTH - margin_x_px

bottom = margin_y_px
top = WINDOW_HEIGHT - margin_y_px


margin_lines = [
    shapes.Line(
        left,
        bottom,
        right,
        bottom,
        thickness=1,
        color=(150, 150, 150),
        batch=ui_batch
    ),

    shapes.Line(
        right,
        bottom,
        right,
        top,
        thickness=1,
        color=(150, 150, 150),
        batch=ui_batch
    ),

    shapes.Line(
        right,
        top,
        left,
        top,
        thickness=1,
        color=(150, 150, 150),
        batch=ui_batch
    ),

    shapes.Line(
        left,
        top,
        left,
        bottom,
        thickness=1,
        color=(150, 150, 150),
        batch=ui_batch
    )
]


# --------------------------------------------------
# Tekst
# --------------------------------------------------

if tablet_available:
    initial_text = (
        "Tablet: OK | "
        "X=0.0 mm Y=0.0 mm P=0.000"
    )
else:
    initial_text = (
        "Tablet: BRAK | terminal aktywny"
    )


position_label = pyglet.text.Label(
    initial_text,
    x=45,
    y=15,
    color=(0, 0, 0, 255)
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


# --------------------------------------------------
# Czyszczenie
# --------------------------------------------------

def clear_drawing():
    global last_x, last_y

    for line in lines:
        line.delete()

    lines.clear()

    last_x = None
    last_y = None

    print("Wyczyszczono kartkę")


# --------------------------------------------------
# Kliknięcie
# --------------------------------------------------

@window.event
def on_mouse_press(x, y, button, modifiers):

    inside_button = (
        BUTTON_X <= x <= BUTTON_X + BUTTON_WIDTH
        and
        BUTTON_Y <= y <= BUTTON_Y + BUTTON_HEIGHT
    )

    if inside_button:
        clear_drawing()


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

    now = time.monotonic()

    if now - last_send_time < SEND_INTERVAL:
        return

    last_send_time = now

    inside = 1 if inside_page else 0

    # współrzędne kartki:
    # 0 ... 297  ->  -148.5 ... +148.5
    # 0 ... 210  ->  -105.0 ... +105.0

    plotter_x = x_mm - A4_WIDTH_MM / 2.0
    plotter_y = y_mm - A4_HEIGHT_MM / 2.0

    message = (
        f"tablet "
        f"{plotter_x:.2f} "
        f"{plotter_y:.2f} "
        f"{pressure:.3f} "
        f"{inside}\n"
    )

    with serial_lock:
        ser.write(
            message.encode("ascii")
        )

# --------------------------------------------------
# Obsługa tabletu
# --------------------------------------------------

if canvas is not None:

    @canvas.event
    def on_enter(cursor):
        print("Piórko wykryte")


    @canvas.event
    def on_leave(cursor):
        global last_x
        global last_y

        last_x = None
        last_y = None

        # Powiedz ESP32, że pióro
        # nie jest aktywne.
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
        global last_x
        global last_y

        global last_x_mm
        global last_y_mm

        # ------------------------------------------
        # piksele -> mm
        # ------------------------------------------

        x_mm = (
            x /
            (WINDOW_WIDTH - 1) *
            A4_WIDTH_MM
        )

        y_mm = (
            y /
            (WINDOW_HEIGHT - 1) *
            A4_HEIGHT_MM
        )

        last_x_mm = x_mm
        last_y_mm = y_mm

        # ------------------------------------------
        # Obszar roboczy
        # ------------------------------------------

        inside_page = (
            MARGIN_MM
            <= x_mm
            <= A4_WIDTH_MM - MARGIN_MM

            and

            MARGIN_MM
            <= y_mm
            <= A4_HEIGHT_MM - MARGIN_MM
        )

        # ------------------------------------------
        # ESP32
        # ------------------------------------------

        send_tablet_position(
            x_mm,
            y_mm,
            pressure,
            inside_page
        )

        # ------------------------------------------
        # Tekst
        # ------------------------------------------

        if inside_page:
            status = "OK"
        else:
            status = "POZA OBSZAREM"

        position_label.text = (
            f"X={x_mm:6.1f} mm   "
            f"Y={y_mm:6.1f} mm   "
            f"P={pressure:.3f}   "
            f"{status}"
        )

        # ------------------------------------------
        # Rysowanie
        # ------------------------------------------

        if (
            pressure > PRESSURE_THRESHOLD
            and inside_page
        ):

            if (
                last_x is not None
                and last_y is not None
            ):

                line = shapes.Line(
                    last_x,
                    last_y,
                    x,
                    y,
                    thickness=2,
                    color=(0, 0, 0),
                    batch=drawing_batch
                )

                lines.append(line)

            last_x = x
            last_y = y

        else:

            last_x = None
            last_y = None


# --------------------------------------------------
# Rysowanie okna
# --------------------------------------------------

@window.event
def on_draw():

    window.clear()

    drawing_batch.draw()
    ui_batch.draw()

    position_label.draw()
    clear_label.draw()


# --------------------------------------------------
# ESP32 RX
# --------------------------------------------------

pyglet.clock.schedule_interval(
    read_esp,
    0.01
)


# --------------------------------------------------
# Start
# --------------------------------------------------

pyglet.app.run()