import os

import pyglet

from pyglet import shapes

from pyglet.gl import glClearColor

import threading

import time

import serial
import socket

from pyglet.window import mouse





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
        WINDOW_WIDTH / 2
        + x_mm * (WINDOW_WIDTH - 1) / A4_WIDTH_MM
    )

    screen_y = (
        WINDOW_HEIGHT / 2
        + y_mm * (WINDOW_HEIGHT - 1) / A4_HEIGHT_MM
    )

    return screen_x, screen_y




def send_command(command):

    message = command.strip() + "\n"



    try:

        connection.send(message)



        print("CMD:", command)



    except (serial.SerialException, OSError) as e:

        print("TX error:", e)





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

    resizable=False

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



lines = []



last_x = None

last_y = None



last_x_mm = 0.0

last_y_mm = 0.0



PRESSURE_THRESHOLD = 0.01





# --------------------------------------------------

# Margines

# --------------------------------------------------



margin_x_px = (

    MARGIN_MM / A4_WIDTH_MM

    * WINDOW_WIDTH

)



margin_y_px = (

    MARGIN_MM / A4_HEIGHT_MM

    * WINDOW_HEIGHT

)



left = margin_x_px

right = WINDOW_WIDTH - margin_x_px

bottom = margin_y_px

top = WINDOW_HEIGHT - margin_y_px



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



    for line in lines:

        line.delete()



    lines.clear()



    last_x = None

    last_y = None



    print("Wyczyszczono kartkę")







@window.event



@window.event

def on_mouse_press(x, y, button, modifiers):



    if button != mouse.LEFT:

        return



    # HOME

    if is_inside_rectangle(x, y, home_button):

        if home_state != HOME_HOMING:

            set_home_state(HOME_HOMING)

            send_command("home")

        return



    # WYCZYŚĆ

    if is_inside_rectangle(x, y, clear_button):

        clear_drawing()

        return



    # Obszar rysowania

    inside_drawing_area = (

        left <= x <= right

        and bottom <= y <= top

    )



    if inside_drawing_area:

        connect_tablet()

        return





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

def on_draw():

    window.clear()



    drawing_batch.draw()

    ui_batch.draw()



    position_label.draw()

    pos_label.draw()

    clear_label.draw()



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

pyglet.app.run()
