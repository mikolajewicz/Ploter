import pyglet

window = pyglet.window.Window(
    width=1,
    height=1,
    visible=True
)

window.set_location(-10000, -10000)

tablets = pyglet.input.get_tablets()

if not tablets:
    print("Nie znaleziono tabletu")
    exit()

tablet = tablets[0]

print("Znaleziono tablet:")
print(tablet)

canvas = tablet.open(window)

@canvas.event
def on_motion(cursor, x, y, pressure, *args):
    print(
        f"X = {x:7.1f}   "
        f"Y = {y:7.1f}   "
        f"pressure = {pressure:.3f}"
    )

@canvas.event
def on_enter(cursor):
    print("Piórko wykryte:", cursor)

@canvas.event
def on_leave(cursor):
    print("Piórko poza zasięgiem:", cursor)

pyglet.app.run()