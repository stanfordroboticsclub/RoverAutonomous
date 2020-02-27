import msgpack
import UDPComms
import tkinter as tk
import math


sub = UDPComms.Subscriber(8312, timeout = 1)

SQUARE = 500
r = tk.Tk()
canvas = tk.Canvas(r,width=SQUARE,height=SQUARE)
canvas.pack()

PIXELS_PER_METER = 100


def create_point(x,y, angle, color):
    canvas.create_oval(x, y, x, y, width = 1, fill = color)
    if angle != None:
        length = 10
        canvas.create_line(x, y, x + length * math.cos(angle) , y + length * math.sin(angle), arrow=tk.LAST)

def update():
    try:
        data = sub.get()

        canvas.delete('all')
        for point, angle, color in data:
            print(point, angle)
            x,y = point
            create_point(PIXELS_PER_METER * x + SQUARE/2, PIXELS_PER_METER * y + SQUARE/2, angle, color)
        print()
    finally:
        r.after(100,update)

r.after(100,update)
r.mainloop()


