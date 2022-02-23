import numpy as np
import tkinter as Tk
import zmq
import re
from math import *
from scipy import interpolate


####################################
#  Build Spline
#  Set ds smaller for less drift, larger for
#    smaller data sets.   0.001 is close and
#    larger than 0.005 is off.
####################################


def bye():
    exit()


def clear():
    global l1, l2, px, py, sliders, publisher
    px.set('')
    py.set('')
    sliders = bytes(str(l1.get() + l2.get()) + ':' + str(0) + ':' + str(l1.get()) + ':' + str(l2.get()), 'ascii')
    publisher.send_multipart([topic, sliders])


def rand():
    global sqrtlen
    x = sqrtlen * (2 * np.random.rand(50) - 1)
    y = sqrtlen * (2 * np.random.rand(50) - 1)
    convert(x, y)
    update()


def run():
    global px, py
    x = list(map(float, re.sub(r'(,\s*)|(\s+)', ' ', px.get()).split(" ")))
    y = list(map(float, re.sub(r'(,\s*)|(\s+)', ' ', py.get()).split(" ")))
    convert(x, y)
    update()


def convert(x, y):
    global out, px, py, a1, a2
    t = np.arange(0, 1, 0.002)
    tck, u = interpolate.splprep([x, y], s=0.0)
    out = interpolate.splev(t, tck)
    px.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(x))))
    py.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(y))))

    for i in range(len(out[0])):
        x = out[0][i]
        y = out[1][i]

        if sqrt(x * x + y * y) > a1 + a2:
            out[0][i] = out[0][i - 1]
            out[1][i] = out[1][i - 1]
        elif sqrt(x * x + y * y) < abs(a2 - a1):
            out[0][i] = out[0][i - 1]
            out[1][i] = out[1][i - 1]


def update():
    global out, l1, l2, publisher, topic
    for i in range(len(out[0])):
        sliders = bytes(str(out[0][i]) + ':' + str(out[1][i]) + ':' + str(l1.get()) + ':' + str(l2.get()), 'ascii')
        publisher.send_multipart([topic, sliders])


root = Tk.Tk()
root.wm_title("IK Controller")

CMD = Tk.Frame(root)
Inverse = Tk.Frame(root)
CMD.pack(side=Tk.TOP)
Inverse.pack(side=Tk.TOP)

Run = Tk.Button(CMD, text="Run", command=run)
Run.grid(row=4, column=2, sticky=Tk.E)
Rand = Tk.Button(CMD, text="Random", command=rand)
Rand.grid(row=4, column=3, sticky=Tk.E)
Clear = Tk.Button(CMD, text="Clear", command=clear)
Clear.grid(row=4, column=4, sticky=Tk.E)
Q = Tk.Button(CMD, text="Quit", command=bye)
Q.grid(row=4, column=5, sticky=Tk.E)

px = Tk.StringVar()
py = Tk.StringVar()
LabelInput1 = Tk.Label(CMD, text="X: ")
LabelInput1.grid(row=2, column=1)
Entry1 = Tk.Entry(CMD, textvariable=px, width=50)
Entry1.grid(row=2, column=2)
LabelInput1 = Tk.Label(CMD, text="Y: ")
LabelInput1.grid(row=3, column=1)
Entry1 = Tk.Entry(CMD, textvariable=py, width=50)
Entry1.grid(row=3, column=2)

l1 = Tk.IntVar(value='10')
l2 = Tk.IntVar(value='10')
a1 = l1.get()
a2 = l2.get()
length = a1 + a2
sqrtlen = length / sqrt(2)
out = [[], []]

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5556")
topic = b"TwoLinkCoords"

clear()
Tk.mainloop()
