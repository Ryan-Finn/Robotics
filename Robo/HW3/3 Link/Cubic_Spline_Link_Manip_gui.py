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
    global px, py, ang3, l1, l2, length, sliders, publisher
    px.set(length)
    py.set(0)
    ang3.set(0)
    a1 = l1.get()
    a2 = l2.get()
    a3 = l3.get()
    sliders = bytes(str(length) + ':0:' + str(a1) + ':' + str(a2 + a3) + ':0:0', 'ascii')
    publisher.send_multipart([topic, sliders])


def rand():
    global sqrtlen
    x = sqrtlen * (2 * np.random.rand(50) - 1)
    y = sqrtlen * (2 * np.random.rand(50) - 1)
    t3 = 360 * np.random.rand() - 180
    px.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(x))))
    py.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(y))))
    ang3.set(t3)


def update():
    global out, px, py, ang2, ang3, l1, l2, l3, sliders, publisher
    x = list(map(float, re.sub(r'(,\s*)|(\s+)', ' ', px.get()).split(" ")))
    y = list(map(float, re.sub(r'(,\s*)|(\s+)', ' ', py.get()).split(" ")))
    t = np.arange(0, 1, 0.002)
    tck, u = interpolate.splprep([x, y], s=0.0)
    out = interpolate.splev(t, tck)
    px.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(x))))
    py.set(re.sub(r'(,\s*)|(\s+)', ' ', re.sub(r'(\[\s*)|(\s*])', '', str(y))))

    a1 = l1.get()
    a2 = l2.get()
    a3 = l3.get()
    t3 = ang3.get()
    theta3 = -pi * float(t3) / 180.0
    a2 = a2 + a3 * cos(theta3)
    endX = a1 + a2
    endY = a3 * sin(theta3)
    a2 = sqrt(a2 * a2 + endY * endY)
    t2 = tan(endY / a2)
    for i in range(len(out[0])):
        x = out[0][i]
        y = out[1][i]
        if sqrt(x * x + y * y) > sqrt(endX * endX + endY * endY):
            print('Out of range')
        elif sqrt(x * x + y * y) < abs(a2 - a1):
            print('Out of range')
        else:
            sliders = bytes(
                str(x) + ':' + str(y) + ':' + str(a1) + ':' + str(a2) + ':' + str(t2) + ':' + str(
                    t3), 'ascii')
            publisher.send_multipart([topic, sliders])


root = Tk.Tk()
root.wm_title("IK Controller")

CMD = Tk.Frame(root)
Inverse = Tk.Frame(root)
CMD.pack(side=Tk.TOP)
Inverse.pack(side=Tk.TOP)

l1 = Tk.IntVar(value='10')
l2 = Tk.IntVar(value='10')
l3 = Tk.IntVar(value='10')
a1 = l1.get()
a2 = l2.get()
a3 = l3.get()
ang2 = Tk.IntVar()
ang3 = Tk.IntVar()
length = a1 + a2 + a3
sqrtlen = length / sqrt(2)

px = Tk.StringVar()
py = Tk.StringVar()
LabelInput1 = Tk.Label(CMD, text="X: ")
LabelInput1.grid(row=1, column=1)
Entry1 = Tk.Entry(CMD, textvariable=px, width=50)
Entry1.grid(row=1, column=2)
LabelInput2 = Tk.Label(CMD, text="Y: ")
LabelInput2.grid(row=2, column=1)
Entry2 = Tk.Entry(CMD, textvariable=py, width=50)
Entry2.grid(row=2, column=2)

LabelInput3 = Tk.Label(CMD, text="t3: ")
LabelInput3.grid(row=3, column=1)
Entry3 = Tk.Entry(CMD, textvariable=ang3, width=10)
Entry3.grid(row=3, column=2)

Run = Tk.Button(CMD, text="Run", command=update)
Run.grid(row=5, column=1, sticky=Tk.E)
Rand = Tk.Button(CMD, text="Random", command=rand)
Rand.grid(row=5, column=2, sticky=Tk.E)
Clear = Tk.Button(CMD, text="Clear", command=clear)
Clear.grid(row=5, column=3, sticky=Tk.E)
Q = Tk.Button(CMD, text="Quit", command=bye)
Q.grid(row=5, column=4, sticky=Tk.E)

out = [[], []]

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5556")
topic = b"ThreeLinkCoords"

clear()
Tk.mainloop()
