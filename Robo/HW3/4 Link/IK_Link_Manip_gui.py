#!/usr/bin/env python

import tkinter as Tk
from math import *
import zmq


######################
#  Exit and reset
#####################

def bye():
    exit()


def clear():
    global posX, posY, ang3, ang4, length
    posX.set(length)
    posY.set(0)
    ang3.set(0)
    ang4.set(0)
    update(0)


###########################
#  Publish angles
###########################

def update(foo):
    global posX, posY, ang3, ang4, l1, l2, l3, l4, length
    x = float(posX.get())
    y = float(posY.get())
    t3 = float(ang3.get())
    t4 = float(ang4.get())
    a1 = l1.get()
    a2 = l2.get()
    a3 = l3.get()
    a4 = l4.get()

    theta3 = -pi * float(t3) / 180.0
    theta4 = -pi * float(t4) / 180.0 + theta3
    a2 = a2 + a3 * cos(theta3) + a4 * cos(theta4)
    endX = a1 + a2
    endY = a3 * sin(theta3) + a4 * sin(theta4)
    a2 = sqrt(a2 * a2 + endY * endY)
    t2 = tan(endY / a2)
    if sqrt(x * x + y * y) > sqrt(endX * endX + endY * endY):
        print("Out of range")
    elif sqrt(x * x + y * y) < abs(a2 - a1):
        print("Out of range")
    else:
        sliders = bytes(
            str(x) + ':' + str(y) + ':' + str(a1) + ':' + str(a2) + ':' + str(t2) + ':' + str(t3) + ':' + str(t4),
            'ascii')
        publisher.send_multipart([topic, sliders])


##########################
#  Build GUI
###########################

root = Tk.Tk()
root.wm_title("IK Controller")

CMD = Tk.Frame(root)
Inverse = Tk.Frame(root)
CMD.pack(side=Tk.TOP)
Inverse.pack(side=Tk.TOP)

Clear = Tk.Button(CMD, text="Clear", command=clear)
Clear.grid(row=1, column=5, sticky=Tk.E)
Q = Tk.Button(CMD, text="Quit", command=bye)
Q.grid(row=1, column=6, sticky=Tk.E)

LabelServo1 = Tk.Label(Inverse, text="X")
LabelServo1.grid(row=1, column=1, sticky=Tk.E)
LabelServo2 = Tk.Label(Inverse, text="Y")
LabelServo2.grid(row=1, column=2, sticky=Tk.E)
LabelServo3 = Tk.Label(Inverse, text="t3")
LabelServo3.grid(row=1, column=3, sticky=Tk.E)
LabelServo4 = Tk.Label(Inverse, text="t4")
LabelServo4.grid(row=1, column=4, sticky=Tk.E)

l1 = Tk.IntVar(value='10')
l2 = Tk.IntVar(value='10')
l3 = Tk.IntVar(value='10')
l4 = Tk.IntVar(value='10')
length = l1.get() + l2.get() + l3.get() + l4.get()
posX = Tk.IntVar()
posY = Tk.IntVar()
ang3 = Tk.IntVar()
ang4 = Tk.IntVar()
servo1 = Tk.Scale(Inverse, from_=length, to=-length, variable=posX, command=update)
servo1.grid(row=2, column=1, sticky=Tk.W)
servo2 = Tk.Scale(Inverse, from_=length, to=-length, variable=posY, command=update)
servo2.grid(row=2, column=2, sticky=Tk.W)
servo3 = Tk.Scale(Inverse, from_=180, to=-180, variable=ang3, command=update)
servo3.grid(row=2, column=3, sticky=Tk.W)
servo4 = Tk.Scale(Inverse, from_=180, to=-180, variable=ang4, command=update)
servo4.grid(row=2, column=4, sticky=Tk.W)

######################
#  0MQ
#######################

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5556")
topic = b"FourLinkCoords"

clear()
Tk.mainloop()
