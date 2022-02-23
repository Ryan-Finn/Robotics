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
    global l1, l2
    len = l1.get() + l2.get()
    posX.set(len)
    posY.set(0)
    update(0)
    servo1.config(from_=len, to=-len)
    servo2.config(from_=len, to=-len)


###########################
#  Publish angles
###########################

def update(foo):
    global l1, l2, publisher, topic
    x = float(posX.get())
    y = float(posY.get())
    a1 = l1.get()
    a2 = l2.get()

    if sqrt(x * x + y * y) > a1 + a2:
        print("Out of range")
    elif sqrt(x * x + y * y) < abs(a2 - a1):
        print("Out of range")
    else:
        sliders = bytes(str(x) + ':' + str(y) + ':' + str(a1) + ':' + str(a2), 'ascii')
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

l1 = Tk.IntVar(value='10')
l2 = Tk.IntVar(value='10')
len = l1.get() + l2.get()
posX = Tk.IntVar()
posY = Tk.IntVar()
servo1 = Tk.Scale(Inverse, from_=len, to=-len, variable=posX, command=update)
servo1.grid(row=2, column=1, sticky=Tk.W)
servo2 = Tk.Scale(Inverse, from_=len, to=-len, variable=posY, command=update)
servo2.grid(row=2, column=2, sticky=Tk.W)

######################
#  0MQ
#######################

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5556")
topic = b"TwoLinkCoords"

clear()
Tk.mainloop()
