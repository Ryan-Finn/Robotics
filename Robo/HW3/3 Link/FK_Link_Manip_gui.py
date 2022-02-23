#!/usr/bin/env python

import tkinter as Tk
import zmq


######################
#  Exit and reset
#####################

def bye():
    exit()


def clear():
    ang1.set(0)
    ang2.set(0)
    ang3.set(0)
    update(0)
    servo1.config(from_=180, to=-180)
    servo2.config(from_=180, to=-180)
    servo3.config(from_=180, to=-180)


###########################
#  Publish angles
###########################

def update(foo):
    global publisher, topic
    t1 = float(ang1.get())
    t2 = float(ang2.get())
    t3 = float(ang3.get())
    sliders = bytes(str(t1) + ':' + str(t2) + ':' + str(t3), 'ascii')
    publisher.send_multipart([topic, sliders])


##########################
#  Build GUI
###########################

root = Tk.Tk()
root.wm_title("FK Controller")

CMD = Tk.Frame(root)
Forward = Tk.Frame(root)
CMD.pack(side=Tk.TOP)
Forward.pack(side=Tk.TOP)

Clear = Tk.Button(CMD, text="Clear", command=clear)
Clear.grid(row=1, column=5, sticky=Tk.E)
Q = Tk.Button(CMD, text="Quit", command=bye)
Q.grid(row=1, column=6, sticky=Tk.E)

LabelServo1 = Tk.Label(Forward, text="t1")
LabelServo1.grid(row=1, column=1, sticky=Tk.E)
LabelServo2 = Tk.Label(Forward, text="t2")
LabelServo2.grid(row=1, column=2, sticky=Tk.E)
LabelServo3 = Tk.Label(Forward, text="t3")
LabelServo3.grid(row=1, column=3, sticky=Tk.E)

ang1 = Tk.IntVar()
ang2 = Tk.IntVar()
ang3 = Tk.IntVar()
servo1 = Tk.Scale(Forward, from_=180, to=-180, variable=ang1, command=update)
servo1.grid(row=2, column=1, sticky=Tk.W)
servo2 = Tk.Scale(Forward, from_=180, to=-180, variable=ang2, command=update)
servo2.grid(row=2, column=2, sticky=Tk.W)
servo3 = Tk.Scale(Forward, from_=180, to=-180, variable=ang3, command=update)
servo3.grid(row=2, column=3, sticky=Tk.W)

######################
#  0MQ
#######################

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5555")
topic = b"ThreeLinkTheta"

Tk.mainloop()

clear()
