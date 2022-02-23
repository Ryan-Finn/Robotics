#!/usr/bin/env python

###################

from math import *
from tkinter import *

import zmq


#################
# The Tk function that will draw the updated arm position
#
# Is pretty basic.  It plots the links as two fat line segments.
# If the pen is down (pen = 1) then it will plot the ink dot.
#################

def drawARM(theta1, theta2, theta3, theta4):
    global ws, l1, l2, l3, l4, px, link_width, link1, link2, link3, link4, offset_x, offset_y, base_r, servo1, servo2, servo3, servo4
    s = px.get()

    x1 = int(offset_x / float(s))
    y1 = int(offset_y / float(s))
    x2 = x1 + l1.get() * cos(theta1)
    y2 = (y1 + l1.get() * sin(theta1))

    u1 = x2
    v1 = y2
    u2 = u1 + l2.get() * cos(theta2)
    v2 = (v1 + l2.get() * sin(theta2))

    a1 = u2
    b1 = v2
    a2 = a1 + l3.get() * cos(theta3)
    b2 = (b1 + l3.get() * sin(theta3))

    c1 = a2
    d1 = b2
    c2 = c1 + l4.get() * cos(theta4)
    d2 = (d1 + l4.get() * sin(theta4))

    sx1 = s * x1 - base_r
    sy1 = s * y1 + base_r
    sx2 = s * x1 + base_r
    sy2 = s * y1 - base_r

    sx3 = s * u1 - base_r
    sy3 = s * v1 + base_r
    sx4 = s * u1 + base_r
    sy4 = s * v1 - base_r

    sx5 = s * a1 - base_r
    sy5 = s * b1 + base_r
    sx6 = s * a1 + base_r
    sy6 = s * b1 - base_r

    sx7 = s * c1 - base_r
    sy7 = s * d1 + base_r
    sx8 = s * c1 + base_r
    sy8 = s * d1 - base_r

    link1 = ws.create_line(s * x1, s * y1, s * x2, s * y2, width=link_width, fill="blue")
    link2 = ws.create_line(s * u1, s * v1, s * u2, s * v2, width=link_width, fill="blue")
    link3 = ws.create_line(s * a1, s * b1, s * a2, s * b2, width=link_width, fill="blue")
    link4 = ws.create_line(s * c1, s * d1, s * c2, s * d2, width=link_width, fill="blue")

    servo1 = ws.create_oval(sx1, sy1, sx2, sy2, fill="green")
    servo2 = ws.create_oval(sx3, sy3, sx4, sy4, fill="green")
    servo3 = ws.create_oval(sx5, sy5, sx6, sy6, fill="green")
    servo4 = ws.create_oval(sx7, sy7, sx8, sy8, fill="green")


#################
# The Tk function to update arm position
#
# This function deletes the previous position/image and
# and then calls plotARM to plot the new one
#################

def plotARM(theta1, theta2, theta3, theta4):
    global link1, link2, link3, link4, servo1, servo2, servo3, servo4
    ws.delete(link1)
    ws.delete(servo2)
    ws.delete(link2)
    ws.delete(servo3)
    ws.delete(link3)
    ws.delete(servo4)
    ws.delete(link4)
    drawARM(theta1, theta2, theta3, theta4)


#################
# The main GUI construction function
#
#################

def buildGUI():
    global root, ws, Draw, l1, l2, l3, l4, px, t1, t2, t3, t4
    root = Tk()
    root.wm_title("4 Link Arm Simulation")
    l1 = IntVar(value='10')
    l2 = IntVar(value='10')
    l3 = IntVar(value='10')
    l4 = IntVar(value='10')
    px = IntVar(value='10')
    t1 = IntVar(value='0')
    t2 = IntVar(value='0')
    t3 = IntVar(value='0')
    t4 = IntVar(value='0')
    CMD = Frame(root)
    SIM = Frame(root)
    CMD.pack(side=TOP)
    SIM.pack(side=BOTTOM)
    Reset = Button(CMD, text="Reset", command=reset)
    Reset.grid(row=1, column=9, sticky=E)
    Quit = Button(CMD, text="Quit", command=bye)
    Quit.grid(row=1, column=10, sticky=E)

    LabelInput1 = Label(CMD, text="scale: ")
    LabelInput1.grid(row=2, column=1)
    Entry1 = Entry(CMD, textvariable=px, width=4)
    Entry1.grid(row=2, column=2)

    LabelInput2 = Label(CMD, text="Link1: ")
    LabelInput2.grid(row=2, column=3)
    Entry2 = Entry(CMD, textvariable=l1, width=4)
    Entry2.grid(row=2, column=4)
    LabelInput3 = Label(CMD, text="Link2: ")
    LabelInput3.grid(row=2, column=5)
    Entry3 = Entry(CMD, textvariable=l2, width=4)
    Entry3.grid(row=2, column=6)

    LabelInput4 = Label(CMD, text="Link3: ")
    LabelInput4.grid(row=2, column=7)
    Entry4 = Entry(CMD, textvariable=l3, width=4)
    Entry4.grid(row=2, column=8)
    LabelInput5 = Label(CMD, text="Link4: ")
    LabelInput5.grid(row=2, column=9)
    Entry5 = Entry(CMD, textvariable=l4, width=4)
    Entry5.grid(row=2, column=10)

    ws = Canvas(SIM, width=canvas_width, height=canvas_height, bd=5, relief=RIDGE)
    ws.pack()


#  Yeah, just cause
def bye():
    global subscriber
    print("Exiting")
    subscriber.close()
    quit()


def reset():
    global ws
    ws.delete(ALL)
    drawARM(0, 0, 0, 0)


###############
# 0MQ Comm
##############

def zmqinit():
    global context, subscriber, topic
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5555")
    topic = "FourLinkTheta"
    subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)


def receive():
    try:
        [address, message] = subscriber.recv_multipart(flags=zmq.NOBLOCK)
        angles = message.split(b":")
        t1.set(angles[0])
        t2.set(angles[1])
        t3.set(angles[2])
        t4.set(angles[3])
        theta1 = -pi * float(angles[0]) / 180.0
        theta2 = -pi * float(angles[1]) / 180.0 + theta1
        theta3 = -pi * float(angles[2]) / 180.0 + theta2
        theta4 = -pi * float(angles[3]) / 180.0 + theta3
        plotARM(theta1, theta2, theta3, theta4)
    except zmq.Again as e:
        pass
    root.after(1, receive)


##############
# Ok, this makes these global variables.
##############

base_r = 10
canvas_width = 500
canvas_height = 500
offset_x = canvas_width / 2
offset_y = canvas_height / 2
link_width = 12
pen = 0

############
# Here is main program stuff
############

zmqinit()
buildGUI()
drawARM(0, 0, 0, 0)

################
# Start the sim
##############

receive()
mainloop()
