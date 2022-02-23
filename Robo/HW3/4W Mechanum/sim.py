import tkinter as tk
import PySimpleGUI as sg
import numpy as np
import matplotlib.pyplot as plt
import zmq

sg.Popup("This simulation shows the path that the robot follows in 30 sec. given each wheel velocity and direction on "
         "slider.")
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect('tcp://127.0.0.1:2002')
sock.setsockopt_string(zmq.SUBSCRIBE, '')

values1 = [1, 2, 3, 4]
values = np.zeros(3)
for i in range(3):
    message = sock.recv_pyobj()
    print(message)
    values1[i] = message
for i in range(3):
    for j in range(3):
        if values1[j][0] == i:
            values[i] = float(values1[j][1])
        else:
            continue

D = values[0]
L1 = values[1]
L2 = values[2]

root = tk.Tk()
root.title("SET Theta1 and Theta2")
root.geometry("200x400")


def theta1():
    plt.clf()
    T1 = slide1.get()
    T2 = slide2.get()
    T3 = slide3.get()
    T4 = slide4.get()
    T5 = slide5.get()
    A = float(T1) + float(T2) + float(T3) + float(T4)
    B = -float(T1) + float(T2) + float(T3) - float(T4)
    C = -float(T1) + float(T2) - float(T3) + float(T4)
    for i in range(30):
        x = (D / 8) * (A * np.cos(float(T5) * np.pi / 180) - B * np.sin(float(T5) * np.pi / 180)) * i
        y = (D / 8) * (A * np.sin(float(T5) * np.pi / 180) + B * np.cos(float(T5) * np.pi / 180)) * i
        plt.xlim([-100 * (D / 2) * np.cos(np.pi / 4), 100 * (D / 2) * np.cos(np.pi / 4)])
        plt.ylim([-100 * (D / 2) * np.sin(np.pi / 4), 100 * (D / 2) * np.sin(np.pi / 4)])
        plt.plot(x, y, "o", color="black")

    plt.title("Mechanum Drive Simulation")
    plt.xlabel("X")
    plt.ylabel("Y")


slide1 = tk.Scale(root, from_=-2, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel1 = tk.Label(root, text="velocity of FL")
mainlabel1.pack()
slide1.pack()

slide2 = tk.Scale(root, from_=-2, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel2 = tk.Label(root, text="velocity of FR")
mainlabel2.pack()
slide2.pack()

slide3 = tk.Scale(root, from_=-2, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel3 = tk.Label(root, text="velocity of BL")
mainlabel3.pack()
slide3.pack()

slide4 = tk.Scale(root, from_=-2, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel4 = tk.Label(root, text="velocity of BR")
mainlabel4.pack()
slide4.pack()

slide5 = tk.Scale(root, from_=0, to=45, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel5 = tk.Label(root, text="theta")
mainlabel5.pack()
slide5.pack()

root.mainloop()
