import tkinter as tk
import PySimpleGUI as sg
import numpy as np
import matplotlib.pyplot as plt
import zmq

sg.Popup("This simulation shows the path that the robot follows in 20 sec. given each wheel velocity on slider.")
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect('tcp://127.0.0.1:2001')
sock.setsockopt_string(zmq.SUBSCRIBE, '')

values1 = [1, 2, 3]
values = np.zeros(2)
for i in range(3):
    message = sock.recv_pyobj()
    print(message)
    values1[i] = message
for i in range(2):
    for j in range(2):
        if values1[j][0] == i:
            values[i] = float(values1[j][1])
        else:
            continue

D = values[0]
L = values[1]

root = tk.Tk()
root.title("SET Theta1 and Theta2")
root.geometry("200x400")


def theta1():
    kk = 0
    plt.clf()
    T1 = slide1.get()
    T2 = slide2.get()
    th = np.arctan(float(T2) / float(T1))
    for i in range(20):
        x1 = ((float(T1) + float(T2)) * (D / 2) * np.cos(th)) * i
        y1 = ((float(T1) + float(T2)) * (D / 2) * np.sin(th)) * i
        plt.xlim([-100 * (D / 2) * np.cos(np.pi / 4), 100 * (D / 2) * np.cos(np.pi / 4)])
        plt.ylim([-100 * (D / 2) * np.sin(np.pi / 4), 100 * (D / 2) * np.sin(np.pi / 4)])
        plt.plot(x1, y1, "o", color="black")
        kk = kk + np.pi / 60
    plt.title("Dİferantial Drive Simulation")
    plt.xlabel("X")
    plt.ylabel("Y")


slide1 = tk.Scale(root, from_=0.001, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel1 = tk.Label(root, text="theta1_dot")
mainlabel1.pack()
slide1.pack()

slide2 = tk.Scale(root, from_=0.001, to=2, resolution=0.001, orient=tk.VERTICAL, command=theta1)
mainlabel2 = tk.Label(root, text="theta2_dot")
mainlabel2.pack()
slide2.pack()

root.mainloop()
