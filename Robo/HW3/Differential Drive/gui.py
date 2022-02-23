import PySimpleGUI as sg
import zmq
import time

layout = [[sg.Text('Wheel Diameter "D"'), sg.InputText(size=(10, 1)),
           sg.Text('Axle length "2L"'), sg.InputText(size=(10, 1))],
          [sg.Button('Ok'), sg.Button('Cancel')]]

window = sg.Window('Differantial Robot', layout)
event1, values1 = window.read()
window.close()

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:2001")

messages = values1
curMsg = 0
while True:
    time.sleep(1)
    sock.send_pyobj([curMsg, messages[curMsg]])
    if curMsg == 1:
        curMsg = 0
    else:
        curMsg = curMsg + 1
