#!/usr/bin/env python

from math import *
import zmq


##################################
# The IK computation
#################################

def inverse(x, y, a1, a2, dt2, dt3, dt4):
    global publisher, topic
    d = (x * x + y * y - a1 * a1 - a2 * a2) / (2.0 * a1 * a2)
    t2 = atan2(-sqrt(1.0 - d * d), d)
    t1 = atan2(y, x) - atan2(a2 * sin(t2), a1 + a2 * cos(t2))
    dt1 = (180.0 * t1 / pi)
    dt2 = (180.0 * t2 / pi) + (180.0 * dt2 / pi)
    print(x, y, dt1, dt2)
    sliders = bytes(str(dt1) + ':' + str(dt2) + ':' + str(dt3) + ':' + str(dt4), 'ascii')
    publisher.send_multipart([topic, sliders])


##################################
# Setup ZMQ PUBSUB
#################################

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5555")
topic = b"FourLinkTheta"

subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5556")
topic2 = "FourLinkCoords"
subscriber.setsockopt_string(zmq.SUBSCRIBE, topic2)

##################################
# recv is a blocking call
# Wait for new message
# Process when it arrives
# Send result (in the inverse function)
#################################

while True:
    [address, message] = subscriber.recv_multipart()
    # print (message)
    var = message.split(b":")
    x = float(var[0])
    y = float(var[1])
    a1 = float(var[2])
    a2 = float(var[3])
    t2 = float(var[4])
    t3 = float(var[5])
    t4 = float(var[6])
    inverse(x, y, a1, a2, t2, t3, t4)
