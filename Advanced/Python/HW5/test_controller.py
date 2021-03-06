#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:16:20 2022

@author: jonathan
"""

import zmq
import json
import numpy as np

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
# pub_socket.connect("ipc:///tmp/robo_sim/pub.ipc")
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc")
sub_socket.connect("tcp://localhost:5555")

sub_socket.setsockopt(zmq.SUBSCRIBE, b"c")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"actual_state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"model_state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"sensor_state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")

s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1

count = 0
while True:
    topic, message = sub_socket.recv_multipart()
    print(topic, ":", json.loads(message.decode()))

    if topic == b"collision" and json.loads(message.decode())["collision"]:
        wheel_speeds = {"omega1": 0, "omega2": 0}
        pub_socket.send_multipart([b"wheel_speeds", json.dumps(wheel_speeds).encode()])
        break

    # if topic == b"c" and json.loads(message.decode())["c"] < 25:
    #     wheel_speeds = {"omega1": 0, "omega2": 0}
    #     pub_socket.send_multipart([b"wheel_speeds", json.dumps(wheel_speeds).encode()])
    #     continue

    if topic == b"lidar":
        max_dist = 0.5
        angle_interval = np.pi / 19
        message_dict = json.loads(message.decode())
        lidar_dists = message_dict["distances"]

        min_index = np.argmin(lidar_dists)
        min_angle = -np.pi / 2 + min_index * angle_interval
        min_dist = lidar_dists[min_index]

        if min_dist < 0.8 * max_dist:
            k = -10 * np.copysign(np.exp(-np.abs(min_angle)), min_angle)
            if not np.isfinite(k):
                k = 0
        else:
            k = 0

        omega1 = (s + w * k) / (2 * r)
        omega2 = (s - w * k) / (2 * r)

        wheel_speeds = {"omega1": omega1, "omega2": omega2}
        pub_socket.send_multipart([b"wheel_speeds", json.dumps(wheel_speeds).encode()])
        print()
        count += 1
