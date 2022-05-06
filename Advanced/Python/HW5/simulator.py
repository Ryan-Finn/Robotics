#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:00:06 2022

@author: jonathan
"""

import zmq
import differential_drive as dd
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import json
from scipy.integrate import solve_ivp
import shapely.geometry as geom
import descartes as dc
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("map", help="The path to the map file to be loaded")
args = parser.parse_args()

map_name = args.map

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
# pub_socket.connect("ipc:///tmp/robo_sim/pub.ipc")
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc")
sub_socket.connect("tcp://localhost:5555")

sub_socket.setsockopt(zmq.SUBSCRIBE, b"wheel_speeds")

found_landmarks = dict()

robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
robot2 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
timestep = 0.02

lidar_num_rays = 20
lidar_max_dist = 0.5
lidar_min_angle = 0
lidar_max_angle = np.pi
lidar_range_sigma = 0.05
lidar_bearing_sigma = 0.01

wheel_noise_a1 = 0.01
wheel_noise_a2 = 0
wheel_noise_a3 = 0
wheel_noise_a4 = 0.01


def load_map(filename):
    obstacles = []
    with open(filename) as file:
        obj = json.load(file)
    for obs_pts in obj["obstacles"]:
        poly = geom.Polygon(obs_pts)
        obstacles.append(poly)
    start = obj["start"]
    goal = obj["goal"]

    return obstacles, start, goal


obstacles, start, goal = load_map(map_name)
landmarks = []
landmarks.append(goal)
for obs in obstacles:
    landmarks += obs.boundary.coords
landmarks = np.array(landmarks)


def lidar(state, obstacles):
    x = state[0]
    y = state[1]
    theta = state[2]

    num_rays = lidar_num_rays
    max_dist = lidar_max_dist
    min_angle = lidar_min_angle - np.pi / 2
    max_angle = lidar_max_angle - np.pi / 2
    angles = np.linspace(min_angle, max_angle, num_rays)

    lines = []
    for angle in angles:
        end_x = max_dist * np.cos(theta + angle) + x
        end_y = max_dist * np.sin(theta + angle) + y
        l = geom.LineString([[x, y], [end_x, end_y]])
        for obs in obstacles:
            l_temp = l.difference(obs)
            if isinstance(l_temp, geom.MultiLineString):
                l = l_temp.geoms[0]
            else:
                l = l_temp
        lines.append(l)
    return lines


def add_wheel_noise(omega1, omega2):
    sigma1 = np.sqrt(wheel_noise_a1 * omega1 ** 2 + wheel_noise_a2 * omega2 ** 2)
    sigma2 = np.sqrt(wheel_noise_a3 * omega1 ** 2 + wheel_noise_a4 * omega2 ** 2)
    omega1 += np.random.normal(0, sigma1)
    omega2 += np.random.normal(0, sigma2)
    return omega1, omega2


def visible_landmarks(state, landmarks):
    x = state[0]
    y = state[1]
    theta = state[2]

    diffs = landmarks - np.array([x, y])
    dists = np.linalg.norm(diffs, axis=1)
    angs = np.arctan2(diffs[:, 1], diffs[:, 0])
    thetas = angs + (lidar_max_angle - lidar_min_angle) / 2 - theta
    thetas = np.remainder(thetas, 2 * np.pi)
    vis = np.where((dists <= lidar_max_dist) & (thetas >= lidar_min_angle) & (thetas <= lidar_max_angle))

    inds = []
    end_x = x + (dists - 0.001) * np.cos(angs)
    end_y = y + (dists - 0.001) * np.sin(angs)
    for ind, i in enumerate(vis[0]):
        l = geom.LineString([[x, y], [end_x[i], end_y[i]]])
        for obs in obstacles:
            if l.intersects(obs):
                inds.append(ind)
                break
    vis2 = np.delete(vis, inds)

    return zip(vis2, dists[vis2], thetas[vis2])


def calc(actual_state, sensor_state, t):
    omega1 = 0
    omega2 = 0
    try:
        topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
        queue_empty = False
        queue_count = 1
        while not queue_empty:
            try:
                topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                queue_count += 1
            except zmq.ZMQError:
                queue_empty = True
                if queue_count > 1:
                    print("Dropped {} old messages from queue. Receiving more than one message per timestep".format(
                        queue_count - 1))
        message_dict = json.loads(message_str.decode())
        omega1 = message_dict["omega1"]
        omega2 = message_dict["omega2"]
        t += timestep
    except zmq.ZMQError:
        pass

    res = solve_ivp(robot1.deriv, [0, timestep], sensor_state, args=[[omega1, omega2], 0])
    sensor_state = res.y[:, -1]
    sensor_state[2] = sensor_state[2] % (2 * np.pi)
    state_dict = {"timestamp": t, "x": sensor_state[0], "y": sensor_state[1], "theta": sensor_state[2]}
    state_str = json.dumps(state_dict).encode()
    pub_socket.send_multipart([b"model_state", state_str])

    noisy_omega1, noisy_omega2 = add_wheel_noise(omega1, omega2)
    res = solve_ivp(robot1.deriv, [0, timestep], actual_state, args=[[noisy_omega1, noisy_omega2], 0])
    if not robot1.collides(res.y[:, -1], obstacles):
        actual_state = res.y[:, -1]
        actual_state[2] = actual_state[2] % (2 * np.pi)
    state_dict = {"timestamp": t, "x": actual_state[0], "y": actual_state[1], "theta": actual_state[2]}
    state_str = json.dumps(state_dict).encode()
    pub_socket.send_multipart([b"actual_state", state_str])

    collision_dict = {"timestamp": t, "collision": robot1.collides(res.y[:, -1], obstacles)}
    collision_str = json.dumps(collision_dict).encode()
    pub_socket.send_multipart([b"collision", collision_str])

    curr_marks = []
    min_c = np.inf
    for index, dist, theta in visible_landmarks(actual_state, landmarks):
        new = False
        dist_ = dist + np.random.normal(0, lidar_range_sigma)
        theta_ = theta + sensor_state[2] - np.pi / 2 + np.random.normal(0, lidar_bearing_sigma)

        curr_marks.append({"ind": index, "dist": dist_, "theta": theta_ % (2 * np.pi)})

        if int(index) not in found_landmarks:
            new = True
            found_landmarks[int(index)] = {}
            found_landmarks[int(index)]["x"] = 0
            found_landmarks[int(index)]["y"] = 0
            found_landmarks[int(index)]["conf"] = 0

        if found_landmarks[int(index)]["conf"] < 25:
            x_ = sensor_state[0] + dist_ * np.cos(theta_)
            y_ = sensor_state[1] + dist_ * np.sin(theta_)
            found_landmarks[int(index)]["conf"] += 1
            conf = found_landmarks[int(index)]["conf"]
            x = found_landmarks[int(index)]["x"]
            y = found_landmarks[int(index)]["y"]
            found_landmarks[int(index)]["x"] = ((conf - 1) * x + x_) / conf
            found_landmarks[int(index)]["y"] = ((conf - 1) * y + y_) / conf

            if min_c > conf:
                min_c = conf

        if new:
            l = ax1.plot(found_landmarks[int(index)]["x"], found_landmarks[int(index)]["y"], 'k.')[0]
            drawn_marks1.append(l)
            l = ax2.plot(found_landmarks[int(index)]["x"], found_landmarks[int(index)]["y"], 'k.')[0]
            drawn_marks2.append(l)

    # c_dict = {"timestamp": t, "c": min_c}
    # c_str = json.dumps(c_dict).encode()
    # pub_socket.send_multipart([b"c", c_str])

    lines = lidar(actual_state, obstacles)
    dists = [line.length + np.random.normal(0, lidar_range_sigma) for line in lines]

    # if min_c < 25:
    #     return actual_state, sensor_state, lines, t

    if len(curr_marks) > 4:
        x = []
        y = []

        for i in range(len(curr_marks) - 2):
            index1 = curr_marks[i]["ind"]
            x1 = found_landmarks[index1]["x"]
            y1 = found_landmarks[index1]["y"]
            r1 = curr_marks[i]["dist"]

            index2 = curr_marks[i + 1]["ind"]
            x2 = found_landmarks[index2]["x"] - x1
            y2 = found_landmarks[index2]["y"] - y1
            r2 = curr_marks[i + 1]["dist"]

            index3 = curr_marks[i + 2]["ind"]
            x3 = found_landmarks[index3]["x"] - x1
            y3 = found_landmarks[index3]["y"] - y1
            r3 = curr_marks[i + 2]["dist"]

            if x2*x2 + y2*y2 < (r1 + r2)**2 or np.sqrt(x2*x2 + y2*y2) + min(r1, r2) < max(r1, r2):
                continue

            if x3*x3 + y3*y3 < (r1 + r3)**2 or np.sqrt(x3*x3 + y3*y3) + min(r1, r3) < max(r1, r3):
                continue

            a1 = x2
            b1 = y2
            c1 = (x2 * x2 + y2 * y2 - r2 * r2 + r1 * r1) / 2

            a2 = x3
            b2 = y3
            c2 = (x3 * x3 + y3 * y3 - r3 * r3 + r1 * r1) / 2

            d = a1 * b2 - a2 * b1

            if d != 0:
                x0 = (c1 * b2 - c2 * b1) / d
                y0 = (a1 * c2 - a2 * c1) / d
                x.append(x1 + x0)
                y.append(y1 + y0)

        if len(x) > 0:
            sensor_state[0] = (sensor_state[0] + sum(x)) / (len(x) + 1)
            sensor_state[1] = (sensor_state[1] + sum(y)) / (len(y) + 1)

    state_dict = {"timestamp": t, "x": sensor_state[0], "y": sensor_state[1], "theta": sensor_state[2]}
    state_str = json.dumps(state_dict).encode()
    pub_socket.send_multipart([b"sensor_state", state_str])

    lidar_dict = {"timestamp": t, "distances": dists}
    lidar_str = json.dumps(lidar_dict).encode()
    pub_socket.send_multipart([b"lidar", lidar_str])

    return actual_state, sensor_state, lines, t


def producer():
    actual_state = [start[0], start[1], 0]
    sensor_state = [start[0], start[1], 0]
    t = 0
    while True:
        actual_state, sensor_state, lines, t = calc(actual_state, sensor_state, t)
        yield actual_state, sensor_state, lines


drawn_lines1 = []
drawn_lines2 = []
drawn_marks1 = []
drawn_marks2 = []
drawn_marks3 = []
drawn_marks4 = []


def animate(data):
    state, sensor, lines = data
    patches1 = robot1.draw(ax1, state)
    patches2 = robot2.draw(ax2, state)

    for line, drawn_line in zip(lines, drawn_lines1):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)

    for line, drawn_line in zip(lines, drawn_lines2):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)

    for mark, drawn_mark in zip(found_landmarks.items(), drawn_marks1):
        try:
            x, y = mark[1]['x'], mark[1]['y']
            drawn_mark.set_data(x, y)
        except NotImplementedError:
            print(mark)

    for mark, drawn_mark in zip(found_landmarks.items(), drawn_marks2):
        try:
            x, y = mark[1]['x'], mark[1]['y']
            drawn_mark.set_data(x, y)
        except NotImplementedError:
            print(mark)

    for drawn_mark in drawn_marks3:
        try:
            drawn_mark.set_data(sensor[0], sensor[1])
        except NotImplementedError:
            print(sensor)

    for drawn_mark in drawn_marks4:
        try:
            drawn_mark.set_data(sensor[0], sensor[1])
        except NotImplementedError:
            print(sensor)

    window_size = 1
    ax1.set_xlim(state[0] - window_size, state[0] + window_size)
    ax1.set_ylim(state[1] - window_size, state[1] + window_size)
    return *obs_patches1, *patches1, *patches2, *drawn_lines1, *drawn_lines2, *drawn_marks1, *drawn_marks2, *drawn_marks3, *drawn_marks4


fig, [ax1, ax2] = plt.subplots(1, 2)

obs_patches1 = []
for obs in obstacles:
    patch = dc.PolygonPatch(obs)
    ax1.add_patch(patch)
    obs_patches1.append(patch)
    patch = dc.PolygonPatch(obs)
    ax2.add_patch(patch)

ax2.plot(start[0], start[1], 'g.')
ax2.plot(goal[0], goal[1], 'r.')

state, sensor, lines, _ = calc([start[0], start[1], 0], [start[0], start[1], 0], 0)
patches1 = robot1.draw(ax1, state)
patches2 = robot2.draw(ax2, state)

for patch in patches1:
    ax1.add_patch(patch)

for patch in patches2:
    ax2.add_patch(patch)

for line in lines:
    x, y = line.xy
    l = ax1.plot(x, y, 'g')[0]
    drawn_lines1.append(l)
    l = ax2.plot(x, y, 'g')[0]
    drawn_lines2.append(l)

l = ax1.plot(sensor[0], sensor[1], 'r.')[0]
drawn_marks3.append(l)
l = ax2.plot(sensor[0], sensor[1], 'r.')[0]
drawn_marks4.append(l)

ax1.axis("equal")
ax2.axis("equal")

time_scale = 1  # Make this number bigger to slow down time
animation = anim.FuncAnimation(fig, animate, producer, interval=timestep * 1000 * time_scale, blit=True)

plt.show(block=True)
