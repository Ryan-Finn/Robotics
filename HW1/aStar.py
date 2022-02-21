"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from numpy.random import rand

np.random.seed(6)

show_animation = True


class Grid:
    def __init__(self, sx=0, sy=0, gx=0, gy=0, resolution=1, rr=0):
        self.start = (sx, sy)
        self.goal = (gx, gy)
        self.resolution = resolution
        self.rr = rr
        self.width, self.height = 0, 0
        self.r1, self.r2 = [], []
        self.init()
        self.obstacle_map = [[False for _ in self.r2]
                             for _ in self.r1]
        self.motion = [[-1, 0, 1],
                       [0, 1, 1],
                       [1, 0, 1],
                       [0, -1, 1],
                       [-1, -1, math.sqrt(2)],
                       [-1, 1, math.sqrt(2)],
                       [1, 1, math.sqrt(2)],
                       [1, -1, math.sqrt(2)]]

    def copy(self, grid):
        self.start = grid.start
        self.goal = grid.goal
        self.resolution = grid.resolution
        self.rr = grid.rr
        self.width, self.height = grid.width, grid.height
        self.r1, self.r2 = grid.r1, grid.r2
        self.obstacle_map = grid.obstacle_map
        self.motion = grid.motion

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __eq__(self, n2):
            return self.x == n2.x and self.y == n2.y

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def aStar(self, start=None, goal=None, checked_set=None, corner_set2=None, single=False):
        print("Exploring...")

        if start is None:
            start = self.start
        if goal is None:
            goal = self.goal
        if checked_set is None:
            checked_set = dict()
        if corner_set2 is None:
            corner_set2 = dict()

        start_node = self.Node(self.calc_xy_index(start[0], self.start[0]),
                               self.calc_xy_index(start[1], self.start[1]), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal[0], self.start[0]),
                              self.calc_xy_index(goal[1], self.start[1]), 0.0, -1)

        corner_set = dict()
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        open_set2, closed_set2 = dict(), dict()
        open_set2[self.calc_grid_index(goal_node)] = goal_node
        current2, c_id2 = None, None

        while 1:
            if len(open_set) == 0:
                print("No path exists")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if not single:
                c_id2 = min(open_set2, key=lambda o: open_set2[o].cost + self.calc_heuristic(start_node, open_set2[o]))
                current2 = open_set2[c_id2]

            # show graph
            if show_animation:
                if single:
                    plt.plot(self.calc_grid_position(current.x, self.start[0]),
                             self.calc_grid_position(current.y, self.start[1]), "1r")
                else:
                    plt.plot(self.calc_grid_position(current.x, self.start[0]),
                             self.calc_grid_position(current.y, self.start[1]), "+y")
                    plt.plot(self.calc_grid_position(current2.x, self.start[0]),
                             self.calc_grid_position(current2.y, self.start[1]), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            exist = False
            if single:
                if current == goal_node:
                    exist = True
            else:
                c_gd = self.calc_grid_index(current)
                c_gd2 = self.calc_grid_index(current2)
                if c_gd in closed_set2:
                    exist = True
                    corner_set[c_gd] = current
                elif c_gd2 in closed_set:
                    exist = True
                    current = current2
                    corner_set[c_gd2] = current2

            if exist and len(corner_set2) == 0:
                print("A path exists")
                if show_animation:
                    plt.plot(self.calc_grid_position(current.x, self.start[0]),
                             self.calc_grid_position(current.y, self.start[1]), "or")
                corner_set[self.calc_grid_index(start_node)] = start_node
                corner_set[self.calc_grid_index(goal_node)] = goal_node
                if single:
                    return self.jump_point(start_node, goal_node, goal_node, corner_set)
                else:
                    return self.jump_point(start_node, current, goal_node, corner_set)

            del open_set[c_id]
            closed_set[c_id] = current
            self.expand_grid(current, c_id, open_set, closed_set, corner_set)

            for _, corner in corner_set.items():
                if self.visible(start_node, corner) and self.calc_grid_index(corner) not in corner_set2:
                    for _, node in checked_set.items():
                        if self.visible(node, corner):
                            corner_set2[self.calc_grid_index(corner)] = corner
                            plt.plot(self.calc_grid_position(corner.x, self.start[0]),
                                     self.calc_grid_position(corner.y, self.start[1]), "^r")
                            return

            if not single:
                del open_set2[c_id2]
                closed_set2[c_id2] = current2
                self.expand_grid(current2, c_id2, open_set2, closed_set2, corner_set)

    def expand_grid(self, current, c_id, open_set, closed_set, corner_set):
        obs = []
        for i, _ in enumerate(self.motion):
            node = self.Node(current.x + self.motion[i][0],
                             current.y + self.motion[i][1],
                             current.cost + self.motion[i][2], c_id)
            n_id = self.calc_grid_index(node)

            if self.obstacle_node(node):
                obs.append(i)
                continue

            if self.bounds_node(node) or n_id in closed_set:
                continue

            if n_id not in open_set:
                open_set[n_id] = node  # discovered a new node
            elif open_set[n_id].cost > node.cost:  # This path is the best until now. record it
                open_set[n_id] = node

        corner = False
        if 4 in obs and 3 not in obs and 0 not in obs or \
                5 in obs and 0 not in obs and 1 not in obs or \
                6 in obs and 1 not in obs and 2 not in obs or \
                7 in obs and 2 not in obs and 3 not in obs:
            corner_set[c_id] = self.Node(current.x, current.y, np.inf, -1)
            corner = True

        if corner and show_animation:
            plt.plot(self.calc_grid_position(current.x, self.start[0]),
                     self.calc_grid_position(current.y, self.start[1]), "^m")

    def jump_point(self, start, intersect, goal, corner_set):
        print("Finding path...")
        print(start.parent_index)
        print(intersect.parent_index)
        print(goal.parent_index)
        grid = Grid()
        grid.copy(self)
        open_set, open_set2, closed_set = dict(), dict(), dict()

        intersect.parent_index = -1
        open_set[self.calc_grid_index(intersect)] = intersect
        open_set2[self.calc_grid_index(intersect)] = intersect
        current, current2 = intersect, intersect
        c_id, c_id2 = self.calc_grid_index(intersect), self.calc_grid_index(intersect)

        while 1:
            if len(open_set) == 0:
                print("bah---------------------------------")
                grid.aStar((intersect.x, intersect.y), (start.x, start.y), closed_set, corner_set, True)
                closed_set = dict()
                open_set[self.calc_grid_index(intersect)] = intersect
                current = intersect
                c_id = self.calc_grid_index(intersect)
            elif len(open_set2) == 0:
                print("meh---------------------------------")
                grid.aStar((intersect.x, intersect.y), (goal.x, goal.y), closed_set, corner_set, True)
                closed_set = dict()
                open_set2[self.calc_grid_index(intersect)] = intersect
                current2 = intersect
                c_id2 = self.calc_grid_index(intersect)

            if current != start:
                c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(start, open_set[o]))
                current = open_set[c_id]
            else:
                c_id2 = min(open_set2, key=lambda o: open_set2[o].cost + self.calc_heuristic(goal, open_set2[o]))
                current2 = open_set2[c_id2]

            if show_animation:
                if current != start:
                    plt.plot(self.calc_grid_position(current.x, self.start[0]),
                             self.calc_grid_position(current.y, self.start[1]), "^g")
                else:
                    plt.plot(self.calc_grid_position(current2.x, self.start[0]),
                             self.calc_grid_position(current2.y, self.start[1]), "^g")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 1 == 0:
                    plt.pause(0.001)

            if current == start and current2 == goal:
                print("Goal Found!")
                start.parent_index = current.parent_index
                self.calc_final_path(intersect, start, closed_set)
                # self.calc_final_path(intersect, current2, closed_set)
                return

            if current != start:
                del open_set[c_id]
                closed_set[c_id] = current
                self.expand_corners(current, c_id, open_set, closed_set, corner_set)
            else:
                del open_set2[c_id2]
                closed_set[c_id2] = current2
                self.expand_corners(current2, c_id2, open_set2, closed_set, corner_set)

    def expand_corners(self, current, c_id, open_set, closed_set, corner_set):
        for _, corner in corner_set.items():
            node = self.Node(corner.x, corner.y, current.cost + self.calc_heuristic(current, corner), c_id)
            n_id = self.calc_grid_index(node)

            if n_id in closed_set:
                continue
            if self.visible(node, current):
                continue
            if n_id not in open_set or open_set[n_id].cost > node.cost:
                open_set[n_id] = node

    def visible(self, node, current):
        rise = node.y - current.y
        run = node.x - current.x

        for x in range(min(current.x, node.x) + 1, max(current.x, node.x) - 1):
            m = rise / run
            b = current.y - current.x * m
            y = m * x + b
            if self.obstacle_map[x][round(y)]:
                node.parent_index = -1
                node.cost = np.inf
                return False
        for y in range(min(current.y, node.y) + 1, max(current.y, node.y) - 1):
            n = run / rise
            d = current.x - current.y * n
            x = n * y + d
            if self.obstacle_map[round(x)][y]:
                node.parent_index = -1
                node.cost = np.inf
                return False
        return True

    def calc_final_path(self, start_node, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.start[0])], [
            self.calc_grid_position(goal_node.y, self.start[1])]
        parent_index = goal_node.parent_index
        print("--   ", parent_index)
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.start[0]))
            ry.append(self.calc_grid_position(n.y, self.start[1]))
            parent_index = n.parent_index
            print(parent_index)
        if show_animation:
            for i in range(0, len(rx) - 1):
                plt.plot(rx[i], ry[i], "ob")
                plt.plot((rx[i], rx[i + 1]), (ry[i], ry[i + 1]), "-b")
                plt.pause(0.001)
            plt.plot(0, 0, "ob")

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.start[1]) * self.width + (node.x - self.start[0])

    def obstacle_node(self, node):
        if self.obstacle_map[node.x][node.y]:
            return True
        return False

    def bounds_node(self, node):
        px = self.calc_grid_position(node.x, self.start[0])
        py = self.calc_grid_position(node.y, self.start[1])

        if px < self.start[0] - self.width / 4:
            return True
        elif py < self.start[1] - self.height / 4:
            return True
        elif px > self.goal[0] + self.width / 4:
            return True
        elif py > self.goal[1] + self.height / 4:
            return True
        return False

    def init(self):
        self.width = 2 * round((self.goal[0] - self.start[0]) / self.resolution)
        self.height = 2 * round((self.goal[1] - self.start[1]) / self.resolution)
        self.r1 = range(math.floor(self.start[0] - self.width / 4), math.ceil(self.goal[0] + self.width / 4))
        self.r2 = range(math.floor(self.start[1] - self.height / 4), math.ceil(self.goal[1] + self.height / 4))

    def calc_obstacle_map(self, ox, oy):
        # obstacle map generation
        for ix in self.r1:
            x = self.calc_grid_position(ix, self.start[0])
            for iy in self.r2:
                y = self.calc_grid_position(iy, self.start[1])
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break


def main():
    print(__file__ + "  Press Esc to exit")

    # start and goal position
    sx = 0
    sy = 0
    gx = 35
    gy = 35
    grid_size = 1
    robot_radius = 1
    grid = Grid(sx, sy, gx, gy, grid_size, robot_radius)

    if show_animation:
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")
        plt.pause(1)

    # a bunch of random elliptical obstacles
    ells = []
    w = gx - sx
    h = gy - sy
    for _ in range(15):
        el = Ellipse(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 15 + 5 + robot_radius,
                     height=rand() * 15 + 5 + robot_radius, angle=rand() * 360)
        while el.contains_point((sx, sy)) or el.contains_point((gx, gy)):
            el = Ellipse(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 15 + 5 + robot_radius,
                         height=rand() * 15 + 5 + robot_radius, angle=rand() * 360)
        el.width -= robot_radius
        el.height -= robot_radius
        ells.append(el)

    # discretize each ellipse
    print("Creating Obstacles...")
    for el in ells:
        h = math.ceil(max(el.width, el.height) / 2)
        h = h + h % grid_size
        ex = el.center[0] - h
        ex = ex - ex % grid_size
        ey = el.center[1] - h
        ey = ey - ey % grid_size

        ox, oy = [], []
        for i in range(int(ex), int(ex + 2 * h), grid_size):
            for j in range(int(ey), int(ey + 2 * h), grid_size):
                if el.contains_point((i, j)):
                    ox.append(i)
                    oy.append(j)
        grid.calc_obstacle_map(ox, oy)
        if show_animation:
            plt.plot(ox, oy, ".k")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    grid.aStar(single=False)
    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
