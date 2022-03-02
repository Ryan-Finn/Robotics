from aStar import Grid
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from numpy.random import rand

# np.random.seed(6)
maze = False
show_animation = True
single_sided_astar = False


def boundary_and_obstacles(top_vertex, bottom_vertex, obs_number=1500):
    # below can be merged into a rectangle boundary
    ay = list(range(bottom_vertex[1], top_vertex[1]))
    ax = [bottom_vertex[0]] * len(ay)
    cy = ay
    cx = [top_vertex[0]] * len(cy)
    bx = list(range(bottom_vertex[0] + 1, top_vertex[0]))
    by = [bottom_vertex[1]] * len(bx)
    dx = [bottom_vertex[0]] + bx + [top_vertex[0]]
    dy = [top_vertex[1]] * len(dx)

    # generate random obstacles
    ob_x = np.random.randint(bottom_vertex[0] + 1, top_vertex[0], obs_number).tolist()
    ob_y = np.random.randint(bottom_vertex[1] + 1, top_vertex[1], obs_number).tolist()
    # x y coordinate in certain order for boundary
    x = ax + bx + cx + dx
    y = ay + by + cy + dy
    obstacle = np.vstack((ob_x, ob_y)).T.tolist()
    obs_array = np.array(obstacle)
    bound = np.vstack((x, y)).T
    bound_obs = np.vstack((bound, obs_array))
    return bound_obs


def main():
    print(__file__ + "  Press Esc to exit")

    # start and goal position
    sx = 0
    sy = 0
    gx = 35
    gy = 35
    robot_radius = 0.5
    grid = Grid(sx, sy, gx, gy, robot_radius, single_sided_astar, maze, show_animation)

    if show_animation:
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")
        plt.pause(1)

    if maze:
        print("Creating Obstacles...")
        obs = boundary_and_obstacles((gx + 10, gy + 10), (sx - 10, sy - 10))
        ox, oy = [], []
        for ob in obs:
            if (ob[0] != sx or ob[1] != sy) and (ob[0] != gx or ob[1] != gy):
                ox.append(ob[0])
                oy.append(ob[1])
                if show_animation:
                    if len(ox) % 150 == 0:
                        plt.plot(ox, oy, "sk")
                        plt.gcf().canvas.mpl_connect('key_release_event',
                                                     lambda event: [exit(0) if event.key == 'escape' else None])
                        plt.pause(0.001)
                        grid.calc_obstacle_map(ox, oy)
                        ox, oy = [], []
        grid.calc_obstacle_map(ox, oy)
    else:
        # a bunch of random elliptical obstacles
        obs = []
        w = gx - sx
        h = gy - sy
        for _ in range(15):
            ob = Ellipse(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
                         height=rand() * 16 + 4 + robot_radius, angle=rand() * 360)
            while ob.contains_point((sx, sy)) or ob.contains_point((gx, gy)):
                ob = Ellipse(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
                             height=rand() * 16 + 4 + robot_radius, angle=rand() * 360)
            ob.width -= robot_radius
            ob.height -= robot_radius
            obs.append(ob)

        # discretize each ellipse
        print("Creating Obstacles...")
        for ob in obs:
            h = math.ceil(max(ob.width, ob.height) / 2)
            x = ob.center[0] - h
            y = ob.center[1] - h

            ox, oy = [], []
            for i in range(int(x), int(x + 2 * h)):
                for j in range(int(y), int(y + 2 * h)):
                    if ob.contains_point((i, j)):
                        ox.append(i)
                        oy.append(j)
            grid.calc_obstacle_map(ox, oy)
            if show_animation:
                plt.plot(ox, oy, "sk")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                plt.pause(0.001)

    grid.aStar(single=single_sided_astar)
    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
