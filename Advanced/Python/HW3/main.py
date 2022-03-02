from aStar import Grid
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from numpy.random import rand

np.random.seed(11)
show_animation = True
single_sided_astar = False


def main():
    print(__file__ + "  Press Esc to exit")

    # start and goal position
    sx = 0
    sy = 0
    gx = 35
    gy = 35
    robot_radius = 0.5
    grid = Grid(sx, sy, gx, gy, robot_radius, single_sided_astar, False, show_animation)

    if show_animation:
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")
        plt.pause(1)

    # a bunch of random elliptical obstacles
    obs = []
    w = gx - sx
    h = gy - sy
    for _ in range(15):
        ob = Rectangle(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
                       height=rand() * 16 + 4 + robot_radius, angle=(0, 90)[round(rand())])
        while ob.contains_point((sx, sy)) or ob.contains_point((gx, gy)):
            ob = Rectangle(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
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
