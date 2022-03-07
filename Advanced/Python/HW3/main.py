import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from numpy.random import rand
from aStar import Grid
from brushfire import Brushfire

np.random.seed(7)
show_animation = True
single_sided_astar = True


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

    ox = [x for x in range(sx - 5, gx + 6)]
    oy = [sy - 5 for _ in range(sy - 5, gy + 6)]
    grid.calc_obstacle_map(ox, oy)
    if show_animation:
        plt.plot(ox, oy, "sk")

    ox = [x for x in range(sx - 5, gx + 6)]
    oy = [gy + 5 for _ in range(sy - 5, gy + 6)]
    grid.calc_obstacle_map(ox, oy)
    if show_animation:
        plt.plot(ox, oy, "sk")

    ox = [sx - 5 for _ in range(sx - 5, gx + 6)]
    oy = [y for y in range(sy - 5, gy + 6)]
    grid.calc_obstacle_map(ox, oy)
    if show_animation:
        plt.plot(ox, oy, "sk")

    ox = [gx + 5 for _ in range(sx - 5, gx + 6)]
    oy = [y for y in range(sy - 5, gy + 6)]
    grid.calc_obstacle_map(ox, oy)
    if show_animation:
        plt.plot(ox, oy, "sk")

    # 4 random rectangular obstacles, one in each quadrant
    print("Creating Obstacles...")
    obs = []
    w = gx - sx
    h = gy - sy
    # Quad 1
    ob = Rectangle(xy=(rand() * w / 2 + w / 2 - 7, rand() * h / 2 + h / 2 - 7), width=rand() * 8 + 4 + robot_radius,
                   height=rand() * 8 + 4 + robot_radius)
    while ob.contains_point((sx, sy)) or ob.contains_point((gx, gy)):
        ob = Rectangle(xy=(rand() * w / 2 + w / 2 - 7, rand() * h / 2 + h / 2 - 7), width=rand() * 8 + 4 + robot_radius,
                       height=rand() * 8 + 4 + robot_radius)
    ob.set_width(ob.get_width() - robot_radius)
    ob.set_height(ob.get_height() - robot_radius)
    obs.append(ob)

    # Quad 2
    ob = Rectangle(xy=(rand() * w / 2, rand() * h / 2 + h / 2 - 7), width=rand() * 8 + 4,
                   height=rand() * 8 + 4)
    obs.append(ob)

    # Quad 3
    ob = Rectangle(xy=(rand() * w / 2, rand() * h / 2), width=rand() * 8 + 4 + robot_radius,
                   height=rand() * 8 + 4 + robot_radius)
    while ob.contains_point((sx, sy)) or ob.contains_point((gx, gy)):
        ob = Rectangle(xy=(rand() * w / 2, rand() * h / 2), width=rand() * 8 + 4 + robot_radius,
                       height=rand() * 8 + 4 + robot_radius)
    ob.set_width(ob.get_width() - robot_radius)
    ob.set_height(ob.get_height() - robot_radius)
    obs.append(ob)

    # Quad 4
    ob = Rectangle(xy=(rand() * w / 2 + w / 2 - 7, rand() * h / 2), width=rand() * 8 + 4,
                   height=rand() * 8 + 4)
    obs.append(ob)

    # discretize each rectangle
    for ob in obs:
        x = ob.get_x()
        y = ob.get_y()

        ox, oy = [], []
        for i in range(int(x), int(x + ob.get_width())):
            for j in range(int(y), int(y + ob.get_height())):
                if ob.contains_point((i, j)):
                    ox.append(i)
                    oy.append(j)
        grid.calc_obstacle_map(ox, oy)
        if show_animation:
            plt.plot(ox, oy, "sk")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.pause(0.01)

    Brushfire(grid)
    grid.aStar(single=single_sided_astar)
    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
