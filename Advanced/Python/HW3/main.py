import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from numpy.random import rand
from aStar import Grid
from brushfire import Brushfire

# np.random.seed(11)
show_animation = False
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

    # a few random rectangular obstacles
    obs = []
    w = gx - sx
    h = gy - sy
    for _ in range(3):
        ob = Rectangle(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
                       height=rand() * 16 + 4 + robot_radius)
        while ob.contains_point((sx, sy)) or ob.contains_point((gx, gy)):
            ob = Rectangle(xy=(rand() * w + sx, rand() * h + sy), width=rand() * 16 + 4 + robot_radius,
                           height=rand() * 16 + 4 + robot_radius)
        ob.set_width(ob.get_width() - robot_radius)
        ob.set_height(ob.get_height() - robot_radius)
        obs.append(ob)

    # discretize each rectangle
    print("Creating Obstacles...")
    for ob in obs:
        x = ob.get_x() - ob.get_width() / 2
        y = ob.get_y() - ob.get_height() / 2

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
            plt.pause(0.001)

    Brushfire(grid)
    # grid.aStar(single=single_sided_astar)
    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
