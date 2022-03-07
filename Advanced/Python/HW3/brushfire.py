import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class Brushfire:
    def __init__(self, grid):
        self.grid = grid
        global show_animation
        show_animation = grid.show
        self.minX = grid.r1[0]
        self.minY = grid.r2[0]
        self.gvd = [[self.Node(0, x, y, [x, y]) for y in grid.r2] for x in grid.r1]
        self.moves = [0, 1,
                      1, 0,
                      0, -1,
                      -1, 0,
                      1, 1,
                      1, -1,
                      -1, -1,
                      -1, 1]
        self.get_gvd()

    class Node:
        def __init__(self, val, x, y, parent_index):
            self.val = val
            self.x = x
            self.y = y
            self.parent_index = parent_index

        def __eq__(self, n2):
            return self.val == n2.val

        def same_parent(self, node):
            return self.parent_index == node.parent_index

    def get_gvd(self):
        open_set = []
        temp_set = []

        for i in range(len(self.grid.r1)):
            x = i + self.minX
            for j in range(len(self.grid.r2)):
                y = j + self.minY
                node = self.gvd[i][j]
                if not self.grid.obstacle_map[x][y]:
                    open_set.append(node)
                else:
                    node.val = 1
                    temp_set.append(node)
                    for k in range(len(self.moves) // 4):
                        x2 = i + self.moves[2 * k]
                        y2 = j + self.moves[2 * k + 1]
                        if len(self.grid.r1) > x2 > -1 and len(self.grid.r2) > y2 > -1:
                            if self.grid.obstacle_map[x2 + self.minX][y2 + self.minY]:
                                self.gvd[x2][y2].parent_index = node.parent_index
        if show_animation:
            line = plt.plot(0, 0)
        while len(open_set) > 0:
            target_set = temp_set.copy()
            temp_set = []
            for node in target_set:
                for i in range(len(self.moves) // 2):
                    x = node.x + self.moves[2 * i]
                    y = node.y + self.moves[2 * i + 1]
                    if len(self.grid.r1) > x - self.minX > -1 and len(self.grid.r2) > y - self.minY > -1:
                        neighbor = self.gvd[x - self.minX][y - self.minY]
                        if neighbor.val == 0:
                            self.grid.obstacle_map[x][y] = True
                            neighbor.parent_index = node.parent_index
                            neighbor.val = node.val + 1
                            temp_set.append(neighbor)
                            open_set.remove(neighbor)
                            # if show_animation:
                                # line = plt.plot(x, y, "s", color=line[0].get_color())
                        elif neighbor.val >= node.val and not node.same_parent(neighbor):
                            self.grid.obstacle_map[x][y] = False
                            neighbor.val = np.inf
                            if show_animation:
                                plt.plot(x, y, "s", color="#888888")
            if show_animation:
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                plt.pause(0.001)
                line = plt.plot(0, 0)

        for node in temp_set:
            for i in range(len(self.moves) // 2):
                x = node.x + self.moves[2 * i]
                y = node.y + self.moves[2 * i + 1]
                if len(self.grid.r1) > x - self.minX > -1 and len(self.grid.r2) > y - self.minY > -1:
                    neighbor = self.gvd[x - self.minX][y - self.minY]
                    if neighbor.val >= node.val and not node.same_parent(neighbor):
                        self.grid.obstacle_map[x][y] = False
                        neighbor.val = np.inf
                        if show_animation:
                            plt.plot(x, y, "s", color="#888888")
        if show_animation:
            plt.pause(0.001)

        self.grid.obstacle_map[self.grid.start[0]][self.grid.start[1]] = False
        self.grid.obstacle_map[self.grid.goal[0]][self.grid.goal[1]] = False

        node = self.gvd[self.grid.start[0] - self.minX][self.grid.start[1] - self.minY]
        while node.val != np.inf:
            for i in range(len(self.moves) // 2):
                x = node.x + self.moves[2 * i] - self.minX
                y = node.y + self.moves[2 * i + 1] - self.minY
                neighbor = self.gvd[x][y]
                if neighbor.val > node.val:
                    node = self.gvd[x][y]
                    self.grid.obstacle_map[x + self.minX][y + self.minY] = False
                    if show_animation:
                        plt.plot(x + self.minX, y + self.minX, "s", color="#888888")
                    break

        node = self.gvd[self.grid.goal[0] - self.minX][self.grid.goal[1] - self.minY]
        while node.val != np.inf:
            for i in range(len(self.moves) // 2):
                x = node.x + self.moves[2 * i] - self.minX
                y = node.y + self.moves[2 * i + 1] - self.minY
                neighbor = self.gvd[x][y]
                if neighbor.val > node.val:
                    node = self.gvd[x][y]
                    self.grid.obstacle_map[x + self.minX][y + self.minY] = False
                    if show_animation:
                        plt.plot(x + self.minX, y + self.minX, "s", color="#888888")
                    break
