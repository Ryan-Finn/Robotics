import math
import numpy as np
import matplotlib.pyplot as plt
from aStar import Grid


class Brushfire:
    def __init__(self, grid):
        self.grid = grid
        self.gvd = [[self.Node(x, y, [x, y]) for y in grid.r2] for x in grid.r1]
        self.get_gvd()

    class Node:
        def __init__(self, x, y, parent_index):
            self.x = x
            self.y = y
            self.parent_index = parent_index

        def __eq__(self, n2):
            return self.x == n2.x and self.y == n2.y

        def __str__(self):
            return str(self.x) + ", " + str(self.y) + ", " + str(self.parent_index)

        def same_parent(self, node):
            if self.parent_index + [0, 1] == node.parent_index:
                return True
            if self.parent_index + [1, 0] == node.parent_index:
                return True
            if self.parent_index + [0, -1] == node.parent_index:
                return True
            if self.parent_index + [-1, 0] == node.parent_index:
                return True
            return False

    def get_gvd(self):
        for row in self.gvd:
            for node in row:
                print(node)
