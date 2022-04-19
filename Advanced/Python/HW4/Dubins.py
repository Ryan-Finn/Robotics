import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString

half = math.pi / 2
tau = 2 * math.pi


def calcRadiusAngles(center, r, p):
    x0 = p[0] - center[0]
    y0 = p[1] - center[1]
    d2 = x0**2 + y0**2
    a = r**2 * np.array([x0, y0]) / d2
    b = r * math.sqrt(d2 - r**2) * np.array([-y0, x0]) / d2
    c1 = a + b
    c2 = a - b

    return math.atan2(c1[1], c1[0]) % tau, math.atan2(c2[1], c2[0]) % tau


def makeRightArc(center, r, t0, t1):
    tStop = (t0 - t1) % tau
    ang = np.linspace(0, tStop, round(72 * tStop / tau)) + t1

    arc = [(center[0] + r * math.cos(a), center[1] + r * math.sin(a)) for a in ang]
    arc.reverse()

    return arc


def makeLeftArc(center, r, t0, t1):
    tStop = (t1 - t0) % tau
    ang = np.linspace(0, tStop, round(72 * tStop / tau)) + t0

    arc = [(center[0] + r * math.cos(a), center[1] + r * math.sin(a)) for a in ang]

    return arc


class Dubins:
    def __init__(self, start, goal, r, t0=None, t1=None):
        self.start = start
        self.goal = goal
        self.r = r
        if t0 is None:
            t0 = math.atan2(goal[1] - start[1], goal[0] - start[0]) % tau

        if t1 is None:
            t1 = math.atan2(goal[1] - start[1], goal[0] - start[0]) % tau

        self.t0 = t0
        self.t1 = t1

        self.r0 = (t0 - half) % tau
        self.r1 = (t1 - half) % tau
        self.l0 = (t0 + half) % tau
        self.l1 = (t1 + half) % tau

    def RSR(self):
        start = self.start
        goal = self.goal
        r = self.r
        r0 = self.r0
        r1 = self.r1

        x0 = start[0] + r * math.cos(r0)
        y0 = start[1] + r * math.sin(r0)
        x1 = goal[0] + r * math.cos(r1)
        y1 = goal[1] + r * math.sin(r1)

        a = (math.atan2(y1 - y0, x1 - x0) + half) % tau
        dx = r * math.cos(a)
        dy = r * math.sin(a)

        arc = makeRightArc((x0, y0), r, (r0 + math.pi) % tau, a)
        arc.append((x0 + dx, y0 + dy))
        arc.append((x1 + dx, y1 + dy))

        for el in makeRightArc((x1, y1), r, a, (r1 + math.pi) % tau):
            arc.append(el)

        return LineString(arc)

    def LSL(self):
        start = self.start
        goal = self.goal
        r = self.r
        l0 = self.l0
        l1 = self.l1

        x0 = start[0] + r * math.cos(l0)
        y0 = start[1] + r * math.sin(l0)
        x1 = goal[0] + r * math.cos(l1)
        y1 = goal[1] + r * math.sin(l1)

        a = (math.atan2(y1 - y0, x1 - x0) - half) % tau
        dx = r * math.cos(a)
        dy = r * math.sin(a)

        arc = makeLeftArc((x0, y0), r, (l0 + math.pi) % tau, a)
        arc.append((x0 + dx, y0 + dy))
        arc.append((x1 + dx, y1 + dy))

        for el in makeLeftArc((x1, y1), r, a, (l1 + math.pi) % tau):
            arc.append(el)

        return LineString(arc)

    def RSL(self):
        start = self.start
        goal = self.goal
        r = self.r
        r0 = self.r0
        l1 = self.l1

        x0 = start[0] + r * math.cos(r0)
        y0 = start[1] + r * math.sin(r0)
        x1 = goal[0] + r * math.cos(l1)
        y1 = goal[1] + r * math.sin(l1)

        if (x1 - x0)**2 + (y1 - y0)**2 < 4 * r**2:
            return None

        a, _ = calcRadiusAngles((x0, y0), 2 * r, (x1, y1))
        dx0 = r * math.cos(a)
        dy0 = r * math.sin(a)
        dx1 = r * math.cos(a + math.pi)
        dy1 = r * math.sin(a + math.pi)

        arc = makeRightArc((x0, y0), r, (r0 + math.pi) % tau, a)
        arc.append((x0 + dx0, y0 + dy0))
        arc.append((x1 + dx1, y1 + dy1))

        for el in makeLeftArc((x1, y1), r, (a + math.pi) % tau, (l1 + math.pi) % tau):
            arc.append(el)

        return LineString(arc)

    def LSR(self):
        start = self.start
        goal = self.goal
        r = self.r
        l0 = self.l0
        r1 = self.r1

        x0 = start[0] + r * math.cos(l0)
        y0 = start[1] + r * math.sin(l0)
        x1 = goal[0] + r * math.cos(r1)
        y1 = goal[1] + r * math.sin(r1)

        if (x1 - x0)**2 + (y1 - y0)**2 < 4 * r**2:
            return None

        _, a = calcRadiusAngles((x0, y0), 2 * r, (x1, y1))
        dx0 = r * math.cos(a)
        dy0 = r * math.sin(a)
        dx1 = r * math.cos(a + math.pi)
        dy1 = r * math.sin(a + math.pi)

        arc = makeLeftArc((x0, y0), r, (l0 + math.pi) % tau, a)
        arc.append((x0 + dx0, y0 + dy0))
        arc.append((x1 + dx1, y1 + dy1))

        for el in makeRightArc((x1, y1), r, (a + math.pi) % tau, (r1 + math.pi) % tau):
            arc.append(el)

        return LineString(arc)

    def getPaths(self):
        paths = list(filter(None, [self.RSR(), self.LSL(), self.RSL(), self.LSR()]))
        paths.sort(key=lambda line: line.length)

        return paths, self.t1


if __name__ == '__main__':
    start = (5, 5)
    goal = (15, 15)
    # goal = (16.178745736248718, 4.881368804433764)
    d = Dubins(start, goal, 2, half / 2, half)

    plt.plot(start[0], start[1], 'xg')
    plt.plot(goal[0], goal[1], 'xb')
    paths, _ = d.getPaths()

    #for path in paths:
    plt.plot(*paths[0].xy)
    plt.axis("equal")
    plt.show()
