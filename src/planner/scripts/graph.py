import cv2 as cv
import numpy as np
from math import pi, sqrt, atan, sin, cos
from datetime import datetime

class Graph():
    def __init__(self, w=800, h=600, grid_size=20):
        """
        :param w: window width - pixels
        :param h: window height - pixels
        :param delta_r: radius step - pixels
        :param delta_fi: angle step - degrees
        """
        self.gs = grid_size
        self.w = w
        self.h = h
     
        # start point
        self.graph = None
        self.graph_points = []
        # [(x0, y0), (xk, yk), ... ]

        self.path = None
        # ls of points in self.graph_points
        
        self.img = None


    def draw_img(self):
        """
        drawing start point (R button)
                end point (R button)
                collision (L button)
        """
        isDrawing = False
        isStart = True
        isTarget = False
        switch = False
        self.graph = self.graph_points = None
        point_size = 5
        # --------------------------------------------------------
        def painter(event, x, y, flags, param):
            nonlocal isDrawing, isStart, isTarget, switch
            if event == cv.EVENT_LBUTTONDOWN:
                isDrawing = True
                cv.circle(img, (x,y), point_size, (0, 0, 0), -1)

            if event == cv.EVENT_MOUSEMOVE:
                if isDrawing == True:
                    cv.circle(img, (x,y), point_size, (0, 0, 0), -1)

            if event == cv.EVENT_LBUTTONUP:
                isDrawing = False
                cv.circle(img, (x,y), point_size, (0, 0, 0), -1)

            if event == cv.EVENT_RBUTTONDOWN:
                if isStart:
                    cv.circle(img, (x,y), 10, (255, 1, 1), -1)
                    self.graph_points = [(x, y)]
                    isStart = False
                    switch = True
                if isTarget:
                    cv.circle(img, (x,y), 10, (1, 1, 255), -1)
                    self.graph_points.append((x, y))
                    isTarget = False
                    switch = False

            if event == cv.EVENT_RBUTTONUP:
                if switch:
                    isTarget = True   

        #-----------------------------------
        img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)
        cv.namedWindow('Draw collisions')
        cv.setMouseCallback('Draw collisions', painter)
        print('Press Esc to stop drawing')

        while(1):
            cv.imshow('Draw collisions', img)
            if cv.waitKey(1) == 27:    # Esc
                if len(self.graph_points) < 2:
                    print('Set target point!')
                else:
                    self.img = img
                    break

        cv.destroyAllWindows()


    def show_img(self, wnd_name, grid=True, graph=True, path=True):
        img = self.img.copy()
        
        # draw grid
        if grid:
            for y in range(self.gs, self.h, self.gs):
                cv.line(img, (0, y), (self.w, y), color=(0, 0, 0))
            for x in range(self.gs, self.w, self.gs):
                cv.line(img, (x, 0), (x, self.h), color=(0, 0, 0))

        if self.graph_points:
            for i, (x, y) in enumerate(self.graph_points):
                if i > 1:
                    # don't draw start/end points
                    cv.circle(img, (x, y), 4, (1, 255, 255), -1)
                cv.putText(img, str(i + 1), (x + 10, y + 5), cv.FONT_HERSHEY_SIMPLEX, 0.7, 2, 1)

        if self.path:
            for i in range(len(self.path) - 1):
                p1 = self.graph_points[self.path[i]]
                p2 = self.graph_points[self.path[i + 1]]

                cv.line(img, p1, p2, (1, 255, 1), 2)

        cv.imshow(wnd_name, img)


    def _find_graph_points(self):
        """
        find the graph points
        of the collisions
        """
        m = self.h // self.gs
        n = self.w // self.gs

        # collision map
        map = np.zeros((m ,n), dtype=np.int8)

        # set collision map 
        for j in range(m - 1):
            for i in range(n - 1):
                x, y = i * self.gs, j * self.gs
                roi = self.img[y : y + self.gs, x : x + self.gs]
                if np.any(roi == 0):
                    map[j, i] = 1
        
        # find collision support points
        for j in range(1, m - 1):
            for i in range(1, n - 1):
                if map[j, i] == 1:
                    def func(j, i, dj, di):
                        if map[j + dj, i] == 0 and map[j, i + di] == 0:
                            i = i+1 if di > 0 else i
                            j = j+1 if dj > 0 else j
                            self.graph_points.append( (i * self.gs, j * self.gs) )

                    func(j, i, -1, +1)
                    func(j, i, -1, -1)
                    func(j, i, +1, +1)
                    func(j, i, +1, -1)

        self._find_distances()


    def _line_cross_collision(self, x0, y0, xk, yk):
        """
        check whether the line (x0, y0) (xk, yk)
        cross the collision in self.img
        :return: bool
        """
        if x0 == xk and y0 == yk:
            return self.img[y0, x0, 0] == 0

        # ax + by + c = 0
        if x0 != xk:
            a_b = (y0 - yk) / (x0 - xk)
            c_b = - a_b * x0 + y0
            
            # y = a_b * x + c_b
            #print('y = {:.2f}*x + {:.2f}'.format(a_b, c_b))

            x0, xk = min(x0, xk), max(x0, xk)
            for x in range(x0, xk + 1):
                y = int(a_b * x + c_b)
                if self.img[y, x, 0] == 0:
                    return True
        else:
            b_a = (x0 - xk) / (y0 - yk)
            c_a = - b_a * y0 + x0

            # x = b_a * y + c_a

            for y in range(y0, yk + 1):
                x = int(b_a * y + c_a)

                if self.img[y, x, 0] == 0:
                    return True

        return False


    def _show_graph(self):
        """
        print matrix of distances to console
        """
        assert self.graph is not None

        n = self.graph.shape[0]
        # title
        print('    ', end='')
        for i in range(n):
            print('{:^3}'.format(i + 1), end=' ')
        print()

        for j in range(n):
            print('{:^2}'.format(j+1), end='| ')
            for i in range(n):
                print('{:^3}'.format(self.graph[j, i]), end=' ')
            print()


    def _find_distances(self):
        """
        на основе списка точек
        составить граф расстояний self.graph
        """
        if not self.graph_points:
            return
        
        n_points = len(self.graph_points)

        self.graph = np.zeros((n_points, n_points), dtype=np.uint16)

        for j in range(0, n_points):
            for i in range(j+1, n_points):
                x0, y0 = self.graph_points[j]
                xk, yk = self.graph_points[i]

                if not self._line_cross_collision(x0, y0, xk, yk):
                    dist = sqrt((x0 - xk)**2 + (y0 - yk)**2)
                    self.graph[j, i] = self.graph[i, j] = int(dist)

        
    def _find_path(self):
        """
        By means of Dijkstra's algorithm
        find the optimal path in self.graph
        """
        if self.graph is None:
            raise Exception('Graph is not defined')
        n_points = self.graph.shape[0]

        # на очереди первая (стартовая) вершина
        queue = [0]

        # расстояния от первой вершины до всех остальных
        dists = 5000 * np.ones(n_points, dtype=np.uint16)
        dists[0] = 0

        # построение пути
        i = 0
        path = []
        while i < len(queue):
                q = queue[i]
                for v in range(n_points):
                    if q != v and self.graph[q, v] and dists[v] > dists[q] + self.graph[q, v]:
                        dists[v] = dists[q] + self.graph[q, v]
                        path.append((q, v))
                        queue.append(v)
                i += 1

        # find the optimal path
        if dists[1] == 5000:
            self.path = None
            print('No path exists')

        d = 1
        self.path = [1]
        for i in range(len(path)):
            p1, p2 = path[-i-1]
            if p2 == d:
                self.path.append(p1)
                if p1 == 0:
                    break
                else:
                    d = p1
        
        self.path.reverse()


    def set_img(self, img, start_point, target_point):
        """
        :param img: 2-D numpy array (h, w, 3)
        :param start_point: (x, y)
        :param end_point: (x, y)
        """
        if len(img.shape) == 2:
            self.h, self.w = img.shape
            self.img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)
            self.img[img == 0] = (0, 0, 0)

        self.graph_points = [start_point, target_point]


    def load_img(self, img_path, start_point, target_point):
        """
        :param img_path: path/to/img
        :param start_point: (x, y)
        :param end_point: (x, y)
        """
        img = cv.imread(img_path, 0)
        np.where(img < 127, 0, 255)

        self.h, self.w = img.shape
        self.img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)
        self.img[img == 0] = (0, 0, 0)

        self.graph_points = [start_point, target_point]


    def get_path(self, array=True):
        """
        get global path
        :return path: 2-D array (Npoints, 2coords) if array
                      else
                      list of (x_i, y_i)
        """
        n = len(self.path)
        if self.path is None or n < 2:
            print('Path is undefined. Use planner')

        path = []
        for j in self.path:
            p = self.graph_points[j]
            path.append(p)
        
        if array:
            array_path = np.empty((n, 2), dtype=np.int16)
            for i, (x, y) in enumerate(path):
                array_path[i][0] = x
                array_path[i][1] = y
            return array_path
        
        return path


    def planner(self, print_graph=False):
        """
        path planner in Visibility Graph
        :param print_graph: whether to print matrix of distances
        """
        if self.img is None:
            raise Exception('No collision map. Use draw_img()')

        if len(self.graph_points) < 2:
            raise Exception('No start/end points. Use draw_img()')

        self._find_graph_points()

        if print_graph:
            self._show_graph()

        self._find_path()


if __name__ == '__main__':
    test = Graph()

    # 1
    start_point = (10, 10)
    target_point = (400, 400)
    test.load_img('map.jpg', start_point, target_point)

    # 2
    test.draw_img()

    # Time measure for algorigthm
    now = datetime.now()
    test.planner()
    print(datetime.now() - now)

    test.show_img('wnd', grid=False)

    cv.waitKey(0)
    cv.destroyAllWindows()