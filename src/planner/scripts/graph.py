from __future__ import print_function
import cv2 as cv
import numpy as np
from datetime import datetime
from planner import Planner
from math import sqrt

class Node:
    def __init__(self, id):
        self.id = id        # number of graph point
        self.prev = None    # previous node
        self.h = None       # empirical score
        self.d = 5000       # dist from start
        self.f = None       # d + h


class Graph(Planner):
    def __init__(self, w=800, h=600, grid_size=20, path_finder='Astar'):
        """
        :param w: window width - pixels
        :param h: window height - pixels
        :param delta_r: radius step - pixels
        :param delta_fi: angle step - degrees
        :param path_finder: 'Astar' (default) or 'Dijkstra'
        """
        self.gs = grid_size
        self.w = w
        self.h = h
     
        # array M*M - graph of distances
        self.graph = None
        
        self.graph_points = []
        # [(x0, y0), (xk, yk), ... ]

        self.path = None
        # ls of (x, y)

        self.start_xy = self.target_xy = None
        
        self.img = None

        if path_finder == 'Astar':
            self.find_path = self._Astar_finder
        elif path_finder == 'Dijkstra':
            self.find_path = self._Dijkstra_finder
        else:
            raise Exception('Invalid path finder: {}'.format(path_finder))


    def _plot_grid(self, img):
        # grid
        for y in range(self.gs, self.h, self.gs):
            cv.line(img, (0, y), (self.w, y), color=(0, 0, 0))
        for x in range(self.gs, self.w, self.gs):
            cv.line(img, (x, 0), (x, self.h), color=(0, 0, 0))

        # graph points
        for i, (x, y) in enumerate(self.graph_points):
            if i > 1:
                # don't draw start/end points
                cv.circle(img, (x, y), 4, (1, 255, 255), -1)
            cv.putText(img, str(i + 1), (x + 10, y + 5), cv.FONT_HERSHEY_SIMPLEX, 0.7, 2, 1)

        return img


    def _find_graph_points(self):
        """
        find the graph points
        of the collisions
        """
        m = self.h // self.gs
        n = self.w // self.gs

        # collision map
        map = np.zeros((m ,n), dtype=np.int8)

        # initial points of the graph
        self.graph_points = [self.start_xy, self.target_xy]

        # set collision map 
        for j in range(m - 1):
            for i in range(n - 1):
                x0, y0 = i * self.gs, j * self.gs
                x, y = x0 + self.gs, y0 + self.gs
                roi = self.img[y0 : y , x0 : x]
                
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

                            x, y = i * self.gs, j * self.gs
                            if (x, y) != self.start_xy and (x, y) != self.target_xy:
                                self.graph_points.append( (x, y) )

                    func(j, i, -1, +1)
                    func(j, i, -1, -1)
                    func(j, i, +1, +1)
                    func(j, i, +1, -1)

        # compute graph of distances between graph_points
        self._find_distances()


    def _find_distances(self):
        """
        based on graph points
        bulid distance graph - self.graph
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
            a_b = 1.0*(y0 - yk) / (x0 - xk)
            c_b = - a_b * x0 + y0
            
            # y = a_b * x + c_b

            x0, xk = min(x0, xk), max(x0, xk)
            for x in range(x0, xk + 1):
                y = int(a_b * x + c_b)
                if self.img[y, x, 0] == 0:
                    return True
        else:
            b_a = 1.0*(x0 - xk) / (y0 - yk)
            c_a = - b_a * y0 + x0

            # x = b_a * y + c_a

            y0, yk = min(y0, yk), max(y0, yk)
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

        
    def _Dijkstra_finder(self):
        """
        By means of Dijkstra's algorithm
        find the optimal path in self.graph
        """
        if self.graph is None:
            raise Exception('Graph is not defined')
        n_points = self.graph.shape[0]

        # the first point on the queue - start point
        queue = [0]

        # dist btwn start point and others
        dists = 5000 * np.ones(n_points, dtype=np.uint16)
        dists[0] = 0

        # building path
        i = 0
        path = []
        while i < len(queue) and dists[1] == 5000:
            q = queue[i]
            for v in range(n_points):
                # 1) if q and v are not same points
                # 2) path btwn q and v exists
                # 3) path btwn q and v is optimal
                if q != v and self.graph[q, v] and dists[v] > dists[q] + self.graph[q, v]:
                    dists[v] = dists[q] + self.graph[q, v]
                    path.append((q, v))
                    queue.append(v)
            
            i += 1

        # find the optimal path
        if dists[1] == 5000:
            return []
            print('No path exists')

        d = 1
        self.path = [self.target_xy]
        for i in range(len(path)):
            p1, p2 = path[-i-1]
            if p2 == d:
                self.path.append(self.graph_points[p1])
                if p1 == 0:
                    break
                else:
                    d = p1
        
        self.path.reverse()


    def _Astar_finder(self):
        """
        """
        if self.graph is None:
            raise Exception('Graph is not defined')
        n_points = self.graph.shape[0]

        # create list of Nodes for each graph points
        nodes = [Node(i) for i in range(n_points)]
        # dist from start to start
        nodes[0].d = 0

        # empirical function - dist to target point
        (xk, yk) = self.graph_points[1]
        for i in range(1, n_points): # except start point
            (x, y) = self.graph_points[i]
            d = sqrt((xk - x)**2 + (yk - y)**2)
            nodes[i].h = d

        # starting from start point
        queue = [nodes[0]]

        # nodes that were visited
        closed_nodes = []

        while queue and queue[0].id != 1:
            node_start = queue[0]

            for node in nodes:
                d = self.graph[node_start.id, node.id]
                if node not in closed_nodes and d > 0 and node.d > node_start.d + d:

                    # dist from the start to this node
                    node.d = node_start.d + d
                    # key of priority
                    node.f = node.d + node.h
                    # save prev node
                    node.prev = node_start

                    queue.append(node)

            # sort by node.f
            queue = queue[1: ]
            if queue:
                queue.sort(key=lambda x: x.f)

            closed_nodes.append(node_start)

        if not queue:
            print('No path exists')
            return

        node = queue[0]
        path = []
        while node is not None:
            path.append(node.id)
            node = node.prev

        path.reverse()

        self.path = []
        for p in path:
            (x, y) = self.graph_points[p]
            self.path.append((x, y))


    def planner(self, print_graph=False):
        """
        path planner in Visibility Graph
        :param print_graph: whether to print matrix of distances
        """
        check_message = self._check_input_data()
        if check_message != 'OK':
            raise Exception(check_message)

        if not (self.start_xy and self.target_xy):
            raise Exception('No start/end points. Use draw_img()')

        # 1
        self._find_graph_points()

        if print_graph:
            # for debug
            self._show_graph()

        # 2
        self.find_path()


if __name__ == '__main__':
    test = Graph(grid_size=50)

    # 1
    #test.draw_img()

    test.load_img('maps/0_(60,140)_(660,400).jpg', (60,140), (660,400))
    
    # Time measure for algorigthm
    now = datetime.now()
    test.planner(print_graph=False)
    print(datetime.now() - now)

    test.show_img(grid=True, path=True)

    print('S =', int(test.get_len_path()))
    cv.waitKey(0)
    cv.destroyAllWindows()
