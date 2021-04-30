#!/usr/bin/env python

import cv2 as cv
import numpy as np
from math import pi, sqrt, atan, sin, cos
from datetime import datetime

import rospy
from std_msgs.msg import Bool

class Radial():
    def __init__(self, w=800, h=600, delta_r=30, delta_fi=10):
        """
        :param w: window width - pixels
        :param h: window height - pixels
        :param delta_r: radius step - pixels
        :param delta_fi: angle step - degrees
        """
        self.delta_r = delta_r
        self.delta_fi = delta_fi
        self.w = w
        self.h = h
     
        # start point
        self.x0 = self.y0 = 0

        self.img = self.target_xy = self.global_path = \
                    self.local_path = self.map = None


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
        point_size = 5
        # --------------------------------------------------------
        def painter(event, x, y, flags, param):
            # nonlocal isDrawing, isStart, isTarget, switch
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
                    self.x0, self.y0 = x, y
                    self.global_path = [(x, y)]
                    isStart = False
                    switch = True
                if isTarget:
                    cv.circle(img, (x,y), 10, (1, 1, 255), -1)
                    self.target_xy = (x, y)
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
                if not self.target_xy:
                    print('Set target point!')
                else:
                    self.img = img
                    break

        cv.destroyAllWindows()

    def _set_map(self):
        """
        basing on obstacles images self.img
        build polar map (x=fi, y=r)
        """
        if self.img is None:
            raise Exception('No img exists')

        h, w, _ = self.img.shape

        self.target_ji = self._xy2polar(*self.target_xy)

        self.map = 500*np.ones((w // self.delta_r, 360 // self.delta_fi), dtype=np.int16)   # in polar 

        d = self.delta_r // 2
        for x in range(d//2, w - d//2 + 1, d):
            for y in range(d//2, w - d//2 + 1, d):
                if np.any(self.img[y-d//2 : y+d//2, x-d//2:x+d//2, 0] == 0):  # collision
                    j, i = self._xy2polar(x, y)
                    self.map[j, i] = -1
        
        # remeber initial coordinates
        # print(self.map[0, 0])
        self.map[0, np.where(self.map[0, :] == 500)] = 1


    def show_img(self, wnd_name, grid=False, local_path=False, global_path=True):
        """
        :param wnd_name: window title
        :param grid: whether to plot polar grid
        :param local_path: whether to plot local path (yellow points)
        :param global_path: whether to plot global path (green lines)
        """
        img = self.img.copy()
        
        if grid:
            img = self._draw_polar_grid(img)

        # draw local path (yellow points)
        if self.local_path and local_path:
            for x, y in self.local_path:
                cv.circle(img, (x, y), 4, (1, 255, 255), -1)     # yellow points

        # draw global path (green points)
        if self.global_path and global_path:
            for i, (x, y) in enumerate(self.global_path[:-1]):
                xk, yk = self.global_path[i+1]
                cv.line(img,  (x, y), (xk, yk), (0, 255, 0), 3)

        cv.imshow(wnd_name, img)
        cv.waitKey(0)


    def _show_map(self):
        print('d_r = {} px, d_fi = {} grad'.format(self.delta_r, self.delta_fi))
        grid = 40
        h, w = self.map.shape
        img = 255 * np.ones((h*grid, w*grid, 3), dtype=np.uint8)
        for x in range(w):
            for y in range(h):
                color = (255, 0, 0) if (y, x) == self.target_ji else (0, 0, 0)
                cv.rectangle(img, (x*grid, y*grid), ((x+1)*grid, (y+1)*grid), color, 1)
                cv.putText(img, str(self.map[y, x]), (x*grid+15, y*grid+15), cv.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)

        print('Press ESC to close')
        # show
        while 1:
            cv.imshow('map', img)
            if cv.waitKey(1) == 27:    # Esc
                    break


    def _lee(self):
        """
        Lee algorithm implementation for self.map
        """
        j_target, i_target = self.target_ji
        
        h, w = self.map.shape

        query = np.where(self.map[0, :] > -1)[0].tolist()
        query = [(0, q) for q in query]

        for (j, i) in query:
            cell = self.map[j, i]
            if cell != -1:

                def move(dist):
                    dj, di = dist
                    if self.map[dj, di] > -1 and cell + 1 < self.map[dj, di]:
                        self.map[dj, di] = cell + 1
                        query.append((dj, di))
                        return True
                    return False

                # down
                if j+1 < h: move((j+1, i))
                # up
                if j > 0: move((j-1, i))
                # right
                move((j, i+1)) if i+1 < w else move((j, 0))   # 360 grad = 0 grad 
                # left
                move((j, i-1))


    def _find_path(self):
        """
        by means filled self.map
        find the optimal path
        """
        j, i = self.target_ji
        if self.map[j, i] == 500 or self.map[j, i] == -1:
            return False

        self.local_path = [self.polar2xy(j, i)]  # more comfortable to begin with 1

        h, w = self.map.shape

        rotat_point = None
        while j > 0:
            current = self.map[j, i]
            # up
            if self.map[j-1, i] == current - 1:
                j -= 1
            # left
            elif self.map[j, i-1] == current - 1:
                i -= 1
                rotat_point = (j, i)
            # down
            elif j+1 < h and self.map[j+1, i] == current - 1:
                j += 1
            else:
                i = i + 1 if i+1 < w else 0
                rotat_point = (j, i)
     
            self.local_path.append(self.polar2xy(j, i))  # more comfortable to begin with 1
    
        if rotat_point is None:
            self.global_path.append(self.target_xy)
        else:
            x, y = self.polar2xy(*rotat_point)
            self.global_path.append((x, y))

        return True


    def _xy2polar(self, x, y):
        """
        counterclockwise, beginning with (x>0, y=0)
        fi = [0..2*pi)
        return (j, i) - cell in polar self.map
        """
        x -= self.x0
        y -= self.y0
        r = sqrt(x*x + y*y)
        if x == 0 and y < 0:
            fi = -pi/2
        elif x == 0 and y > 0:
            fi = pi/2
        else:
            fi = atan(y / x)

        if x >= 0 and fi <= 0:
            fi = -fi
        elif x >= 0 and fi > 0:
            fi = 2*pi - fi
        elif x < 0:
            fi = pi - fi   

        fi = fi / pi * 180  # from rad to degree

        return int(r / self.delta_r), int(fi / self.delta_fi)

    def polar2xy(self, j, i):
        """
        :param j: r axis in self.map
        :param i: fi axis in self.map
        :return: (x, y) pixels - tuple
        """

        r = j * self.delta_r + self.delta_r / 2
        fi = i * self.delta_fi + self.delta_fi / 2
        
        fi = fi / 180 * pi # radian to degree

        x = r*cos(fi) + self.x0
        y = self.y0 - r*sin(fi)
        
        return int(x), int(y)


    def _draw_polar_grid(self, img):
        r_max = 600
        for fi in range(0, 360, self.delta_fi):
            fi = fi / 180 * pi  # to rad

            x = int(r_max*cos(fi)) + self.x0
            y = self.y0 - int(r_max*sin(fi))

            cv.line(img,  (self.x0, self.y0), (x, y), color=(0, 0, 0))

        for r in range(self.delta_r, r_max, self.delta_r):
            cv.circle(img, (self.x0, self.y0), r, (0, 0, 0))
        
        return img


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

        self.global_path = [start_point]
        self.x0, self.y0 = start_point
        self.target_xy = target_point


    def load_img(self, img_path, start_point, target_point):
        """
        :param img_path: path/to/img
        :param start_point: (x, y)
        :param end_point: (x, y)
        """
        img = cv.imread(img_path, 0)
        np.where(img < 127, 0, 255)
        # Debug section
        crop_img = img[67:530, 108:706]
        self.h, self.w = crop_img.shape
        #Debug
        self.img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)
        self.img[crop_img == 0] = (0, 0, 0)

        self.global_path = [start_point]
        self.x0, self.y0 = start_point
        self.target_xy = target_point


    def get_path(self, array=True):
        """
        get global path
        :return path: 2-D array (Npoints, 2coords) if array
                      else
                      list of (x_i, y_i)
        """
        if self.global_path is None or len(self.global_path) < 2:
            print('Path is undefined. Use planner')

        n = len(self.global_path)
        path = np.empty((n, 2), dtype=np.int16)
        if array:
            for i, (x, y) in enumerate(self.global_path):
                path[i][0] = x
                path[i][1] = y
            return path
        
        return self.global_path


    def planner(self, debug=False):
        """
        :return: ls of [(x, y)] - path or None
        """
        i = 1
        while self.global_path[-1] != self.target_xy:
            self._set_map()
            self._lee()
            path_exists = self._find_path()
            if not path_exists:
                print('Path does not exists')
                self.global_path = None
                return None

            if debug:
                img_name = 'img{}'.format(i)
                self.show_img(img_name, grid=True, local_path=True, global_path=False)

            self.x0, self.y0 = self.global_path[-1]

            i += 1


# # 1


# # 2
# # test.draw_img()




# cv.waitKey(0)
# cv.destroyAllWindows()
def callback(data):
    test = Radial()
    start_point = (150, 150)
    target_point = (200, 200)
    test.load_img('test_plot.png', start_point, target_point) #error hadling???
    rospy.loginfo("Image is ready getting, searching for path...")
    # Time measure for algorigthm
    now = datetime.now()
    test.planner(debug=False)
    # print(datetime.now() - now)
    test.show_img('final', grid=True, local_path=True, global_path=True)


if __name__ == '__main__':
    rospy.init_node('planner_node', anonymous=True)
    rospy.Subscriber("space_ready_topic", Bool, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()