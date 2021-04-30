#!/usr/bin/env python

import cv2 as cv
import numpy as np
from math import pi, sqrt, atan, sin, cos
from datetime import datetime
from planner import Planner
import os

import rospy
from std_msgs.msg import Bool

class Radial(Planner):
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
        self.start_xy = self.target_xy = None

        self.img = self.path = None
        self.local_path = self.map = None

        # list of (x, y)
        self.collisions = None


    def _set_map(self):
        """
        basing on obstacles self.img
        build polar map (x=fi, y=r)
        """
        h, w, _ = self.img.shape

        self.target_ji = self._xy2polar(*self.target_xy)

        self.map = 500*np.ones(((w+h) // self.delta_r, 360 // self.delta_fi), dtype=np.int16)   # in polar 

        for (x, y) in zip(self.collisions[0], self.collisions[1]):
            j, i = self._xy2polar(x, y)
            self.map[j, i] = -1
            
        # initialize coordinates (string)
        self.map[0, np.where(self.map[0, :] == 500)] = 1


    def show_map(self):
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

            if self.map[j_target, i_target] < 500:
                break

            def move(dist):
                dj, di = dist
                if self.map[dj, di] > -1 and cell + 1 < self.map[dj, di]:
                    self.map[dj, di] = cell + 1
                    query.append((dj, di))

            # down
            if j+1 < h:
               move((j+1, i))
            # up
            if j > 0:
               move((j-1, i))
            # right
            if i+1 < w:
                move((j, i+1)) 
            else: 
                move((j, 0))   # 360 grad = 0 grad 
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

        self.local_path = [self._polar2xy(j, i)]  # counting from 1

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
     
            self.local_path.append(self._polar2xy(j, i))  # counting from 1
    
        if rotat_point is None:
            self.path.append(self.target_xy)
        else:
            x, y = self._polar2xy(*rotat_point)
            self.path.append((x, y))

        return True


    def _xy2polar(self, x, y):
        """
        counterclockwise, starting from (x>0, y=0)
        fi = [0..2*pi)
        return (j, i) - cell in polar self.map
        """
        x0, y0 = self.start_xy
        x -= x0
        y -= y0
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


    def _polar2xy(self, j, i):
        """
        :param j: r axis in self.map
        :param i: fi axis in self.map
        :return: (x, y) pixels - tuple
        """

        r = j * self.delta_r + self.delta_r / 2
        fi = i * self.delta_fi + self.delta_fi / 2
        
        fi = fi / 180 * pi # radian to degree

        x = r*cos(fi) + self.start_xy[0]
        y = self.start_xy[1] - r*sin(fi)
        
        return int(x), int(y)


    def _plot_grid(self, img):
        r_max = 2000
        for fi in range(0, 360, self.delta_fi):
            fi = fi / 180 * pi  # to rad

            x = int(r_max*cos(fi)) + self.start_xy[0]
            y = self.start_xy[1] - int(r_max*sin(fi))

            cv.line(img,  self.start_xy, (x, y), color=(0, 0, 0))

        for r in range(self.delta_r, r_max, self.delta_r):
            cv.circle(img, self.start_xy, r, (0, 0, 0))
        
        return img


    def _get_collision_points(self):

        h, w, _ = self.img.shape
        
        y, x = np.where(self.img[:, :, 0] == 0)

        self.collisions = (x, y)
        

    def _get_contours_only(self):
        """ remain only contours of collisions """

        img = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)

        _, thresh = cv.threshold(img, 1, 255, cv.THRESH_BINARY)

        c = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

        np_countours = np.array(c)

        img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)

        cv.drawContours(image=img, contours=np_countours, contourIdx=-1, color=(0, 0, 0), thickness=1)

        self.img = img


    def planner(self, debug=False):
        """
        :return: ls of [(x, y)] - path or None
        """
        check_message = self._check_input_data()
        if check_message != 'OK':
            raise Exception(check_message)

        self._get_contours_only()

        #now = datetime.now()
        self._get_collision_points()

        #return

        self._set_map()

        #print('init', datetime.now() - now)

        self.path = [self.start_xy]

        i = 1
        while self.path[-1] != self.target_xy:
            #now = datetime.now()
            self._set_map()
            #print('set map', datetime.now() - now)
            #now = datetime.now()
            self._lee()
            #print('lee', datetime.now() - now)
            path_exists = self._find_path()

            if debug:
                img_name = 'img{}'.format(i)
                self.show_img(title='img{}'.format(i), grid=True)

            if not path_exists:
                print('Path does not exists')
                self.path = None
                return None

            self.start_xy = self.path[-1]

            i += 1
    
def callback(data):
    test = Radial(delta_r=10, delta_fi=5)
    start_point = (150, 150)
    target_point = (400, 100)
    img = cv.imread('test_plot.jpg', 0)
    crop_img = img[67:530, 108:706]
    cv.imwrite('test_plot_cropped.jpg', crop_img)
    test.load_img('test_plot_cropped.jpg', start_point, target_point)
    rospy.loginfo("Image is ready getting, searching for path...")
    # Time measure for algorigthm
    now = datetime.now()
    test.planner(debug=False)
    # print(datetime.now() - now)
    test.show_img('final', grid=True, path=True)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    #ros initialization
    rospy.init_node('planner_node', anonymous=True)
    rospy.Subscriber("space_ready_topic", Bool, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()