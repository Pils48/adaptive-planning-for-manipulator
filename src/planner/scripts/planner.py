#!/usr/bin/env python

import numpy as np
import os
import cv2 as cv
from math import sqrt

class Planner():

    def __init__(self):
        pass
    
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
                    self.start_xy = (x, y)
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
                if not (self.start_xy and self.target_xy):
                    print('Set start/target point!')
                else:
                    self.img = img
                    break

        cv.destroyAllWindows()


    def load_img(self, img_path, start_point, target_point):
        """
        :param img_path: path/to/img
        :param start_point: (x, y)
        :param end_point: (x, y)
        """
        img = cv.imread(img_path, 0)
        img = np.where(img < 127, 0, 255)

        self.h, self.w = img.shape
        self.img = 255*np.ones((self.h, self.w, 3), dtype=np.uint8)
        self.img[img == 0] = (0, 0, 0)

        self.start_xy = start_point
        assert img[start_point[1], start_point[0]] == 255, \
            'Start point must be on non-collision point'

        self.target_xy = target_point
        assert img[target_point[1], target_point[0]] == 255, \
            'Target point must be on non-collision point'


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

        self.start_xy = start_point
        assert img[start_point[1], start_point[0]] == (255, 255, 255), \
            'Start point must be on non-collision point'

        self.target_xy = target_point
        assert img[target_point[1], target_point[0]] == (255, 255, 255), \
            'Target point must be on non-collision point'


    def show_img(self, title='img', grid=True, path=True, save_name=None):
        """
        :param grid: whether to plot grid
        :param path: whether to plot path between start-target
        :param save_name: path to save img. If None, without saving
        """
        img = self.img.copy()
        
        # draw grid
        if grid:
            img = self._plot_grid(img)

        # draw path
        if path and self.path:
            for i in range(len(self.path) - 1):
                p1 = self.path[i]
                p2 = self.path[i + 1]

                cv.line(img, p1, p2, (1, 255, 1), 2)

        # plot start and target points
        cv.circle(img, self.start_xy, 10, (255, 1, 1), -1)
        cv.circle(img, self.target_xy, 10, (1, 1, 255), -1)

        cv.imshow(title, img)

        if save_name is not None:
            cv.imwrite(save_name, img)


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
        
        if array:
            array_path = np.empty((n, 2), dtype=np.int16)
            for i, (x, y) in enumerate(path):
                array_path[i][0] = x
                array_path[i][1] = y
            return array_path
        
        return self.path


    def get_len_path(self):
        """
        :return: length of self.path (in px)
                    or None if it doesn't exists
        """
        if not self.path:
            return None

        S = 0
        for i in range(len(self.path) - 1):
            (x0, y0) = self.path[i]
            (x, y) = self.path[i + 1]

            S += sqrt( (x - x0)**2 + (y - y0)**2 )
        
        return S


    def _check_input_data(self):
        """
        check self.img, self.start, self.target exist
        and
        start/target point don't belong collision
        :return: 'OK' or Error message (string)
        """
        if self.img is None:
            return 'Map image is not defined'
        
        if not (self.start_xy and self.target_xy):
            return 'Start/target points are not defined'

        x, y = self.start_xy
        if np.any(self.img[y, x] == 0):
            return 'Start point is on the collision'

        x, y = self.target_xy
        if np.any(self.img[y, x] == 0):
            return 'Target point is on the collision'

        return 'OK'

