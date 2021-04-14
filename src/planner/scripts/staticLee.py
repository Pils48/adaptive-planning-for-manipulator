import numpy as np
import cv2 as cv
from datetime import datetime

MAX_step = 500  # no more than 500 steps

# markers on the map
collsn_mark = -1
start_mark = 1
target_mark = MAX_step

class StaticLee:
    def __init__(self, h, w, d=10):
        self.img_shape = (h, w)
        
        self.start = self.target = ()

        self.path = []
        # list of sequence (j, i)
        
        self.d = d
 

    def draw_map(self):
        """
        drawing start point (R button)
                end point (R button)
                collision (L button)
        """
        isDrawing = False
        isStart = True
        isTarget = False
        switch = False
        # --------------------------------------------------------
        def painter(event, x, y, flags, param):
            nonlocal isDrawing, isStart, isTarget, switch
            if event == cv.EVENT_LBUTTONDOWN:
                isDrawing = True
                cv.circle(img, (x,y), 10, (0, 0, 0), -1)

            if event == cv.EVENT_MOUSEMOVE:
                if isDrawing == True:
                    cv.circle(img, (x,y), 10, (0, 0, 0), -1)

            if event == cv.EVENT_LBUTTONUP:
                isDrawing = False
                cv.circle(img, (x,y), 10, (0, 0, 0), -1)

            if event == cv.EVENT_RBUTTONDOWN:
                if isStart:
                    cv.circle(img, (x,y), 2, (255, 1, 1), -1)
                    self.start = (y//self.d, x//self.d)
                    isStart = False
                    switch = True
                if isTarget:
                    cv.circle(img, (x,y), 2, (1, 1, 255), -1)
                    self.target = (y//self.d, x//self.d)
                    isTarget = False
                    switch = False

            if event == cv.EVENT_RBUTTONUP:
                if switch:
                    isTarget = True   
        #-----------------------------------
        h, w = self.img_shape
        img = 255*np.ones((h, w, 3), np.uint8)                 # 3 channels

        cv.namedWindow('Draw collisions')
        cv.setMouseCallback('Draw collisions', painter)
        print('Press Esc to stop drawing')

        while(1):
            cv.imshow('Draw collisions', img)
            if cv.waitKey(1) == 27:    # Esc
                self.img = img
                self._set_map()
                break

        cv.destroyAllWindows()


    def _set_map(self):
        h, w, _ = self.img.shape
        m, n = h // self.d, w // self.d
        self.map = np.zeros((m, n), dtype=np.int16)
        self.map_shape = m, n

        # ---- edges == collisions -----
        self.map[0:m, 0] = collsn_mark
        self.map[0:m, n-1] = collsn_mark
        self.map[0, 0:n] = collsn_mark
        self.map[m-1, 0:n] = collsn_mark
        # ------------------------------
        for j in range(m):
            for i in range(n):
                x0, y0 = i*self.d, j*self.d
                xk, yk = x0 + self.d, y0 + self.d
                roi = self.img[y0:yk, x0:xk, :]
                if np.any(roi == 0) and self.map[j, i] == 0:
                    # set collision mark
                    self.map[j, i] = collsn_mark

        j, i = self.start
        self.map[j, i] = start_mark

        j, i = self.target
        self.map[j, i] = target_mark


    def show_map(self):
        """
        scale double up
        """
        img = 255*np.ones((self.img_shape[0]*2, self.img_shape[1]*2), dtype=np.uint8)
        m, n = self.map_shape
        for j in range(m):
            for i in range(n):
                x0, y0 = i * self.d*2, j * self.d*2
                xk, yk = x0 + self.d*2, y0 + self.d*2

                if (j,i) in self.path:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 2)
                else:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 1)

                text = str(self.map[j, i])[:3]
                cv.putText(img, text, (x0+1, (y0+yk)//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)
        cv.imshow('map', img)
    

    def show_img(self, grid=False, path=True):
        if grid:
            img = self._plot_grid(path=path)
            cv.imshow('img', img)
        else:
            cv.imshow('img', self.img)
        

    def _plot_grid(self, path=True):
        """
        separate into the same discrets
        """
        m, n = self.map_shape
        img = self.img.copy()
        for j in range(m):
            for i in range(n):
                x0, y0 = i * self.d, j * self.d
                xk, yk = x0 + self.d, y0 + self.d
                if (j, i) == self.start and path:
                    # start (blue)
                    cv.rectangle(img, (x0, y0), (xk, yk), (255, 0, 0), -1)
                elif (j, i) == self.target and path:
                    # finish (red)
                    cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 255), -1)
                elif (j, i) in self.path and path:
                    # path (green)
                    cv.rectangle(img, (x0, y0), (xk, yk), (0, 255, 0), -1)
                elif self.map[j, i] >= 0:  
                    cv.rectangle(img, (x0, y0), (xk, yk), (40, 40, 40), 1)

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

        (x, y) = start_point
        self.map[y//self.d, x//self.d] = start_mark
        self.start = (y//self.d, x//self.d)

        (x, y) = target_point
        self.map[y//self.d, x//self.d] = target_point
        self.target = (y//self.d, x//self.d)

        self._set_map()


    def load_img(self, img_path, start_point, target_point):
        """
        :param img_path: path/to/img
        :param start_point: (x, y)
        :param end_point: (x, y)
        """
        img = cv.imread(img_path, 0)
        np.where(img < 127, 0, 255)

        self.img_shape = img.shape

        h, w = img.shape
        self.img = 255*np.ones((h, w, 3), dtype=np.uint8)
        self.img[img == 0] = (0, 0, 0)

        (x, y) = start_point
        self.start = (y//self.d, x//self.d)

        (x, y) = target_point
        self.target = (y//self.d, x//self.d)

        self._set_map()


    def get_path(self, array=True):
        """
        get path
        :return path: 2-D array (Npoints, 2coords) if array
                      else
                      list of (x_i, y_i)
        """
        n = len(self.path)
        if n < 2:
            print('Path is undefined. Use planner')

        path = []
        for (j, i) in self.path:
            (x, y) = (i * self.d + self.d//2, j * self.d + self.d//2)
            path.append((x,y))
        
        if array:
            array_path = np.empty((n, 2), dtype=np.int16)
            for i, (x, y) in enumerate(path):
                array_path[i][0] = x
                array_path[i][1] = y
            return array_path
        
        return path


    def planner(self):
        print('left - up - right - down')
        j_tar, i_tar = np.where(self.map == target_mark)

        step = 1
        # step =   0   1
        queue = [ [] for _ in range(MAX_step+1)]
        queue[step].append(np.where(self.map == start_mark))

        while self.map[j_tar, i_tar] == target_mark:
            coords = queue[step]
            for j, i in coords:
                
                def new_step(j0, i0,  jk, ik, step):
                    point = self.map[jk, ik]
                    if point >= 0:
                        if point == 0 or step + 1 < point:
                            self.map[jk, ik] = step + 1
                            queue[step+1].append((jk, ik))
                # up
                new_step(j, i, j, i-1, step)
                # down
                new_step(j, i, j, i+1, step)
                # left
                new_step(j, i, j-1, i, step)
                # right
                new_step(j, i, j+1, i, step)
            
            step += 1

            if step > MAX_step:
                break

        if step <= MAX_step:
            print('Steps:', step)
            self._find_path()
        else:
            print('No path exists')


    def _find_path(self):
        j, i = self.target
        self.path = [(j, i)]   # list of (j, i)

        step = self.map[j, i]
        while not ((j, i) == self.start):
            # left
            if self.map[j, i+1] == step - 1:
                i += 1
            # right
            elif self.map[j, i-1] == step - 1:
                i -= 1
            # up
            elif self.map[j+1, i] == step - 1:
                j += 1
            # down
            else:
                j -= 1

            self.path.append((j, i))
            step -= 1

        self.path.reverse()


test = StaticLee(320, 400)

# 1
test.load_img('map.jpg', (10, 10), (400, 400))

# 2
test.draw_img()

now = datetime.now()
test.planner()
print('Time:', datetime.now() - now)

test.show_img(grid=True, path=True)
test.show_map()
cv.waitKey(0)
cv.destroyAllWindows()

