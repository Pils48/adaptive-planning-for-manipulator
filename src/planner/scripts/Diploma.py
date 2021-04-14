import numpy as np
import cv2 as cv
from datetime import datetime

radius = 10
min_discrete = 10
max_discrete = min_discrete * (2**4)

MAX_step = 500  # no more than 500 steps

# markers on the map
clean_mark = 0
collsn_mark = -1
start_mark = 1
target_mark = MAX_step

class PathPlanner:
    def __init__(self, m, n):
        self.img_shape = (m, n)
        self.map_shape = (m // min_discrete, n // min_discrete)
        self.img = 255*np.ones((m, n, 3), np.uint8)                 # 3 channels
        self.map = clean_mark*np.ones(self.map_shape, np.int16)
        self.start = ()
        self.target = ()
        self.path = []
 

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
                cv.circle(img, (x,y), radius, (0, 0, 0), -1)

            if event == cv.EVENT_MOUSEMOVE:
                if isDrawing == True:
                    cv.circle(img, (x,y), radius, (0, 0, 0), -1)

            if event == cv.EVENT_LBUTTONUP:
                isDrawing = False
                cv.circle(img, (x,y), radius, (0, 0, 0), -1)

            if event == cv.EVENT_RBUTTONDOWN:
                if isStart:
                    cv.circle(img, (x,y), 2, (255, 1, 1), -1)
                    self.map[y//min_discrete, x//min_discrete] = start_mark
                    self.start = (y//min_discrete, x//min_discrete)
                    isStart = False
                    switch = True
                if isTarget:
                    cv.circle(img, (x,y), 2, (1, 1, 255), -1)
                    self.map[y//min_discrete, x//min_discrete] = target_mark
                    self.target = (y//min_discrete, x//min_discrete)
                    isTarget = False
                    switch = False

            if event == cv.EVENT_RBUTTONUP:
                if switch:
                    isTarget = True   
        #-----------------------------------

        img = self.img.copy()
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
        m, n = self.map_shape
        # ---- edges == collisions -----
        self.map[0:m, 0] = collsn_mark
        self.map[0:m, n-1] = collsn_mark
        self.map[0, 0:n] = collsn_mark
        self.map[m-1, 0:n] = collsn_mark
        # ------------------------------
        for j in range(m):
            for i in range(n):
                x0, y0 = i*min_discrete, j*min_discrete
                xk, yk = x0 + min_discrete, y0 + min_discrete
                roi = self.img[y0:yk, x0:xk, :]
                if np.any(roi == 0) and self.map[j, i] == 0:
                    # set collision mark
                    self.map[j, i] = collsn_mark


    def show_map(self):
        """
        scale double up
        """
        img = 255*np.ones((self.img_shape[0]*2, self.img_shape[1]*2), dtype=np.uint8)
        m, n = self.map_shape
        for j in range(m):
            for i in range(n):
                x0, y0 = i * min_discrete*2, j * min_discrete*2
                xk, yk = x0 + min_discrete*2, y0 + min_discrete*2

                if (j,i) in self.path:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 2)
                else:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 1)

                text = str(self.map[j, i])[:3]
                cv.putText(img, text, (x0+1, (y0+yk)//2), cv.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)
        cv.imshow('map', img)
    

    def show_img(self):
        img = self._separate_static()
        cv.imshow('img', img)
        

    def _separate_static(self):
        """
        separate into the same discrets
        """
        m, n = self.map_shape
        new_img = self.img.copy()
        for j in range(m):
            for i in range(n):
                x0, y0 = i * min_discrete, j * min_discrete
                xk, yk = x0 + min_discrete, y0 + min_discrete
                if (j, i) == self.start:
                    # start (blue)
                    cv.rectangle(new_img, (x0, y0), (xk, yk), (255, 0, 0), -1)
                elif (j, i) == self.target:
                    # finish (red)
                    cv.rectangle(new_img, (x0, y0), (xk, yk), (0, 0, 255), -1)
                elif (j, i) in self.path:
                    # path (green)
                    cv.rectangle(new_img, (x0, y0), (xk, yk), (0, 255, 0), -1)
                elif self.map[j, i] >= 0:  
                    cv.rectangle(new_img, (x0, y0), (xk, yk), (40, 40, 40), 1)

        return new_img


    def static_LEEalgorithm(self):
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
            self._get_path()
        else:
            print('No path exists')


    def _get_path(self):
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


test = PathPlanner(480, 640)
test.draw_map()
now = datetime.now()
test.static_LEEalgorithm()
print('Time:', datetime.now() - now)
test.show_img()
test.show_map()
cv.waitKey(0)
cv.destroyAllWindows()

