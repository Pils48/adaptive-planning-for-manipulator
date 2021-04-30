import numpy as np
import cv2 as cv
from datetime import datetime
import os
from planner import Planner

MAX_step = 500  # no more than 500 steps

# markers on the map
collsn_mark = -1
start_mark = 1
target_mark = MAX_step

class Lee(Planner):
    def __init__(self, h, w, discrete=10):    
        self.map = self.img = None
        self.start_xy =  self.target_xy = None
       
        self.w = w
        self.h = h

        self.path = self.path_ji = None
 
        self.d = discrete


    def _set_map(self):
        h, w, _ = self.img.shape
        m, n = (h // self.d, w // self.d)
        self.map = np.zeros((m, n), np.int16)

        # set start cell in the map
        x, y = self.start_xy
        self.map[y//self.d, x//self.d] = start_mark
        self.start = (y//self.d, x//self.d)
        
        # set target cell in the map
        x, y = self.target_xy
        self.target = (y//self.d, x//self.d)
        self.map[y//self.d, x//self.d] = target_mark

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


    def show_map(self):
        """
        scale double up
        """
        h, w, _ = self.img.shape
        img = 255*np.ones((self.h*2, self.w*2), dtype=np.uint8)
        m, n = self.map.shape
        for j in range(m):
            for i in range(n):
                x0, y0 = i * self.d*2, j * self.d*2
                xk, yk = x0 + self.d*2, y0 + self.d*2

                if (j, i) in self.path_ji:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 3)
                else:
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 1)

                text = str(self.map[j, i])[:3]
                cv.putText(img, text, (x0+5, (y0+yk)//2), cv.FONT_HERSHEY_SIMPLEX, 0.5, 0, 1)

        cv.imshow('map', img)
    

    def _plot_grid(self, img):
        """
        separate into the same discrets
        """
        m, n = self.map.shape
        for j in range(m):
            for i in range(n):
                x0, y0 = i * self.d, j * self.d
                xk, yk = x0 + self.d, y0 + self.d
                if (j, i) == self.start:
                    # start (blue)
                    cv.rectangle(img, (x0, y0), (xk, yk), (255, 0, 0), -1)
                elif (j, i) == self.target:
                    # finish (red)
                    cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 255), -1)
                elif (j, i) in self.path:
                    # path (green)
                    cv.rectangle(img, (x0, y0), (xk, yk), (0, 255, 0), -1)
                elif self.map[j, i] >= 0:  
                    cv.rectangle(img, (x0, y0), (xk, yk), (150, 150, 150), 1)

        return img


    def planner(self):
        check_message = self._check_input_data()
        if check_message != 'OK':
            raise Exception(check_message)
        
        self.path = []

        self._set_map()

        j_tar, i_tar = self.target

        step = 1
        # step =   0   1
        queue = [ [] for _ in range(target_mark+1)]
        queue[step].append(self.start)

        while self.map[j_tar, i_tar] == target_mark:
            coords = queue[step]
            for j, i in coords:
                
                def new_step(jk, ik, step):
                    point = self.map[jk, ik]
                    if point >= 0:
                        if point == 0 or step + 1 < point:
                            self.map[jk, ik] = step + 1
                            queue[step+1].append((jk, ik))

                # up
                new_step(j, i-1, step)
                # down
                new_step(j, i+1, step)
                # left
                new_step(j-1, i, step)
                # right
                new_step(j+1, i, step)
            
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
        self.path_ji = [(j, i)]   # list of (j, i)

        m, n = self.map.shape
        step = self.map[j, i]
        while not ((j, i) == self.start):
            # diag (right-up)
            if self.map[j-1][i] == self.map[j][i+1] and self.map[j-1, i+1] == step - 2:
                j -= 1
                i += 1
                step -= 2
            # diag (right-down)
            elif self.map[j+1][i] == self.map[j][i+1] and self.map[j+1, i+1] == step - 2:
                j += 1
                i += 1
                step -= 2
            # diag (left-down)
            elif self.map[j+1][i] == self.map[j][i-1] and self.map[j+1, i-1] == step - 2:
                j += 1
                i -= 1
                step -= 2
            # diag (left-up)
            elif self.map[j-1][i] == self.map[j][i-1] and self.map[j-1, i-1] == step - 2:
                j -= 1
                i -= 1
                step -= 2

            # left
            elif i+1 < n and self.map[j, i+1] == step - 1:
                i += 1
                step -= 1
            # right
            elif i > 0 and self.map[j, i-1] == step - 1:
                i -= 1
                step -= 1
            # up
            elif j+1 < m and self.map[j+1, i] == step - 1:
                j += 1
                step -= 1
            # down
            else:
                j -= 1
                step -= 1

            self.path_ji.append((j, i))

        self.path_ji.reverse()

        # ls of (x, y)
        for (j, i) in self.path_ji:
            xy = i * self.d + self.d//2, j * self.d + self.d//2
            self.path.append(xy)

        self.path[0] = self.start_xy
        self.path[-1] = self.target_xy


if __name__ == '__main__':
    test = Lee(h=600, w=800, discrete=10)

    # 1
    # test.draw_img()

    test.load_img('maps/0_(60,140)_(660,400).jpg', (60,140), (660,400))

    now = datetime.now()
    test.planner()
    print('Time:', datetime.now() - now)

    test.show_img(grid=True, path=True)
    test.show_map()

    print('S =', int(test.get_len_path()))

    cv.waitKey(0)
    cv.destroyAllWindows()

