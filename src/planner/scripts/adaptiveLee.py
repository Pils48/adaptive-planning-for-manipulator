import numpy as np
import cv2 as cv
from datetime import datetime
import os
from planner import Planner

# ------------------------------------- Parametres -----------------------------------
MAX_discrete = 2**2     # max discrete 4 times more size than min discrete

MAX_step = 100          # limit of steps

# markers on the map
collsn_mark = -1
start_mark = 1
target_mark = MAX_step

class AdaptiveLee(Planner):
    def __init__(self, w=800, h=600, d=10):
        """
        :param h: height of img in pixels
        :param w: wiight of img in pixels
        """
        self.map = self.map_shape = None
        self.w, self.h = w, h
        self.d = d
        
        self.start_xy = self.target_xy = None     

        # ls of (d, j, i)
        self.path_ji = []  
        
        # ls of (x, y)
        self.path = []                                  
        

    def _set_map(self):
        """ build maps for all discrete sizes """
        img = self.img.copy()
        h, w, _ = img.shape
        self.target, self.start = None, None  # [d, j, i]

        # build map for each discrete size - d
        # d = {1, 2, 4, 8, ...}
        d = 1

        # each discrete has its own map
        self.map = dict()
        
        # sizes of maps
        self.map_shape = dict()

        while d <= MAX_discrete:
            # map size for discrete d
            m, n = h / (d * self.d), w / (d*self.d)
            
            # check size
            if h % (d * self.d) > 0 or w % (d*self.d) > 0:
                raise Exception('H({}) and W({}) must be divided on D({})'\
                                .format(h, w, d*self.d))
            
            self.map_shape[d] = (m, n)
            self.map[d] = np.zeros((m, n), np.int16)    # fill zeros
            d *= 2

        def recurrent(j0, i0, jk, ik, d):
            for j in range(j0, jk):
                for i in range(i0, ik):
                    # convert coords from (d, j, i) on map  ->  x,y on img
                    x0, y0 = i*d*self.d, j*d*self.d
                    xk, yk = (i+1)*d*self.d, (j+1)*d*self.d

                    # region of interests
                    roi = self.img[y0 : yk, x0 : xk, :]
                    
                    # if there's collision in this roi
                    if np.any(roi == 0):  
                        self.map[d][j][i] = -1
                        if d > 1:
                            # change for smaller discrete
                            recurrent(j*2, i*2, j*2+2, i*2+2, d//2)  
                    
                    # no collision in this roi
                    else:
                        # start marker
                        if x0 <= self.start_xy[0] < xk and \
                                                    y0 <= self.start_xy[1] < yk:
                            self.map[d][j][i] = start_mark
                            self.start = (d, j, i)

                        # end marker
                        if x0 <= self.target_xy[0] < xk and \
                                                    y0 <= self.target_xy[1] < yk:
                            self.map[d][j][i] = target_mark
                            self.target = (d, j, i)

        recurrent(0, 0, self.map_shape[MAX_discrete][0],
                        self.map_shape[MAX_discrete][1],
                        MAX_discrete)

        if self.target is None:
            # if there's a collision in min discrete and no target
            j, i = self.target_xy[1] // self.d, self.target_xy[0] // self.d
            self.map[1][j][i] = target_mark
            self.target = (1, j, i)
            x0, y0 = i*self.d, j*self.d
            xk, yk = (i+1)*self.d, (j+1)*self.d
            cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 255), -1)   # end is red

        if self.start is None:
            # if there's a collision in min discrete and no start
            j, i = self.start_xy[1] // self.d, self.start_xy[0] // self.d
            self.map[1][j][i] = start_mark
            self.start = (1, j, i)
            x0, y0 = i*self.d, j*self.d
            xk, yk = (i+1)*self.d, (j+1)*self.d
            cv.rectangle(img, (x0, y0), (xk, yk), (255, 0, 0), -1)   # start is blue
               
        self.img = img


    def _plot_grid(self, img):

        txt_size = self.d / (MAX_discrete * 20)

        def recurrent(j0, i0, jk, ik, d):
                for j in range(j0, jk):
                    for i in range(i0, ik):
                        if self.map[d][j][i] > collsn_mark:
                            # convert [d][j][i] on map  ->  (x, y) on img
                            x0, y0 = i*d*self.d, j*d*self.d
                            xk, yk = (i+1)*d*self.d, (j+1)*d*self.d
                            
                            cv.rectangle(img, (x0, y0), (xk, yk), (150, 150, 150), 1)

                            # set value of discrete to map
                            text = str(self.map[d][j][i])[:3]
                            cv.putText(img, text, (x0+2, (y0+yk)//2+2), cv.FONT_HERSHEY_SIMPLEX, 0.2*d, (0,0,0), 1)
                        else:
                            if d > 1:
                                recurrent(j*2, i*2, j*2+2, i*2+2, d // 2)

        m, n = self.map_shape[MAX_discrete]
        recurrent(0, 0, m, n, MAX_discrete)

        return img


    def show_map(self):
        """ increase scale in 2 """
        d = 1
        while d <= MAX_discrete:
            shape2 = (self.img_shape[0]*2, self.img_shape[1]*2)
            img = 255*np.ones(shape2, np.uint8)   # white img
            m, n = self.map_shape[d]
            for j in range(0, m):
                y0, yk = j * d * self.d * 2, (j+1) * d * self.d * 2
                for i in range(0, n):
                    x0, xk = i * d *self.d * 2, (i+1) * d * self.d * 2
                    cv.rectangle(img, (x0, y0), (xk, yk), 100, 1)
                    # set value of discrete to map
                    text = str(self.map[d][j][i])
                    cv.putText(img, text, (x0+1, (y0+yk)//2+2), cv.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)
            
            cv.imshow('MAP {}x'.format(d), img)
            d *= 2


    @staticmethod
    def _transform_coords(y0, x0, y1, x1, s0, s1):
        """
        coodrinates transform
        :params x0, y0: start coords in s0
        :params x1, y1: start coords in sk
        :params s0, s1: discrete size: 1, 2, 4    
        :return list of new coords [(j, i)]
        """     
        # let s0 > s1
        if s0 > s1:
            dx, dy = x1 - x0, y1 - y0
            ds = s0 // s1
            x1 = [x for x in range(x0 * ds, x0*ds + ds)]
            y1 = [y for y in range(y0 * ds, y0 * ds + ds)]

            # for dx > 0 x1 = x1[-1]
            # for dx < 0 x1 = x1[0]
            # for dy > 0 y1 = y1[-1]
            # for dy < 0 y1 = y1[0]
            def func(dq, q_ls):
                if dq > 0:
                    return [q_ls[-1]]
                elif dq < 0:
                    return [q_ls[0]]
                return q_ls

            x1 = func(dx, x1)
            y1 = func(dy, y1)
            
            results = []
            for x in x1:
                for y in y1:
                    results.append((y, x))
            return results

        # let s0 < s1
        if s0 < s1:
            ds = s1 // s0
            return [(y0 // ds, x0 // ds)]

        # let s0 = s1
        return [(y0, x0)]


    def planner(self):
        check_message = self._check_input_data()
        if check_message != 'OK':
            raise Exception(check_message)

        self._set_map()

        self.path = []

        d0, j0, i0 = self.start             # start point
        self.map[d0][j0][i0] = start_mark   # set this on map
        t_d, t_j, t_i = self.target         # target point
        
        # queue[step] -> ls of [(d, j, i)]
        queue = [[] for _ in range(MAX_step+1)]
        queue[start_mark] = [self.start]

        step = start_mark
        
        #  until we get target point
        while self.map[t_d][t_j][t_i] == target_mark:
            
            for d0, j0, i0 in queue[step]:

                # -------------------------------------------------------------------
                def move(d0, d, j0, i0, dj, di):
                    """
                    d0, d - start and target discrets
                    j0, i0 - start cell
                    dj, ij - step direction (+- 1)
                    """
                    coords = self._transform_coords(j0, i0, j0+dj, i0+di, d0, d) \
                        if d0 != d else [(j0, i0)]

                    for (j, i) in coords:
                        v = self.map[d][j+dj][i+di]
                        if v >= 0:
                            # if change is available and optimal
                            if v == 0 or step+1 < v:
                                self.map[d][j+dj][i+di] = step+1
                                self.map[d][j][i] = step
                                queue[step+1].append((d, j+dj, i+di))
                        elif d > 1:
                            # change for smaller discrete
                            move(d, d//2, j, i, dj, di)
                # -------------------------------------------------------------------

                # try to increase discrete
                d = MAX_discrete
                j, i = self._transform_coords(j0, i0, j0, i0, d0, d)[0]
                while d > d0 and self.map[d][j][i] == -1:
                    d = d // 2
                    j, i = self._transform_coords(j0, i0, j0, i0, d0, d)[0]
                # discrete is increased to d

                # 1) if no increase
                # 2) if this discrete has not be reached yet
                # 3) if more optimal path exists
                if d0 == d or self.map[d][j][i] == 0 or self.map[d][j][i] > step:
                    if d0 != d:
                        # increase discrete and set to map
                        self.map[d][j][i] = step
                    # up
                    if j > 0:
                        move(d, d, j, i, -1, 0)
                    # down
                    if j+1 < self.map_shape[d][0]:
                        move(d, d, j, i, +1, 0)
                    # right
                    if i+1 < self.map_shape[d][1]:
                        move(d, d, j, i, 0, +1)
                    # left
                    if i > 0:
                        move(d, d, j, i, 0, -1)

            step += 1
            if step > MAX_step:
                # limit is reached. No solution
                print('Limit')
                break

        if step <= MAX_step:
            # solution exists
            self._find_path()

        self.steps = step-start_mark


    def _find_path(self):
        """
        based on created maps
        get path - seq of coords (x, y)
        """
        # starting from the end (target)
        d, j, i = self.target       
        step = self.map[d][j][i]

        path = [(d, j ,i)]
        while (d, j, i) != self.start and step > 1:
            # ---- move if values of discretes vary to 1 ----
            # up
            if j > 0 and self.map[d][j-1][i] == step-1:
                path.append((d, j-1, i))
                j -= 1
                step -= 1
            # down
            elif j+1 < self.map_shape[d][0] and \
                                self.map[d][j+1][i] == step-1:
                path.append((d, j+1, i))
                j += 1
                step -= 1
            # right
            elif i > 0 and self.map[d][j][i-1] == step-1:
                path.append((d, j, i-1))
                i -= 1
                step -= 1
            # left
            elif i+1 < self.map_shape[d][1] and \
                                self.map[d][j][i+1] == step-1:
                path.append((d, j, i+1))
                i += 1
                step -= 1
                
            # change for discrete - d
            else:
                found = False
                d_new = MAX_discrete
                while not found:
                    if d == d_new:
                        d_new = d_new // 2
                    else:
                        coords = self._transform_coords(j, i, j, i, d, d_new)
                        for (y, x) in coords:
                            if self.map[d_new][y][x] == step:
                                if d_new > d:
                                    path[-1] = (d_new, y, x)
                                    self.map[d][j][i] = -1
                                d, j, i = d_new, y, x
                                found = True
                                break
                    
                        d_new = d_new // 2 #if d_new > 1 else MAX_discrete      

        # as we move from the target to start
        path.reverse()

        # coords to xy
        self.path = [self.start_xy]
        for (d, j, i) in path[1 :-1]:
            d = d * self.d
            x, y = i * d + d//2, j * d + d//2
            self.path.append((x, y))

        self.path.append(self.target_xy)


if __name__ == '__main__':
    test = AdaptiveLee(w=800, h=600, d=10)

    # 1
    images_dir = 'maps'
    indx = 8

    for f in os.listdir(images_dir):
        if not '.jpg' in f:
            continue
        if f[0] != str(indx):
            continue

        img_path = os.path.join(images_dir, f)
        break

    f = os.path.splitext(f)[0]
    i, start_point, target_point = f.split('_')
   
    start_point = tuple(start_point[1:-1].split(','))
    start_point = (int(start_point[0]), int(start_point[1]))

    target_point = tuple(target_point[1:-1].split(','))
    target_point = (int(target_point[0]), int(target_point[1]))

    test.load_img(img_path, start_point, target_point)

    # 2
    test.draw_img()

    now = datetime.now()
    test.planner()
    print('TIME:', datetime.now() - now)

    test.show_img(grid=True, path=True)

    print('S =', int(test.get_len_path()))
    cv.waitKey(0)
    cv.destroyAllWindows()



