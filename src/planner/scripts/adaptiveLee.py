import numpy as np
import cv2 as cv
from datetime import datetime

# ------------------------------------- Parametres -----------------------------------
MAX_discrete = 2**2     # макс. дискрета будет в 4 раза больше мин. дискреты

MAX_step = 100          # ограничение на кол-во шагов

# markers on the map
clean_mark = 0          # no collision
collsn_mark = -1
start_mark = 1
target_mark = MAX_step

class AdaptiveLee():
    def __init__(self, w, h, d=10):
        """
        :param h: height of img in pixels
        :param w: wiight of img in pixels
        """
        self.img_shape = (h, w)
        self.img = 255*np.ones((h, w, 3), np.uint8)                 # 3 channels
        self.map = self.map_shape = None
        self.d = d

        self.start_xy = ()     # (x, y) start point on img
        self.target_xy = ()    # (x, y) target point on img
        self.path = []         # ls of [d, j, i]
        self.steps = 0


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
                    cv.circle(img, (x,y), 3, (255, 1, 1), -1)
                    self.start_xy = (x, y)
                    isStart = False
                    switch = True
                if isTarget:
                    cv.circle(img, (x,y), 3, (1, 1, 255), -1)
                    self.target_xy = (x, y)
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

        while(True):
            cv.imshow('Draw collisions', img)
            if cv.waitKey(1) == 27:    # Esc
                if self.start_xy == () or self.target_xy == ():
                    print('Set start point and/or target point')
                else:
                    # end drawing
                    self.img = img
                    self._set_map()   # формирование карт для всех дискрет
                    break

        cv.destroyAllWindows()


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

        self.start_xy = start_point

        self.target_xy = target_point

        self._set_map()


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

        self.target_xy = target_point

        self._set_map()


    def _set_map(self):
        """ формирование карт для всех дискрет """
        img = self.img.copy()
        h, w, _ = img.shape
        self.target, self.start = None, None  # [d, j, i]

        # формирование карт для каждой дискреты d
        # d = {1, 2, 4, 8, ...}
        d = 1

        # для каждого размера дискрет существует свои карты
        self.map = dict()
        
        # размеры карт
        self.map_shape = dict()

        while d <= MAX_discrete:
            # размер карты с дискретой d
            m, n = h // (d * self.d), w // (d*self.d)
            self.map_shape[d] = (m, n)
            self.map[d] = np.zeros((m, n), np.int16)    # fill zeros
            d *= 2

        def recurrent(j0, i0, jk, ik, d):
            nonlocal img
            for j in range(j0, jk):
                for i in range(i0, ik):
                    # convert coords from (d, j, i) on map  ->  x,y on img
                    x0, y0 = i*d*self.d, j*d*self.d
                    xk, yk = (i+1)*d*self.d, (j+1)*d*self.d

                    roi = self.img[y0 : yk, x0 : xk, :]
                    if np.any(roi == 0):  # если есть препятствие в этом участке
                        self.map[d][j][i] = -1
                        if d > 1:
                            recurrent(j*2, i*2, j*2+2, i*2+2, d//2)  # переходим в дискрету меньше
                    else:
                        # отрисовка дискреты
                        #cv.rectangle(img, (x0, y0), (xk, yk), (200, 200, 200), 1)

                        # start marker
                        if x0 <= self.start_xy[0] < xk and \
                                                    y0 <= self.start_xy[1] < yk:
                            self.map[d][j][i] = start_mark
                            self.start = (d, j, i)
                            #cv.rectangle(img, (x0, y0), (xk, yk), (255, 0, 0), -1)  # start is blue

                        # end marker
                        if x0 <= self.target_xy[0] < xk and \
                                                    y0 <= self.target_xy[1] < yk:
                            self.map[d][j][i] = target_mark
                            self.target = (d, j, i)
                            #cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 255), -1)  # end is red

        recurrent(0, 0, self.map_shape[MAX_discrete][0],
                        self.map_shape[MAX_discrete][1],
                        MAX_discrete)

        if self.target is None:
            # если в мин. дискрету попали и препятствие, и конечное положение
            j, i = self.target_xy[1] // self.d, self.target_xy[0] // self.d
            self.map[1][j][i] = target_mark
            self.target = (1, j, i)
            x0, y0 = i*self.d, j*self.d
            xk, yk = (i+1)*self.d, (j+1)*self.d
            cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 255), -1)   # end is red

        if self.start is None:
            # если в мин. дискрету попали и препятствие, и начальное положение
            j, i = self.start_xy[1] // self.d, self.start_xy[0] // self.d
            self.map[1][j][i] = start_mark
            self.start = (1, j, i)
            x0, y0 = i*self.d, j*self.d
            xk, yk = (i+1)*self.d, (j+1)*self.d
            cv.rectangle(img, (x0, y0), (xk, yk), (255, 0, 0), -1)   # start is blue
               
        self.img = img


    def show_img(self, grid=True, path=True):
        """ картинка будет увеличена в 2 раза """
        img = cv.resize(self.img, (self.img_shape[1]*2, self.img_shape[0]*2), interpolation = cv.INTER_AREA)

        if self.steps > 0 and grid:    # 
            def recurrent(j0, i0, jk, ik, d):
                nonlocal img
                for j in range(j0, jk):
                    for i in range(i0, ik):
                        if self.map[d][j][i] > collsn_mark:
                            # convert [d][j][i] on map  ->  (x, y) on img
                            x0, y0 = i*d*self.d*2, j*d*self.d*2
                            xk, yk = (i+1)*d*self.d*2, (j+1)*d*self.d*2
                            
                            cv.rectangle(img, (x0, y0), (xk, yk), (0, 0, 0), 2)

                            if self.steps > 0:
                                # отображение значения в данной дискрете
                                text = str(self.map[d][j][i])[:3]
                                cv.putText(img, text, (x0+5, (y0+yk)//2+2), cv.FONT_HERSHEY_SIMPLEX, 1/(MAX_discrete/d), (0,0,0), 1)
                        else:
                            if d > 1:
                                recurrent(j*2, i*2, j*2+2, i*2+2, d // 2)

            m, n = self.map_shape[MAX_discrete]
            recurrent(0, 0, m, n, MAX_discrete)

        if path and self.path:
            d, j, i = self.path[0]
            x0, y0 = int((d*self.d)*(i+0.5)), int((d*self.d)*(j+0.5))
            for (d, j, i) in self.path[1:]:
                x, y = int((d*self.d)*(i+0.5)), int((d*self.d)*(j+0.5))
                cv.line(img, (x0*2, y0*2), (x*2, y*2), (0, 255, 0), 3)
                x0, y0 = x, y

        cv.imshow('img', img)


    def show_map(self):
        """ увеличение масштаба в 2 раза """
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
                    # отображение значения в данной дискрете
                    text = str(self.map[d][j][i])
                    cv.putText(img, text, (x0+1, (y0+yk)//2+2), cv.FONT_HERSHEY_SIMPLEX, 0.3, 0, 1)
            
            cv.imshow('MAP {}x'.format(d), img)
            d *= 2


    @staticmethod
    def _transform_coords(y0, x0, y1, x1, s0, s1):
        """
        преобразование координат
        :params x0, y0: начальные координаты в s0
        :params x1, y1: начальные координаты в sk
        :params s0, s1: системы координат 1, 2, 4    
        :return list of new coords [(j, i)]
        """     
        # пусть s0 > s1
        if s0 > s1:
            dx, dy = x1 - x0, y1 - y0
            ds = s0 // s1
            x1 = [x for x in range(x0 * ds, x0*ds + ds)]
            y1 = [y for y in range(y0 * ds, y0 * ds + ds)]

            # при dx > 0 x1 = x1[-1]
            # при dx < 0 x1 = x1[0]
            # при dy > 0 y1 = y1[-1]
            # при dy < 0 y1 = y1[0]
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

        # пусть s0 < s1
        if s0 < s1:
            ds = s1 // s0
            return [(y0 // ds, x0 // ds)]

        # пусть s0 = s1
        return [(y0, x0)]


    def planner(self):
        if self.map is None:
            raise Exception('Map is undefined. Use load_map() or draw_map()')

        d0, j0, i0 = self.start             # начальное положение
        self.map[d0][j0][i0] = start_mark   # отмечаем на карте
        t_d, t_j, t_i = self.target         # конечное положение
        
        queue = [[] for _ in range(MAX_step+1)]
        # очередь из координат queue[step] -> ls of [(d, j, i)]
        queue[start_mark] = [self.start]

        step = start_mark
        #  пока не дойдем до конечной конечного положения
        while self.map[t_d][t_j][t_i] == target_mark:
            
            # идем по очереди для каждого шага
            for d0, j0, i0 in queue[step]:

                # -------------------------------------------------------------------
                def move(d0, d, j0, i0, dj, di):
                    """
                    d0, d - начальное и конечное значение дискреты
                    j0, i0 - начальные значения ячеек
                    dj, ij - переход между ячейками
                    """
                    coords = self._transform_coords(j0, i0, j0+dj, i0+di, d0, d) \
                        if d0 != d else [(j0, i0)]

                    for (j, i) in coords:
                        v = self.map[d][j+dj][i+di]
                        if v >= 0:
                            # если переход возможен и оптимален
                            if v == 0 or step+1 < v:
                                self.map[d][j+dj][i+di] = step+1
                                self.map[d][j][i] = step
                                queue[step+1].append((d, j+dj, i+di))
                        elif d > 1:
                            # пробуем переход в дискрете поменьше
                            move(d, d//2, j, i, dj, di)
                # -------------------------------------------------------------------

                # пробуем увеличить дискрету
                d = MAX_discrete
                j, i = self._transform_coords(j0, i0, j0, i0, d0, d)[0]
                while d > d0 and self.map[d][j][i] == -1:
                    d = d // 2
                    j, i = self._transform_coords(j0, i0, j0, i0, d0, d)[0]
                # дискрета увеличена до d

                # 1) если увеличить дискрету не получилось
                # 2) если в дикрету еще не переходили
                # 3) если нашелся более оптимальный путь
                if d0 == d or self.map[d][j][i] == 0 or self.map[d][j][i] > step:
                    if d0 != d:
                        # в увеличенную дискрету запишем текущий step
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
                # превышение лимита. Нет решений
                print('Limit')
                break

        if step <= MAX_step:
            # если решение есть
            print('build path...')
            self._find_path()

        self.steps = step-start_mark
        print('Total steps:', self.steps)


    def _find_path(self):
        """
        на основе заполненных карт
        получить путь - последовательность координат
        """
        # начинаем с конца
        d, j, i = self.target       
        step = self.map[d][j][i]

        path = [(d, j ,i)]
        while (d, j, i) != self.start and step > 1:
            # ---- мы переходим, если ячейки отличаются на 1 ----
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
            # иначе есть переход между дискретами
            else:
                found = False
                d_new = MAX_discrete
                while not found:
                    if d == d_new:
                        d_new = d_new // 2 #if d_new > 1 else MAX_discrete
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

        # т.к. мы шли с конца, нужно развернуть
        path.reverse()
        self.path = path


    def get_path(self, array=True):
        if len(self.path) < 2:
            print('No path exists')
            return None

        path = []
        for (d, j, i) in self.path:
            x, y = (d*self.d)*i + self.d//2, (d*self.d)*j + self.d//2

        n = len(path)
        if array:
            array_path = np.empty((n, 2), dtype=np.int16)
            for i, (x, y) in enumerate(path):
                array_path[i][0] = x
                array_path[i][1] = y
            return array_path

        return path



test = AdaptiveLee(w=400, h=320)

# 1
test.load_img('map.jpg', (10, 10), (350, 300))

# 2
#test.draw_img()

now = datetime.now()
test.planner()
print('TIME:', datetime.now() - now)

test.show_img(grid=True, path=True)

cv.waitKey(0)
cv.destroyAllWindows()



