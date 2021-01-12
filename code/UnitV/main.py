#tested with frimware 5-0.22
import gc, random, time
import sensor, image, lcd
import KPU as kpu
from math import sqrt
from machine import UART
from fpioa_manager import fm


# consts
LEFT = "left"
RIGHT = "right"
WINDOW_SIZE = 224
IDs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,]
DIV = "_"

# init sensor
def init_sensor():
    lcd.init()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA) # VGA is over
    sensor.set_hmirror(1)
    sensor.set_windowing((WINDOW_SIZE, WINDOW_SIZE))
    sensor.set_vflip(1)
    sensor.run(1)

# init kpu
def init_kpu(threshold=0.3):
    classes = ["person"]
    task = kpu.load(0x300000) #change to "/sd/name_of_the_model_file.kmodel" if loading from SD card
    a = kpu.set_outputs(task, 0, 7,7, 30)   #the actual shape needs to match the last layer shape of your model(before Reshape)
    anchor = (0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828)
    a = kpu.init_yolo2(task, threshold, 0.3, 5, anchor) #tweak the second parameter if you're getting too many false positives
    return task

# init tracker
def init_tracker(buffer=2):
    g = Graph(buffer=2)
    return g

# init counter
def init_counter(patience=1):
    c = Counter(patience=patience)
    return c

# init uart
def init_uart():
    fm.register(35, fm.fpioa.UART1_TX, force=True)
    fm.register(34, fm.fpioa.UART1_RX, force=True)
    uart = UART(UART.UART1, 115200,8,0,0, timeout=1000, read_buf_len=4096)
    return uart

# initializer
def init(threshold=0.3, buffer=2, patience=1):
    init_sensor()
    task = init_kpu(threshold=threshold)
    graph = init_tracker(buffer=buffer)
    counter = init_counter(patience=patience)
    uart = init_uart()
    return task, graph, counter, uart

# box distance
def get_distance(box1, box2):
    '''Return "distance" between two boxes.
    The "distance" is the degree how close centers of two boxes are and how similar shape of two boxes seem
    "how close": euclid distance between centers
    "how similar": cosine similarity for weight and height of each box

    Parameters
    ----------
    box1: dict
        {x:, y:, w:, h:, value:}
    box2: dict
        {x:, y:, w:, h:, value:}

    Returns
    -------
    distance: float

    '''

    def euclid(x1, y1, x2, y2):
        return sqrt(((x2 - x1)**2 + (y2 - y1)**2))

    def cos_similarity(v1, v2):
        v1_norm, v2_norm, product = (0, 0, 0)
        for a, b in zip(v1, v2):
            product += a*b
            v1_norm += a**2
            v2_norm += b**2
        if v1_norm == 0 or v2_norm == 0:
            similarity = 0
        else:
            similarity = product / (sqrt(v1_norm)*sqrt(v2_norm))
        return similarity

    #print(box1["c"], box2["c"])
    center_d = euclid(box1["c"][0], box1["c"][1],
                      box2["c"][0], box2["c"][1]
                      )
    center_d /= WINDOW_SIZE # normlize
    cos_similarity = cos_similarity([box1["w"], box1["h"]],
                                    [box2["w"], box2["h"]])
    return -center_d + cos_similarity


class Frame():
    def __init__(self, f_index, img, code):
        self.f_index = f_index
        self.img = img
        self.bboxes = self.init_box(code)
        self.num_object = len(self.bboxes)

    def _get_center(self, rect):
        cx = (rect["x"] + rect["x"] + rect["w"]) // 2
        cy = (rect["y"] + rect["y"] + rect["h"]) // 2
        return cx, cy

    def init_box(self, code):
        d = {}
        if code:
            for idx, box in enumerate(code):
                rect = {"x":box.x(), "y":box.y(),
                        "w":box.w(), "h":box.h(),
                        "value":box.value(),}
                cx, cy = self._get_center(rect)
                rect["c"] = [cx, cy]
                d[idx] = rect
        else:
            pass
        return d

    def draw_frames(self,):
        for idx, rect in self.bboxes.items():
            self.img.draw_rectangle(rect["x"], rect["y"], rect["w"], rect["h"])
            self.img.draw_string(rect["x"] , rect["y"], str(idx), color=(255,0,0), scale=4)
            self.img.draw_circle(rect["c"][0], rect["c"][1], 3, (255,0,0),2,1)
        return self.img


class VanishCandidate():
    def __init__(self, idx, bbox, life=2):
        self.idx = idx
        self.bbox = bbox
        self.life = life

    def be_old(self):
        self.life -= 1
        return self.life

    def is_dead(self):
        return self.life == 0


class Counter():
    def __init__(self, patience=2, ids=IDs):
        self.counter = self.init_counter()
        self.vanish_cand_dict = self.init_cand_dict()
        self.patience = patience
        self.IDs = ids

    def init_counter(self):
        return {LEFT:0, RIGHT:0}

    def init_cand_dict(self):
        return {}

    def vanish_update(self, curr_bboxes, prev_bboxes, is_decrease, threshold=0.5):
        ''' Update vanishing candidate list
        Get indexes of difference between current and previous.
        Delete and add candidate by following:
        If one of indexes is already in vanishing list and its bbox is similar to one of list,
        delete the index in the list.
        Otherwise (and if decrease bboxes), add index to the list.


        Parameters
        ----------
        curr_bboxes: list of dict
            Bboxes list in current frame
        prev_bboxes: list of dict
            Bboxes list in just before 1F
        is_decrease: bool
            Whether current frame has many bboxes than previous one

        Returns
        -------
        None

        '''
        diff_idxs = set(curr_bboxes.keys()).symmetric_difference(prev_bboxes.keys())
        for diff_idx in diff_idxs:
            if diff_idx in self.vanish_cand_dict.keys():
                d = get_distance(curr_bboxes[diff_idx], self.vanish_cand_dict[diff_idx].bbox)
                if d > 0.5:
                    del self.vanish_cand_dict[diff_idx]
                else:
                    idx = min(set(self.IDs).difference(curr_bboxes.keys()).difference(self.vanish_cand_dict.keys()))
                    curr_bboxes[idx] = curr_bboxes[diff_idx]
                    del curr_bboxes[diff_idx]
            elif is_decrease:
                self.vanish_cand_dict[diff_idx] = \
                    VanishCandidate(diff_idx, prev_bboxes[diff_idx], self.patience)
            else:
                pass

    def _is_vanish(self, candidate):
        candidate.be_old()
        return candidate.is_dead()

    def _where_vanish(self, candidate):
        if not self._is_vanish(candidate):
            return None
        else:
            if candidate.bbox["c"][0] > (WINDOW_SIZE * 3 / 4):
                return RIGHT
            elif candidate.bbox["c"][0] < (WINDOW_SIZE / 4):
                return LEFT
            else:
                return None

    def count(self):
        del_idxs = []
        for idx, cand in self.vanish_cand_dict.items():
            where = self._where_vanish(cand)
            if type(where) == str:
                self.counter[where] += 1
                del_idxs.append(idx)
                #del self.vanish_cand_dict[idx]
                #gc.collect()
        for idx in del_idxs:
            del self.vanish_cand_dict[idx]
            gc.collect()


class Graph():
    def __init__(self, F_list=None, buffer=2,):
        self.F_list = F_list or []
        self.buffer = buffer
        self.is_decrease = False
        self.diff_idxs = None

    def add_frame(self, frame):
        if len(self.F_list) >= self.buffer:
            self.F_list.pop(0)
        self.F_list.append(frame)

    def _greedy(self, few_bboxes, many_bboxes, is_few_write):
        new_boxes1 = {}
        new_boxes2 = {}
        used = []
        for f_idx, f_box in few_bboxes.items():
            max_d = -1
            match_idx = 0
            for m_idx, m_box in many_bboxes.items():
                if m_idx in used:
                    continue
                d = get_distance(f_box, m_box)
                if d > max_d :
                    max_d = d
                    match_idx = m_idx
            used.append(match_idx)
            new_boxes1[match_idx] = f_box
            new_boxes2[f_idx] = many_bboxes[match_idx]

        if is_few_write:
            return new_boxes1
        else:
            remind_idxs = set(many_bboxes.keys()).difference(set(used))
            empty_idxs = set(many_bboxes.keys()).difference(set(new_boxes2.keys()))
            for empty_idx, re_idx in zip(empty_idxs, remind_idxs):
                new_boxes2[empty_idx] = many_bboxes[re_idx]
            return new_boxes2

    def greedy(self):
        try:
            prev_F = self.F_list[-2]
        except IndexError as e:
            self.add_frame(self.F_list[-1])
            prev_F = self.F_list[-2]
        curr_F = self.F_list[-1]

        #print(prev_F.f_index, prev_F.bboxes)

        if len(curr_F.bboxes) > len(prev_F.bboxes):
            new_boxes = self._greedy(prev_F.bboxes, curr_F.bboxes, False)
        else:
            new_boxes = self._greedy(curr_F.bboxes, prev_F.bboxes, True)

        self.F_list[-1].bboxes = new_boxes

    def check_idx_diff(self):
        self.is_decrease = len(self.F_list[-2].bboxes) > len(self.F_list[-1].bboxes)
        self.diff_idxs = set(self.F_list[-1].bboxes).symmetric_difference(self.F_list[-2].bboxes)
        return self.diff_idxs

    def track(self, frame):
        self.add_frame(frame)
        self.greedy()
        return self.check_idx_diff()

def main():
    ## main
    task, graph, counter, uart = init(threshold=0.5, patience=4)
    frame_idx = 0
    try:
        while(True):
            # get image
            img = sensor.snapshot().rotation_corr(z_rotation=0.0)

            # detect boxes
            a = img.pix_to_ai()
            code = kpu.run_yolo2(task, img)

            # set frame
            currF = Frame(frame_idx, img, code)

            # calc track
            diff_idx = graph.track(currF)

            # counting
            counter.vanish_update(graph.F_list[-1].bboxes, graph.F_list[-2].bboxes, graph.is_decrease)
            counter.count()

            # display on IDE
            #img = currF.draw_frames()
            #img = img.copy((32, 32, 160, 160))
            #img.draw_string(0 ,0, str(counter.counter[LEFT])+","+str(counter.counter[RIGHT]),
                            #color=(0,255,0), scale=3)
            #a = lcd.display(img)

            # to Gray
            msg = str(counter.counter[LEFT])+ DIV + \
                  str(counter.counter[RIGHT])+ DIV + str(currF.num_object)
            _ = uart.write(msg)
            #_ = uart.write(img)
            print(counter.counter)

            # finalize
            frame_idx += 1
            time.sleep(0.05)
    except Exception as e:
        # need delete kpu_task when keyboard interrupt
        a = kpu.deinit(task)
        del task
        gc.collect()
        print(e)

if __name__ == '__main__':
    main()
