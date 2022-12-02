import cv2
import math

enemy_color = 'blue'
rune_armors = []
target = None


def tuple_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def tuple_mul(A, a):
    A = list(A)
    A = [a * i for i in A]
    return tuple(A)


class Vane:
    def __init__(self, vane_contour, epsilon):
        self.contour = vane_contour
        self.rrect = cv2.minAreaRect(self.contour)
        width, height = self.rrect[1]
        self.rrect_area = width * height
        self.hull = cv2.approxPolyDP(self.contour, epsilon, True)
        self.hull_num = len(self.hull)
        self.c_area = cv2.contourArea(vane_contour)


class Armor:
    def __init__(self, armor_contour, vane_contour, epsilon):
        self.vane = Vane(vane_contour, epsilon)
        self.contour = armor_contour
        self.rrect = cv2.minAreaRect(armor_contour)
        regularRotated(self.rrect)
        self.c_area = cv2.contourArea(armor_contour)
        self.area_ratio = self.c_area / self.vane.c_area
        self.r_direction = tuple_sub(self.rrect[0], self.vane.rrect[0])
        self.angel = math.atan2(self.r_direction[1], self.r_direction[0])
        a = tuple_mul(self.vane.rrect[0], 3.3)
        b = tuple_mul(self.rrect[0], 2.3)
        self.circle_center = tuple_sub(a, b)

    def __gt__(self, other):
        if self.vane.hull_num > other.vane.hull_num:
            return True
        else:
            return self.area_ratio > other.area_ratio

    def __lt__(self, other):
        return not (self > other)


def run(frame):
    binary = pre_processing(frame)
    frame = findRuneArmor(binary, frame)
    # print(rune_armors)
    return frame


def pre_processing(frame):
    gray = cv2.cvtColor(frame,  cv2.COLOR_BGR2GRAY)
    B, G, R = cv2.split(frame)
    if enemy_color == 'blue':
        _, gray_bin = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)
        color_bin = cv2.subtract(B, R)
        _, color_bin = cv2.threshold(color_bin, 90, 255, cv2.THRESH_BINARY)
    else:
        _, gray_bin = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)
        color_bin = cv2.subtract(R, B)
        _, color_bin = cv2.threshold(color_bin, 30, 255, cv2.THRESH_BINARY)
    binary = gray_bin & color_bin

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.dilate(binary, element)

    return binary


def findRuneArmor(binary, frame):
    global rune_armors
    rune_armors.clear()
    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contours), len(hierarchy))
    # print(hierarchy)
    # frame = cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
    for i in range(len(contours)):
        son_area = cv2.contourArea(contours[i])
        if son_area < 20:
            continue
        son_box = cv2.minAreaRect(contours[i])
        regularRotated(son_box)

        length, width = son_box[1]
        if length < width:
            length, width = width, length

        son_box_area = length * width

        if (length / width < 1.2 or
            length / width > 2.5 or
            son_box_area < 700 or
                son_box_area > 1700):
            continue

        son_hull = cv2.approxPolyDP(contours[i], 1.0 * 4.0, True)

        if (len(son_hull) < 4 or len(son_hull) > 10):
            continue

        j = i
        while j < len(contours) and j > -1:
            j = hierarchy[0][j][3]

            father_area = cv2.contourArea(contours[j])
            if (son_area / father_area < 0.15 or
                    son_area / father_area > 0.55):
                continue

            father_hull = cv2.approxPolyDP(contours[j], 1.0, True)
            if len(father_hull) < 6:
                continue

            father_box = cv2.minAreaRect(contours[j])

            length, width = father_box[1]
            if length < width:
                length, width = width, length

            father_box_area = length * width

            if length / width < 1.2 or length / width > 2.5 \
                or father_area / father_box_area < 0.2 \
                    or father_area / father_box_area > 0.9 \
            or father_box_area < 7000 \
            or father_box_area > 15000:
                continue
            # print(1)
            rune_armors.append(Armor(contours[i], contours[j], 1.0))
            break

    if len(rune_armors) == 0:
        return False

    chooseTarget(rune_armors)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
    cv2.drawContours(frame, [target.vane.contour], -1, (0, 0, 255), 1)
    # return chooseTarget(rune_armors)
    return frame


def chooseTarget(candinate):
    candinate = sorted(candinate)
    for i in range(len(candinate)):

        if not isTarget(candinate[i]):
            return False

        if not isActivated(candinate[i]):
            global target
            target = candinate[i]
    return True


def isTarget(armor):

    if armor.area_ratio > 0.55 or \
            armor.area_ratio < 0.15 or \
            armor.vane.hull_num < 10 or \
            armor.vane.hull_num > 80 or \
            armor.vane.c_area / armor.vane.rrect_area < 0.3 or \
            armor.vane.c_area / armor.vane.rrect_area > 0.7:
        return False

    return True


def isActivated(t_armor):
    if (t_armor.area_ratio < 0.15 or
        t_armor.area_ratio > 0.35 or
        t_armor.vane.c_area / t_armor.vane.rrect_area < 0.7 or
        t_armor.vane.c_area / t_armor.vane.rrect_area > 0.9 or
            t_armor.vane.hull_num < 6 or t_armor.vane.hull_num > 30):
        return False
    return True


def calCenter(t_armor, binary):

    pass


def regularRotated(r):
    """
    r[1]为(width,height)
    """
    r = list(r)
    width, height = r[1]
    if height < width:
        r[1] = (height, width)
        r[2] = r[2] - 90.0 if r[2] >= 0.0 else r[2] + 90.0
    if r[2] < 0:
        r[2] += 180.0
    r = tuple(r)
