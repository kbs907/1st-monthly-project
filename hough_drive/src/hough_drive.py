#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image

pub = None
x1, x2 = 0, 0
lx1, lx2, rx1, rx2, lpos, rpos = 0, 0, 0, 0, 0, 0
bridge = CvBridge()
image = np.empty(shape=[0])
angle_filter, noise= [], []
Width = 640
Height = 480

Offset = 320
Gap = 40
t = 0.005

class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])


def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub
    
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = float(x_sum) / float(size * 2)
    y_avg = float(y_sum) / float(size * 2)

    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(lines, left=False, right=False):
    global Width, Height
    global Offset, Gap
    global lx1, lx2, rx1, rx2, lpos, rpos
    m, b = get_line_params(lines)
    
    
    if m == 0 and b == 0:
        if left :
		return lx1, lx2, lpos
	if right :
		return rx1, rx2, rpos
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

    return x1, x2, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global lx1, lx2, rx1, rx2, lpos, rpos

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 50
    high_threshold = 150
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return (0, 640), frame

    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    _, _, lpos_tmp = get_line_pos(left_lines, left=True)
    _, _, rpos_tmp = get_line_pos(right_lines, right=True)
    if rpos_tmp-lpos_tmp > 400 :
    	lx1, lx2, lpos = get_line_pos(left_lines, left=True)
    	rx1, rx2, rpos = get_line_pos(right_lines, right=True)
    else :
	if rpos_tmp-320 > 320-lpos_tmp :
		rx1, rx2, rpos = get_line_pos(right_lines, right=True)
		lpos = rpos - 520
	else :
		lx1, lx2, lpos = get_line_pos(left_lines, left=True)
		rpos = lpos + 520

    frame = cv2.line(frame, (int(lx1), Height), (int(lx2), (Height/2)), (255, 0,0), 3)
    frame = cv2.line(frame, (int(rx1), Height), (int(rx2), (Height/2)), (255, 0,0), 3)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    return (lpos, rpos), frame

def draw_steer(image, steer_angle):
    global Width, Height
   
    arrow_pic = cv2.imread('/home/nvidia/xycar_ws/src/hough_drive/src/steer_arrow.png', cv2.IMREAD_COLOR)
    
    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)

def start():
    global pub, angle_filter
    global image
    global Width, Height
   
    rospy.init_node('hough_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    mm1 = MovingAverage(20)
    mm2 = MovingAverage(20)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar B2 v1.0 ----------"
    rospy.sleep(2)

    while not image.size == (640*480*3):
        continue

    old_time = time.time()
    angle = 0
    steer_angle = 0
    angle_old = 0
    Kp_max = 0.6
    Kp_min = 0.05
#0.25 0.05 0.02
    Ki_max = 0.0
    Ki_min = 0.0
    Kd_max = 0.3
    Kd_min = 0.0
    angle_i = 0

    while not rospy.is_shutdown():
	dt = time.time() - old_time
	old_time = time.time()
        pos, frame = process_image(image)
	#print(pos[1]-pos[0])
	mm1.add_sample(pos[0])
	mm2.add_sample(pos[1])

        center = (mm1.get_wmm()+mm2.get_wmm()) / 2.0
	angle_temp = (center-320)
	
	
        draw_steer(frame, steer_angle)
	angle = angle_temp
	#angle = (t*angle + angle_temp*dt)/(t+dt)
	if abs(angle_temp) > 12 :
		angle_p = angle * Kp_max
	else :
		angle_p = angle * Kp_min
		angle_p = abs(angle_p)*angle_p
	if abs(angle_temp) > 12 :
		angle_i += angle * dt * Ki_max
	else :
		angle_i += angle * dt * Ki_min
	
	if angle_i > 5:
	    angle_i = 5
	elif angle_i < -5:
	    angle_i = -5
	if abs(angle_temp) > 12 :
		angle_d = (angle - angle_old) * Kd_max /(dt)
	else :
		angle_d = (angle - angle_old) * Kd_min /(dt)
	
	angle_pid = angle_p + angle_i + angle_d
	#print(angle_p, angle_i, angle_d, angle, angle_pid,dt)
	print(angle_pid)
	#angle = int((angle*t + angle_temp*dt)/(t+dt))
	
	angle_send = int(angle_pid)
        steer_angle = angle * 0.4
	angle_old = angle
	if abs(angle) < 8 :
		drive(0,20)	
	elif abs(angle) < 12 :
        	drive(angle_send, 22)
	else :
		drive(angle_send, 15)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()
