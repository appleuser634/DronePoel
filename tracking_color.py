import cv2
import numpy as np
from sample import Tello
import threading
import time

#cap = cv2.VideoCapture(0)

tello = Tello()

#tello_add = "udp://@0.0.0.0:11111"
#cap = cv2.VideoCapture(tello_add)

font = cv2.FONT_HERSHEY_SIMPLEX

cmd = ""
loop_flag = True
def send_cmd_thread():
    global cmd,tello,loop_flag

    while loop_flag:
        if cmd == "" or cmd == None:
            continue
        tello.send_command(cmd)
        time.sleep(1)


def set_flight(center,cx):
    global cmd

    ok_error = 50
    left_lim = center - ok_error
    right_lim = center + ok_error
    print("left_lim:",left_lim,"right_lim:",right_lim)
    if cx >= left_lim and cx <=right_lim:
        print("OK GO FORWARD")
        cmd = "forward 30"
        #tello.send_command("forward 30")
    elif left_lim >= cx:
        print("send ccw")
        cmd = "ccw 10"
        #tello.send_command("ccw 10")
    elif right_lim <= cx:
        print("send cw")
        cmd = "cw 10"
        #tello.send_command("cw 10")

cmd_thread = threading.Thread(target=send_cmd_thread)
cmd_thread.start()

tello.takeoff()
time.sleep(3)

kernel = np.ones((5,5),np.uint8)
while True:
    frame = tello.frame

    try:
        center_line = int(frame.shape[1] / 2)
    except:
        continue

    gaus = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lo_color = np.array([0,30,50])
    hi_color = np.array([10,255,255])

    mask = cv2.inRange(hsv, lo_color, hi_color)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    colorImage = cv2.bitwise_and(frame, frame, mask=mask)
    
    try:
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        areas = [cv2.contourArea(c) for c in contours]
        areas = np.array(areas)

        max_index = np.argmax(areas)
        cnt = contours[max_index]
        
        M = cv2.moments(cnt)

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        set_flight(center_line,cx)
        
    except:
        cx = 0
        cy = 0
        cmd = "cw 20"
    
    cv2.circle(frame, (cx,cy), 5, (255,255,0), thickness=2)
    
    XY_text = "X=" + str(cx) + " Y=" + str(cy)
    cv2.putText(frame, XY_text, (cx,cy), font, 1, (0,255,255), 5, cv2.LINE_AA)
    
    cv2.imshow("show image!", frame)
    cv2.imshow("show color image!", colorImage)
    cv2.imshow("show mask!", mask)

    k = cv2.waitKey(1)
    if k == ord('q'):
        loop_flag = False
        break

cap.release()
cv2.destroyAllWindows()
