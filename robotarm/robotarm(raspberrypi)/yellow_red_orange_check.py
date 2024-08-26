import cv2
import numpy as np



"""from recieve_data_to_MCU import send_data
import serial
bluetoothSerial = serial.Serial('COM8', baudrate=9600,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS) """


def is_frame_yellow(frame, threshold=0.4):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([25, 50, 50])
    upper_yellow = np.array([33, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_ratio = np.sum(mask == 255) / (frame.shape[0] * frame.shape[1])
    cv2.imshow('YELLOW', mask)
    return int(yellow_ratio > threshold)

def is_frame_red(frame, threshold=0.4):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([175, 50, 70]) 
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    red_ratio = np.sum(mask == 255) / (frame.shape[0] * frame.shape[1])
    cv2.imshow('RED', mask)
    return int(red_ratio > threshold)

def is_frame_orange(frame, threshold=0.4):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([12, 50, 50])
    upper_orange = np.array([24, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    orange_ratio = np.sum(mask == 255) / (frame.shape[0] * frame.shape[1])
    cv2.imshow('ORANGE', mask)
    return int(orange_ratio > threshold)

"""cap = cv2.VideoCapture(1)  # 웹캠을 사용하기 위해 1을 인자로 전달
while True:
    # 웹캠으로부터 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break

    annotated_frame = frame.copy()  # 프레임 복사본에 그리기

    yellow = is_frame_yellow(frame, threshold=0.4)
    if(yellow == 1):
        grap = b'U'
        send_data(bluetoothSerial, grap)


    cv2.imshow('yellow test', annotated_frame)
     # 'q' 키를 누르면 루프 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
bluetoothSerial.close()
cap.release()
cv2.destroyAllWindows()"""
