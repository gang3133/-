from multiprocessing import Process, Value, Lock
import datetime
import time
from picamera2 import Picamera2
import cv2
from ultralytics import YOLO
import cap_cv
import puttext
import calculate_y
import draw
import numpy as np
from collections import Counter
import gpiozero
from bluetooth import *
import serial
import smbus
import math




ser = serial.Serial('/dev/ttyAMA0', 9600)
shared_z_angle = Value('f',0.0)
real_z_angle = Value('d',0)
run_stop = Value('d', 0)
lock = Lock()

socket = BluetoothSocket( RFCOMM )
socket.connect(("98:DA:60:04:C0:64", 1))
print("bluetooth connected!")

button = gpiozero.Button(18)

def process_video(real_z_angle, shared_z_angle, run_stop, lock):
    
    CONFIDENCE_THRESHOLD = 0.5
    GREEN = (0, 255, 0)
    WHITE = (255, 255, 255)
    in_30 = 1
    out_30 = 0

    
    model = YOLO("/home/pi/Downloads/model_- 29 may 2024 20_30_edgetpu.tflite")

    # Initialize the Picamera2
    picam2 = Picamera2()
    picam2.zoom = (0.0, 0.0, 1, 1)  # 0.5배 줌 아웃
    config = picam2.create_still_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    lower_green = np.array([40, 90, 20])
    upper_green = np.array([70, 255, 178])

    path_points = []
    point_lifetime = 0.1  # 점이 유지되는 시간 (초)

    switch = 1
    run_time_SS = 0;
    cnt = 0
    cnt_go1 = 0
    cnt_go2 = 0
    real_cnt = 0

    time.sleep(3)
    while True:
        with lock:
            serial_run_stop = run_stop.value
        if  button.is_pressed or serial_run_stop:
            if switch == 0:
                socket.send(b'04E')
            switch = 1
            cnt = 0
            cnt_go1 = 0
            cnt_go2 = 0
        else:
            if switch == 1:
                time.sleep(2)
                #socket.send(b'07E')
                time.sleep(0.1)
                socket.send(b'02E')
                switch = 0
                run_time_SS  = 0
                
            
                        
            start = datetime.datetime.now()
            # Capture frame-by-frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            
            frame_width = frame.shape[1]
            frame_height = frame.shape[0]
            center_x = frame_width // 2
            center_y = frame_height // 2
            
            request = picam2.capture_request()
            image = request.make_image("main")
            image1 = np.array(request.make_image("main"))
            request.release()
            annotated_frame = image1.copy()
            annotated_frame = cv2.cvtColor(annotated_frame,cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_green, upper_green)

            detection = model(frame)[0]
            
            red_line_left = 150
            red_line_right = 470


            blue_line_right = 95  
            line_left = 525  
            
            left_x1 = 2
            left_y1 = 382
            left_x2 = 151
            left_y2 = 332
            left_m = (left_y2 - left_y1) / (left_x2 - left_x1)
            left_b = left_y1 - left_m * left_x1
            
            right_x1 = 648
            right_y1 = 382
            right_x2 = 489
            right_y2 = 332
            right_m = (right_y2 - right_y1) / (right_x2 - right_x1)
            right_b = right_y1 - right_m * right_x1

            cv2.line(annotated_frame, (left_x1, left_y1), (left_x2, left_y2), (0, 0, 255), 5)
            cv2.line(annotated_frame, (right_x1, right_y1), (right_x2, right_y2), (0, 0, 255), 5)





            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            current_time = time.time()

            left_object_x = None
            left_object_y = None
            right_object_x = None 
            right_object_y = None
            
            with lock:
                current_z_angle = shared_z_angle.value
            with lock:
                real_z_angle.value = 0

            for contour in contours:
                if cv2.contourArea(contour) < 0.1:
                    continue

                x, y, w, h = cv2.boundingRect(contour)

                if not (320 <= y <= 480 and (x <= 150 or x >= 490)):
                    continue

                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 왼쪽 객체와 오른쪽 객체의 위치를 찾습니다.
                    if left_object_x is None or cx < left_object_x:
                        left_object_x = cx
                        left_object_y = cy
                    if right_object_x is None or cx > right_object_x:
                        right_object_x = cx
                        right_object_y = cy

                    path_points.append(((cx, cy), current_time, 'green'))
                    cv2.circle(annotated_frame, (cx, cy), 1, (0, 255, 0), 3)

            for data in detection.boxes.data.tolist():
                    result = draw.draw_rectangle(data, frame, CONFIDENCE_THRESHOLD, GREEN)
                    if result is None:
                        continue

                    xmin, ymin, xmax, ymax, width, height = result
                    obj_center_x = (xmin + xmax) // 2
                    obj_center_y = (ymin + ymax) // 2
                    width_text = f'{width:.1f}'
                    cv2.putText(frame, width_text, (xmin, ymax + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
                    height_text = f'{height:.1f}'
                    cv2.putText(frame, height_text, (xmin, ymax + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
                    y = calculate_y.calculate_y1(width, height)
                    puttext.draw_labels(frame, xmin, ymin, ymax, y, in_30, out_30, WHITE)
                    
                    detect = puttext.draw_labels(frame, xmin, ymin, ymax, y, in_30, out_30, WHITE)
                    if obj_center_x < center_x:
                        socket.send(b'05E')
                        time.sleep(0.1)
                        socket.send(b'02E')
                    elif obj_center_x > center_x:
                        socket.send(b'06E')
                        time.sleep(0.1)
                        socket.send(b'02E')
                    if(detect == 1):
                        if(cnt == 2) :
                            cnt = 0
                            real_cnt = real_cnt + 1
                        if(real_cnt % 2 == 0):
                            run_time_SS = 1
                            socket.send(b'04E')
                            time.sleep(0.1)
                            socket.send(b'06E')
                            time.sleep(7.5)
                            if run_time_SS == 1:
                                #socket.send(b'07E')
                                time.sleep(0.1)
                                socket.send(b'02E')
                                run_time_SS = 0
                                with lock:
                                    real_z_angle.value = 1
                                time.sleep(0.1)
                        else:
                            run_time_SS = 1
                            socket.send(b'04E')
                            time.sleep(0.1)
                            socket.send(b'05E')
                            time.sleep(7.5)
                            if run_time_SS == 1:
                                #socket.send(b'07E')
                                time.sleep(0.1)
                                socket.send(b'02E')
                                run_time_SS = 0
                                with lock:
                                    real_z_angle.value = 1
                                time.sleep(0.1)
                        cnt = cnt + 1
                    else:
                        with lock:
                            real_z_angle.value = 0
                        
                    
                                    
             #if robot turn left                       
            if(355 > current_z_angle > 270):
                run_time_SS = 1
                socket.send(b'05E')
                while not ((0 <= current_z_angle <= 2) or (358 <= current_z_angle <= 360)):
                    with lock:
                        current_z_angle = shared_z_angle.value
                    if  button.is_pressed:
                        if switch == 0:
                            socket.send(b'04E')
                            break
                        
                if run_time_SS == 1:
                    #socket.send(b'07E')
                    time.sleep(0.1)
                    socket.send(b'02E')
                    run_time_SS = 0
            
            #if robot turn right
            if(90 > current_z_angle > 5):
                run_time_SS = 1
                socket.send(b'06E')
                while not ((0 <= current_z_angle <= 2) or (358 <= current_z_angle <= 360)):
                    
                    with lock:
                        current_z_angle = shared_z_angle.value
                    if  button.is_pressed:
                        if switch == 0:
                            socket.send(b'04E')
                            break
                if run_time_SS == 1:
                    #socket.send(b'07E')
                    time.sleep(0.1)
                    socket.send(b'02E')
                    run_time_SS = 0
            
 
            if left_object_x is not None and right_object_x is not None:
                
                if cnt_go1 == 2:
                    cnt_go1 = 0
                    
                if cnt_go1 == 1:
                    if left_object_y < left_m * left_object_x + left_b and right_object_y < right_m * right_object_x + right_b:
                        with lock:
                            real_z_angle.value = 1
                    elif left_object_y < left_m * left_object_x + left_b and right_object_y > right_m * right_object_x + right_b:
                        socket.send(b'05E')
                        time.sleep(0.2)
                        socket.send(b'02E')
                    elif left_object_y > left_m * left_object_x + left_b and right_object_y < right_m * right_object_x + right_b:
                        socket.send(b'06E')
                        time.sleep(0.2)
                        socket.send(b'02E')
                    elif left_object_x > 320 and right_object_x > 320:
                        socket.send(b'05E')
                        time.sleep(0.5)
                        socket.send(b'02E')
                        with lock:
                            real_z_angle.value = 1
                    elif left_object_x < 320 and right_object_x < 320:
                        socket.send(b'06E')
                        time.sleep(0.5)
                        socket.send(b'02E')
                        with lock:
                            real_z_angle.value = 1
                    else:
                        with lock:
                            real_z_angle.value = 0
                cnt_go1 = cnt_go1 + 1
                
            elif left_object_x is None and right_object_x is None:
                
                if cnt_go2 == 2:
                    cnt_go2 = 0
                    
                if cnt_go2 == 1:
                    with lock:
                        real_z_angle.value = 1
                cnt_go2 = cnt_go2 + 1

            end = datetime.datetime.now()
            total = (end - start).total_seconds()
            print(f'Time to process 1 frame: {total * 1000:.0f} milliseconds')
            fps = f'FPS: {1 / total:.2f}'
            
            cv2.putText(frame, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow('detect', frame)
            
            cv2.putText(annotated_frame, fps, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow("gradient", annotated_frame)
            
            
            
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
    picam2.stop()
    socket.close()

MPU_ADDR = 0x68
bus = smbus.SMBus(1)  # I2C bus 1 (라즈베리파이 4)

AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ = [0] * 7

angleAcX = 0.0
angleAcY = 0.0
angleAcZ = 0.0
angleGyX = 0.0
angleGyY = 0.0
angleGyZ = 0.0

RADIAN_TO_DEGREES = 180 / 3.14159
DEGREE_PER_SECOND = 250.0 / 32767.0  # 각속도 센서의 감도

now = 0
past = 0
dt = 0

baseAcX = 0.0
baseAcY = 0.0
baseAcZ = 0.0
baseGyX = 0.0
baseGyY = 0.0
baseGyZ = 0.0


angleX = 0.0
angleY = 0.0
angleZ = 0.0

def process_IMU_sensor(real_z_angle, shared_z_angle, run_stop, lock):
    
    

    def initSensor():
        bus.write_byte_data(MPU_ADDR, 0x6B, 0)

    def getData():
        global AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ
        data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
        AcX = (data[0] << 8) | data[1]
        AcY = (data[2] << 8) | data[3]
        AcZ = (data[4] << 8) | data[5]
        Tmp = (data[6] << 8) | data[7]
        GyX = (data[8] << 8) | data[9]
        GyY = (data[10] << 8) | data[11]
        GyZ = (data[12] << 8) | data[13]

        # 두 바이트가 음수 값인지 확인하여 변환
        if AcX >= 0x8000: AcX = -((65535 - AcX) + 1)
        if AcY >= 0x8000: AcY = -((65535 - AcY) + 1)
        if AcZ >= 0x8000: AcZ = -((65535 - AcZ) + 1)
        if GyX >= 0x8000: GyX = -((65535 - GyX) + 1)
        if GyY >= 0x8000: GyY = -((65535 - GyY) + 1)
        if GyZ >= 0x8000: GyZ = -((65535 - GyZ) + 1)

    def getDT():
        global now, past, dt
        now = time.time()
        dt = now - past
        past = now

    def calibrateSensor():
        global baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ
        sumAcX = 0.0
        sumAcY = 0.0
        sumAcZ = 0.0
        sumGyX = 0.0
        sumGyY = 0.0
        sumGyZ = 0.0
        for _ in range(100):  # 10번을 100번으로 증가시켜 더 정확히 보정
            getData()
            sumAcX += AcX
            sumAcY += AcY
            sumAcZ += AcZ
            sumGyX += GyX
            sumGyY += GyY
            sumGyZ += GyZ
            time.sleep(0.01)
        baseAcX = sumAcX / 100
        baseAcY = sumAcY / 100
        baseAcZ = sumAcZ / 100
        baseGyX = sumGyX / 100
        baseGyY = sumGyY / 100
        baseGyZ = sumGyZ / 100

        # 초기 각도를 기준으로 설정
        initial_angleZ = math.atan2(baseAcY, baseAcX) * RADIAN_TO_DEGREES

    def main():
        
        global angleAcX, angleAcY, angleAcZ, angleGyX, angleGyY, angleGyZ, angleX, angleY, angleZ
        initSensor()
        calibrateSensor()
        global past
        past = time.time()
        go = 0

        while True:
            with lock:
                serial_run_stop = run_stop.value
            if  button.is_pressed or serial_run_stop:
                
                baseAcX = 0.0
                baseAcY = 0.0
                baseAcZ = 0.0
                baseGyX = 0.0
                baseGyY = 0.0
                baseGyZ = 0.0

                angleAcX = 0.0
                angleAcY = 0.0
                angleAcZ = 0.0
                angleGyX = 0.0
                angleGyY = 0.0
                angleGyZ = 0.0
                
                angleX = 0.0
                angleY = 0.0
                angleZ = 0.0
                
                initSensor()
                calibrateSensor()
                past = 0
                past = time.time()
            else:
                with lock:
                    go = real_z_angle.value
                if go == 1:
             
                    #baseAcX = 0.0
                    #baseAcY = 0.0
                    #baseAcZ = 0.0
                    #baseGyX = 0.0
                    #baseGyY = 0.0
                    #baseGyZ = 0.0

                    angleAcX = 0.0
                    angleAcY = 0.0
                    angleAcZ = 0.0
                    angleGyX = 0.0
                    angleGyY = 0.0
                    angleGyZ = 0.0
                
                    angleX = 0.0
                    angleY = 0.0
                    angleZ = 0.0
                
                    initSensor()
                    #calibrateSensor()
                    past = 0
                    past = time.time()
                    go = 0
                    real_z_angle.value = 0
                    
                getData()
                getDT()

                # 가속도 센서 값을 이용하여 각도 계산
                angleAcX = math.atan2(AcY - baseAcY, AcZ - baseAcZ) * RADIAN_TO_DEGREES
                angleAcY = math.atan2(AcX - baseAcX, AcZ - baseAcZ) * RADIAN_TO_DEGREES
                angleAcZ = math.atan2(AcY - baseAcY, AcX - baseAcX) * RADIAN_TO_DEGREES

                # 각속도 센서 값을 이용하여 각도 계산
                angleGyX += (GyX - baseGyX) * DEGREE_PER_SECOND * dt
                angleGyY += (GyY - baseGyY) * DEGREE_PER_SECOND * dt
                angleGyZ += (GyZ - baseGyZ) * DEGREE_PER_SECOND * dt

                # 컴플리멘터리 필터를 이용하여 두 센서의 값을 융합
                alpha = 0.98  # 컴플리멘터리 필터 상수
                angleX = alpha * angleGyX + (1 - alpha) * angleAcX
                angleY = alpha * angleGyY + (1 - alpha) * angleAcY
                angleZ = alpha * angleGyZ + (1 - alpha) * angleAcZ

                # 각도 값을 0~360도 범위로 조정
                angleX = (angleX + 360) % 360
                angleY = (angleY + 360) % 360
                angleZ = (angleZ + 360) % 360

                    
                # Z축 각도만 출력
                print(f"Z축 각도: {angleZ: .1f} 도")
                
                with lock:
                    shared_z_angle.value = angleZ
                
                time.sleep(0.1)  # 처리 속도 조절

    if __name__ == "__main__":
        main()

def read_serial(real_z_angel, run_stop, lock):
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f'Received: {line}') 

            if line == 'stop':
                print(line)
                try:
                    socket.send(b'04E')
                    with lock:
                        real_z_angle.value = 1
                        run_stop.value = 1
                except Exception as e:
                    print(f'Error sending data: {e}')

            elif line == 'go':
                print(line)
                try:
                    socket.send(b'02E')
                    with lock:
                        real_z_angle.value = 1
                        run_stop.value = 0
                except Exception as e:
                    print(f'Error sending data: {e}')

p1 = Process(target=process_IMU_sensor, args=(shared_z_angle, real_z_angle, run_stop, lock))
p2 = Process(target=process_video, args=(shared_z_angle, real_z_angle, run_stop, lock))
p3 = Process(target=read_serial, args=(real_z_angle, run_stop, lock))

p1.start()
p2.start()
p3.start()

p1.join()
p2.join()
p3.join()
