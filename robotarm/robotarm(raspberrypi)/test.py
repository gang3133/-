import cv2
import time
from ultralytics import YOLO  # ultralytics YOLO 라이브러리 임포트
from picamera2 import Picamera2  # picamera2 및 Preview 임포트
from bluetooth import *
import numpy as np
import math
import serial
from coordinate_transformer import first_transform_coordinates 
from coordinate_transformer import second_transform_coordinates
from inverse_kinematics_robot_arm import Robotic_arm_inverse_kinematics_calculation
from yellow_red_orange_check import is_frame_yellow
from yellow_red_orange_check import is_frame_red
from yellow_red_orange_check import is_frame_orange
import gpiozero

button = gpiozero.Button(18)
ser = serial.Serial('/dev/ttyAMA0', 9600)
initial_value_theta1_theta2_theta3 = [90.00003218077504, 90, 180]
socket = BluetoothSocket( RFCOMM )
socket.connect(("00:20:12:08:20:8D", 1))
print("bluetooth connected!")

l0 = 23.9
l1 = 28
l3 = 28
l4 = 20 
y_1 = 40

# 모델 초기화 및 로드
model = YOLO('/home/pi/best150_full_integer_quant_edgetpu.tflite', task='detect')

# PiCamera 설정
picam2_0 = Picamera2(camera_num=0)
picam2_1 = Picamera2(camera_num=1)
preview_config_0 = picam2_0.create_preview_configuration(main = {"size" : (800, 606), "format": 'XRGB8888'}, lores={"size" : (320, 240)}, display= 'lores')
picam2_0.configure(preview_config_0)
picam2_0.start()
preview_config_1 = picam2_1.create_preview_configuration(main = {"size" : (800, 606), "format": 'XRGB8888'}, lores={"size" : (320, 240)}, display= 'lores')
picam2_1.configure(preview_config_1)
picam2_1.start()

# FPS 계산을 위한 초기 설정
fps_counter = 0
start_time = time.time()


x1_transformed, z1_transformed = 0, 0
x2_transformed, z2_transformed = 0, 0
y_little = 0
catch_crops = 0
stop = 0
to_stop = 0
to_go = 0
wait = 0

undetected = 0
undetected_count = 0
undetected_ = 0
undetected_count_ = 0
cnt = 0

grap = b'I'
socket.send(grap)
time.sleep(3)
while True:
    if  button.is_pressed:
        grap = b'I'
        socket.send(grap)


        time.sleep(5) 
               
               
        theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(0, 52, 48)

        print("theta1 =",theta1)
        print("theta2 =",theta2)
        print("theta3 =",theta3)

        print("실제로 움직이는 theta1값 =",real_apply_theta1)
        print("실제로 움직이는 theta2값 =",real_apply_theta2)
        print("실제로 움직이는 theta3값 =",real_apply_theta3) 

        bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
        bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
        bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
        data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
        checksum = sum(data_to_send) 
        checksum = checksum & 0xFF
        checksum = (~checksum + 1) & 0xFF
        checksum = chr(checksum).encode('iso-8859-1')
        checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
        socket.send(checksum_data_to_send)

        time.sleep(5) 

        y_little = 0
        catch_crops = 0
        stop = 0
        to_stop = 0
        to_go = 0
        wait = 0

        undetected = 0
        undetected_count = 0
        undetected_ = 0
        undetected_count_ = 0
        cnt = 0
        
        
    else:
        ##camera0##
        request_0 = picam2_0.capture_request()  # 프레임 캡처 요청
        image_array_0 = picam2_0.capture_array("main")
        image_0 = request_0.make_image("main") 
        request_0.release()
        # 프레임에서 객체 탐지
        results = model(image_0)
        annotated_frame = image_array_0.copy()  # 프레임 복사본에 그리기

        # 프레임의 높이와 너비를 구합니다.
        height, width = annotated_frame.shape[:2]
        
        # 감지된 객체들의 바운딩 박스, 클래스, 신뢰도 정보 얻기
        boxes = results[0].boxes.xyxy.tolist()
        classes = results[0].boxes.cls.tolist()
        confidences = results[0].boxes.conf.tolist()
        names = results[0].names  # 클래스 이름 얻기

        # 감지된 객체들의 정보를 저장할 리스트 초기화
        detected_objects = []

        for box, cls, conf in zip(boxes, classes, confidences):
            x1, y1, x2, y2 = box[:4]
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            # 클래스 이름 얻기
            name = names[int(cls)]
            # 각 객체 정보 저장
            detected_objects.append((x_center, y_center, name, cls, conf, x1, y1))

        # class0 객체에 대해서만 오른쪽부터 번호를 매기기 위해 필터링
        detected_objects_class0 = [obj for obj in detected_objects if obj[3] == 0]
        detected_objects_class0.sort(key=lambda x: -x[0])  # x_center 기준으로 객체들을 오른쪽부터 정렬

        
                    
        ##camera1##
        request_1 = picam2_1.capture_request()  # 프레임 캡처 요청
        image_array_1 = picam2_1.capture_array("main")
        image_1 = request_1.make_image("main") 
        request_1.release()
        # 프레임에서 객체 탐지
        results_ = model(image_1)
        annotated_frame_ = image_array_1.copy()  # 프레임 복사본에 그리기

        # 프레임의 높이와 너비를 구합니다.
        height_, width_ = annotated_frame_.shape[:2]
        
        # 감지된 객체들의 바운딩 박스, 클래스, 신뢰도 정보 얻기
        boxes_ = results_[0].boxes.xyxy.tolist()
        classes_ = results_[0].boxes.cls.tolist()
        confidences_ = results_[0].boxes.conf.tolist()
        names_ = results_[0].names  # 클래스 이름 얻기

        # 감지된 객체들의 정보를 저장할 리스트 초기화
        detected_objects_ = []

        for box_, cls_, conf_ in zip(boxes_, classes_, confidences_):
            x1_, y1_, x2_, y2_ = box_[:4]
            x_center_ = (x1_ + x2_) / 2
            y_center_ = (y1_ + y2_) / 2
            # 클래스 이름 얻기
            name_ = names_[int(cls_)]
            # 각 객체 정보 저장
            detected_objects_.append((x_center_, y_center_, name_, cls_, conf_, x1_, y1_))

        # class0 객체에 대해서만 오른쪽부터 번호를 매기기 위해 필터링
        detected_objects_class0_ = [obj_ for obj_ in detected_objects_ if obj_[3] == 0]
        detected_objects_class0_.sort(key=lambda x: -x[0])  # x_center 기준으로 객체들을 오른쪽부터 정렬

        # class0 객체에 번호 매기기 및 좌표 계산
        if(y_little == 0):
            for i, (x_center, y_center, name, cls, conf, x1, y1) in enumerate(detected_objects_class0):
                x_transformed,z_transformed = first_transform_coordinates(x_center, y_center, width, height)
                label = f'{name} {i+1} xz_value: {x_transformed,z_transformed}' 
                print(label)
                cv2.putText(annotated_frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (203, 192, 255), 2)
        elif(y_little > 0):
            for i, (x_center_, y_center_, name_, cls_, conf_, x1_, y1_) in enumerate(detected_objects_class0_):
                x2_transformed , z2_transformed = second_transform_coordinates(x_center_, y_center_, width_, height_, x1_transformed, z1_transformed)
                label_ = f'{name_} {i+1} xz_value: {x2_transformed, z2_transformed}' 
                print(label_)
                cv2.putText(annotated_frame_, label_, (int(x1_), int(y1_)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (203, 192, 255), 2)
       
        # class1 객체는 감지만 camera0
        for x_center, y_center, name, cls, conf, x1, y1 in detected_objects:
            if cls == 1:  # class1 객체에 대해서 처리
                label = f'{name}'
                print(label)
                cv2.putText(annotated_frame, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 2)
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x_center+(x_center-x1)), int(y_center+(y_center-y1))), (100, 255, 100), 2)
            elif cls == 0: 
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x_center+(x_center-x1)), int(y_center+(y_center-y1))), (203, 192, 255), 2)
        # class1 객체는 감지만 camera1
        for x_center_, y_center_, name_, cls_, conf_, x1_, y1_ in detected_objects_:
            if cls_ == 1:  # class1 객체에 대해서 처리
                label_ = f'{name_}'
                print(label_)
                cv2.putText(annotated_frame_, label_, (int(x1_), int(y1_)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 2)
                cv2.rectangle(annotated_frame_, (int(x1_), int(y1_)), (int(x_center_+(x_center_-x1_)), int(y_center_+(y_center_-y1_))), (100, 255, 100), 2)
            elif cls_ == 0: 
                cv2.rectangle(annotated_frame_, (int(x1_), int(y1_)), (int(x_center_+(x_center_-x1_)), int(y_center_+(y_center_-y1_))), (203, 192, 255), 2)
               
        if detected_objects_class0:
            rightmost_object = detected_objects_class0[0]
            x_center, y_center = rightmost_object[0], rightmost_object[1]
            x_transformed, z_transformed = first_transform_coordinates(x_center, y_center, width, height)
            if(stop == 0):
                if(x_transformed < 5 and x_transformed > -15): #and stopped == 0): 멈춰있지 않을 경우에만 작동하도록 함 
                    #여기서 차체에 멈추라는 명령 보내기(로봇팔 라즈베리파이에서 차체 라즈베리파이에게 시리얼통신으로 보냄
                    #그리고 차체 라즈베리파이를 통해 차체를 움직이는 MCU에 명령보내야함
                    to_stop = 1
        if(to_stop == 1):
            stop = 1 
            time.sleep(3)
            to_stop = 0
            print("Sending: stop")
            ser.write(b'stop\n')
            
        if(to_go == 1):
            stop = 0
            time.sleep(3)
            to_go = 0
            print("Sending: go")
            ser.write(b'go\n')
         #여기서는 차체에서 멈춘 후 약3~5초뒤에 신호를 주고 그 신호가 오게 되면 아래 코드들이 작동하도록 조건문 설정
        if(catch_crops > 0):
            wait += 1

        if(catch_crops > 0 and wait > 5): #프레임에따라 다름 20프레임이면 10초뒤에 wait 200이됨
            yellow = is_frame_yellow(image_array_1, threshold=0.6)
            red = is_frame_red(image_array_1, threshold=0.6)
            orange = is_frame_orange(image_array_1, threshold=0.6)
            if(yellow or red or orange == 1):
                
                '''if(catch_crops == 2):
                    theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(x2_transformed, z2_transformed + 1, 50)

                    print("theta1 =",theta1)
                    print("theta2 =",theta2)
                    print("theta3 =",theta3)

                    print("실제로 움직이는 theta1값 =",real_apply_theta1)
                    print("실제로 움직이는 theta2값 =",real_apply_theta2)
                    print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                    bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                    bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                    bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                    data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                    checksum = sum(data_to_send) 
                    checksum = checksum & 0xFF
                    checksum = (~checksum + 1) & 0xFF
                    checksum = chr(checksum).encode('iso-8859-1')
                    checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                    socket.send(checksum_data_to_send)
                elif(catch_crops == 3):
                    theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(x2_transformed, z2_transformed + 1, 60)

                    print("theta1 =",theta1)
                    print("theta2 =",theta2)
                    print("theta3 =",theta3)

                    print("실제로 움직이는 theta1값 =",real_apply_theta1)
                    print("실제로 움직이는 theta2값 =",real_apply_theta2)
                    print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                    bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                    bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                    bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                    data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                    checksum = sum(data_to_send) 
                    checksum = checksum & 0xFF
                    checksum = (~checksum + 1) & 0xFF
                    checksum = chr(checksum).encode('iso-8859-1')
                    checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                    socket.send(checksum_data_to_send)
                time.sleep(3) '''
                
                grap = b'U'
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(1)
                socket.send(grap)
                time.sleep(5)
                

                
                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(25, 20, 25)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)

                time.sleep(5) 
                
                grap = b'I'
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(1)
                socket.send(grap)
                time.sleep(5)
                
                grap = b'I'
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(0.5)
                socket.send(grap)
                time.sleep(5) 
               
               
                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(0, 52, 48)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)

                time.sleep(5) 

                catch_crops = 0
                y_little = 0
                wait = 0
                undetected = 1

            else:
                y_little += 1
                wait = 0
        
        

        if(y_little == 0 and catch_crops == 0 and stop == 1):
            if detected_objects_class0:
                rightmost_object = detected_objects_class0[0]
                x_center, y_center = rightmost_object[0], rightmost_object[1]
                x1_transformed, z1_transformed = first_transform_coordinates(x_center, y_center, width, height)

                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(x1_transformed, z1_transformed, 30)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)

                catch_crops = 1


        elif( 8 > y_little > 1 and catch_crops == 1 and stop == 1):
            if detected_objects_class0_:
                rightmost_object_ = detected_objects_class0_[0]
                x_center_, y_center_ = rightmost_object_[0], rightmost_object_[1]
                x2_transformed, z2_transformed = second_transform_coordinates(x_center_, y_center_, width_, height_, x1_transformed, z1_transformed)

                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(x2_transformed, z2_transformed, 45)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)
                
                catch_crops = 2
                wait = 0
                
        elif(15 > y_little > 7 and catch_crops == 2 and stop == 1):
            if detected_objects_class0_:
                rightmost_object_ = detected_objects_class0_[0]
                x_center_, y_center_ = rightmost_object_[0], rightmost_object_[1]
                x2_transformed, z2_transformed = second_transform_coordinates(x_center_, y_center_, width_, height_, x2_transformed, z2_transformed)

                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(x2_transformed, z2_transformed, 55)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)
                
                catch_crops = 3
                wait = 0
            
        elif(y_little > 20 and catch_crops > 0 and stop == 0):
                grap = b'I'
                socket.send(grap)


                time.sleep(5) 
               
               
                theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3 = Robotic_arm_inverse_kinematics_calculation(0, 52, 48)

                print("theta1 =",theta1)
                print("theta2 =",theta2)
                print("theta3 =",theta3)

                print("실제로 움직이는 theta1값 =",real_apply_theta1)
                print("실제로 움직이는 theta2값 =",real_apply_theta2)
                print("실제로 움직이는 theta3값 =",real_apply_theta3) 

                bytes_theta1 = real_apply_theta1.encode('iso-8859-1')
                bytes_theta2 = real_apply_theta2.encode('iso-8859-1')
                bytes_theta3 = real_apply_theta3.encode('iso-8859-1')
                data_to_send =  bytes_theta3 + bytes_theta2 + bytes_theta1
                checksum = sum(data_to_send) 
                checksum = checksum & 0xFF
                checksum = (~checksum + 1) & 0xFF
                checksum = chr(checksum).encode('iso-8859-1')
                checksum_data_to_send = bytes_theta3 + bytes_theta2 + bytes_theta1 + b':' + checksum + b'E'
                socket.send(checksum_data_to_send)

                time.sleep(5) 

                catch_crops = 0
                y_little = 0
                wait = 0
                
        elif y_little > 20 and catch_crops > 0 and stop == 1 and not detected_objects_class0_:
            undetected_ = 1 

        if undetected == 1:
            if not detected_objects_class0:
                undetected_count = undetected_count + 1 
            if(cnt == 3):
                if undetected_count == 3:
                    to_go = 1
                    undetected_count = 0
                    undetected = 0
                    cnt = 0
                else:
                    undetected_count = 0
                    undetected = 0
                    cnt = 0
            cnt = cnt + 1
        if undetected_ == 1:
            if not detected_objects_class0_:
                undetected_count_ = undetected_count_ + 1 
            if(cnt == 3):
                if undetected_count_ == 3:
                    stop = 0
                    undetected_count_ = 0
                    undetected_ = 0
                    cnt = 0
                else:
                    undetected_count_ = 0
                    undetected_ = 0
                    cnt = 0
            cnt = cnt + 1
                
        # FPS 계산
        fps_counter += 1
        
        current_time = time.time()
        if current_time - start_time >= 1:
            fps = fps_counter / (current_time - start_time)
            fps_text = f'FPS: {fps:.2f}'
            start_time = current_time
            fps_counter = 0

        # FPS 텍스트 업데이트
        cv2.putText(annotated_frame, fps_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv2.putText(annotated_frame_, fps_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        # 결과 이미지 표시
        cv2.imshow('YOLO Object Detection', annotated_frame)
        cv2.imshow('YOLO Object Detection_', annotated_frame_)

        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 자원 해제
socket.close()
cap.release()
cv2.destroyAllWindows()



