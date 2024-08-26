#draw
import cv2
def draw_rectangle(data, image, confidence_threshold=0.7, color=(0, 255, 0)): 
    confidence = float(data[4]) #정확도
    if confidence < confidence_threshold:
        return None

    xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
    label = int(data[5])
    
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 1) #frame에 xmax,ymax에서 xmin,ymin까지 사각형 그리기 (1픽셀로)
    
    width = xmax - xmin
    height = ymax - ymin
    
    return xmin, ymin, xmax, ymax, width, height #xmin, ymin, xmax, ymax, width, height 값  반환
