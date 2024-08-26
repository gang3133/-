import cv2
def rectangle(data,image):
    CONFIDENCE_THRESHOLD = 0.7
    GREEN = (0, 255, 0)
    confidence = float(data[4]) #검출결과의 신뢰도를 confidence에 저장
    if confidence < CONFIDENCE_THRESHOLD: 
        return None

    xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
    label = int(data[5])
    
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), GREEN, 1)#사각형 그리기 feame에 (xmin, ymin)에서 (xmax, ymax)까지 1픽셀로
    # 사각형 크기 계산
    
    return xmax,xmin,ymax,ymin
