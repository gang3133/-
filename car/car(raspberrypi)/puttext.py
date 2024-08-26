import cv2
import draw
def draw_labels(image, xmin, ymin, ymax, y, in_30, out_30, WHITE):
    y_text = f'{y:.2f}' if y is not None else 'N/A'# y is not None else이면 y값을 소수 2자리까지 y_text반환
    cv2.putText(image, y_text, (xmin, ymax + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
    
    
    if y is not None and 50 < y < 85:  # y 값이 None이 아닌 경우에만 검사
        cv2.putText(image, f'{in_30}', (xmin - 20, ymin - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
        detect = 1
    else:
        cv2.putText(image, f'{out_30}', (xmin - 30, ymin - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)
        detect = 0
    return detect
