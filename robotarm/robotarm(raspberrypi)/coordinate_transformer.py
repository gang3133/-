
def first_transform_coordinates(x_pixel, y_pixel, width, height):
    """
    화면 중심을 (0,0)으로 설정하고, x축이 -20에서 20, y축이 10에서 40까지 되도록 좌표를 변환합니다.

    Parameters:
    x_pixel (int): 원본 x 좌표 (픽셀)
    y_pixel (int): 원본 y 좌표 (픽셀)
    width (int): 화면 너비 (픽셀)
    height (int): 화면 높이 (픽셀)

    Returns:
    tuple: 변환된 x, z 좌표
    """

    normalized_x = x_pixel / width
    normalized_y = y_pixel / height
    
    # x축은 0에서 1 사이의 값을 -20에서 20으로 변환
    x_transformed = (normalized_x * 53) - 26.5
    # y축은 0에서 1 사이의 값을 10에서 40으로 변환
    z_transformed = 55 - (normalized_y * 43)

    return int(x_transformed), int(z_transformed)

def second_transform_coordinates(x_pixel, y_pixel, width, height, second_x, second_z):
   
    normalized_x = x_pixel / width
    normalized_y = y_pixel / height
    
    # 정규화된 좌표를 새로운 범위로 변환 (여기서는 x축 범위를 20, y축(실제로는 z축에 해당) 범위를 15로 함)
    # 중앙을 0, 0으로 설정하고 양쪽 끝을 각각 -10~10, -7.5~7.5로 설정하기 위해
    x_transformed = (normalized_x - 0.5) * 10
    z_transformed = (0.5 - normalized_y) * 13

    # 주어진 중심값을 기준으로 최종 좌표 조정
    x_transformed += second_x 
    z_transformed += second_z

    return int(x_transformed), int(z_transformed)
