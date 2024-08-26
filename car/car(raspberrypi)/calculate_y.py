def calculate_y1(width, height):
    y = None
    if 260 <= width <= 270 and 370 <= height <= 420:  # 35-40
        y = 40 + (35 - 40) * (width - 260) / (270 - 260)
    elif 210 <= width < 260 and 290 <= height < 370:  # 40-50
        y = 50 + (40 - 50) * (width - 210) / (260 - 210)
    elif 170 <= width < 210 and 240 <= height < 290:  # 50-60
        y = 60 + (50 - 60) * (width - 175) / (200 - 175)
    elif 140 <= width < 170 and 210 <= height < 240:  # 60-70
        y = 70 + (60 - 70) * (width - 140) / (170 - 140)
    elif 20 <= width < 140 and 60 <= height < 210:  # 70-100
        y = 100 + (70 - 100) * (width - 20) / (140 - 20)      
    
    return y
