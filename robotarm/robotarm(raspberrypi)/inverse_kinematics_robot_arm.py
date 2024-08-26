import math

initial_value_theta1_theta2_theta3 = [90.00003218077504, 90, 180]
l0 = 23.9
l1 = 28
l3 = 28
l4 = 20 
#y_1 = 40

def apply_theta_M(apply_theta):
    i = math.ceil(abs(apply_theta) * 16)
    a = i // 1000
    b = i % 1000 // 100
    c = i % 1000 % 100 // 10 
    d = i % 100 % 10
    real_apply_theta = '-' + f'{a}' + f'{b}' + f'{c}' + f'{d}'

    return real_apply_theta
        
def apply_theta_P(apply_theta):
    i = math.ceil(abs(apply_theta) * 16)
    a = i // 1000
    b = i % 1000 // 100
    c = i % 1000 % 100 // 10 
    d = i % 100 % 10
    real_apply_theta = '+' + f'{a}' + f'{b}' + f'{c}' + f'{d}'

    return real_apply_theta      

def Robotic_arm_inverse_kinematics_calculation(x_transformed, z_transformed, y_1):
        if(int(x_transformed) == 0):
            x = int(x_transformed)
            y = int(y_1) - l4
        elif(int(x_transformed) == 48) or (int(x_transformed) == -48):
            x = int(x_transformed) - l4
            y = int(y_1) 
        else:
            x = int(x_transformed) - (l4 * int(x_transformed) / (math.sqrt(math.pow(int(x_transformed),2) + math.pow(int(y_1),2))))
            y = int(y_1) - (l4 * int(y_1) / (math.sqrt(math.pow(int(x_transformed),2) + math.pow(int(y_1),2))))

        d = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow((z_transformed - l0),2))

        D_ = (math.pow(l1,2) + math.pow(l3,2) - math.pow(d,2)) / (2 * l1 * l3)
        if(D_>=0):
            theta3_theta2 = math.atan2(math.sqrt(1 - math.pow(D_,2)), D_) * 57.2958
        else:
            theta3_theta2 = math.atan2((math.sqrt(1 - math.pow(D_,2))), D_) * 57.2958

        alpha = math.atan2((z_transformed - l0), math.sqrt(math.pow(x,2) + math.pow(y,2))) * 57.2958

        E = (math.pow(l1,2) + math.pow(d,2) - math.pow(l3,2)) / (2 * d * l1)

        beta = math.atan2(math.sqrt(1 - math.pow(E,2)), E) * 57.2958

        if(alpha + beta > 0):
            theta2 = alpha + beta
        else: 
            theta2 = alpha - beta
        theta3 = theta2 + theta3_theta2
        if(theta3 < 0):
            theta3 = 180 + theta3

        theta1 = math.atan2(int(y),int(x)) * 57.2958

        
        if(initial_value_theta1_theta2_theta3[0] >= theta1):
            apply_theta1 = initial_value_theta1_theta2_theta3[0]-theta1
            real_apply_theta1 = apply_theta_P(apply_theta1)
        else:
            apply_theta1 = theta1 - initial_value_theta1_theta2_theta3[0]
            real_apply_theta1 = apply_theta_M(apply_theta1)

        if(initial_value_theta1_theta2_theta3[1] >= theta2):
            apply_theta2 = initial_value_theta1_theta2_theta3[1]-theta2
            real_apply_theta2 = apply_theta_M(apply_theta2)
        else:
            apply_theta2 = theta2 - initial_value_theta1_theta2_theta3[1]
            real_apply_theta2 = apply_theta_P(apply_theta2)

        if(initial_value_theta1_theta2_theta3[2] >= theta3):
            apply_theta3 = initial_value_theta1_theta2_theta3[2]-theta3
            real_apply_theta3 = apply_theta_P(apply_theta3)
        else:
            apply_theta3 = theta3 - initial_value_theta1_theta2_theta3[2]
            real_apply_theta3 = apply_theta_M(apply_theta3)

        initial_value_theta1_theta2_theta3[0:3] = [theta1, theta2, theta3]
        
        return theta1, theta2, theta3, real_apply_theta1, real_apply_theta2, real_apply_theta3
