#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped

pos_prev_x = 0
pos_prev_y = 0
prev_heading = 0
time_prev = 0


def callback_odometry(data):
    
    global pos_prev_x,pos_prev_y,pos_prev_heading,time_prev
    
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    orientation_q = data.transform.rotation
    heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    
    time_now = rospy.get_time()
    
    v_measured = (((pos_x - pos_prev_x)**2 + (pos_y - pos_prev_y)**2)**0.5)/(time_now - time_prev)
    omega_measured = (heading - prev_heading)/(time_now - time_prev)  
    
    print("Measured Linear Velocity:{}".format(v_measured))
    print("Measured Angular Velocity:{}".format(omega_measured))
    
    pos_prev_x = pos_x
    pos_prev_y = pos_y
    prev_heading = heading
    time_prev = time_now

if __name__ == '__main__':
    try:
        rospy.init_node('Calibration_checker', anonymous=True)
        #rospy.Subscriber('/pwm', PwmInput, callback)
        print("Calibration check")
        rospy.Subscriber('/vicon/fb5_8/fb5_8/', TransformStamped, callback_odometry)
        #rospy.Subscriber('/cmd_vel', Twist, Vel_to_PWM)
        #pub_pwm = rospy.Publisher('/pwm',PwmInput,queue_size=1)
        #start_time = rospy.get_time()
        #rospy.Service('vel_to_PWM', VelToPWM, Vel_to_PWM)


        rate = rospy.Rate(50) #Since bot sends data at 25hz the publisher will be forced to slow down
        while not rospy.is_shutdown():
            #print("Checking inside loop")
            #print("xyz")
            #encoderOut()
            #Prox_IR()
            #sharp_ir()
            #White_sense()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
