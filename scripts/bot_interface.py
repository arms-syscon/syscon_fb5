#!/usr/bin/env python3
import rospy
from serial import Serial
# import time
import struct
from syscon_fb5.msg import PwmInput,EncoderData, Proximity_IR, WhiteLine_Sensor, Sharp_IR
#from syscon_fb5.srv import VelToPWM, VelToPWMResponse
from syscon_fb5.srv import VelToPWM
import yaml
import numpy as np
import rospkg
from geometry_msgs.msg import Twist

ser=Serial('/dev/ttyUSB0',9600)
ser.flushInput()
ser.flushOutput()
ser.write(bytes('8', 'utf-8'))
bytes_A = bytes('A', 'utf-8')

dirname = rospkg.RosPack().get_path('syscon_fb5')
with open(dirname + '/calibration.yaml', 'r') as f:
	config = yaml.load(f, yaml.FullLoader)


def callback(data):

	# ~ is \x7E
	ser.write(bytes_A)
	rightPWM = data.rightInput
	leftPWM = data.leftInput
	print(data)

	print("LeftPWM: {}, RightPWM: {}".format(leftPWM, rightPWM))

	#Ensuring the wheels are set to rotate in the correct direction for each scenario.
	if (rightPWM>=0) and (leftPWM>=0):
		ser.write(bytes('8', 'utf-8'))
	elif (rightPWM<0) and (leftPWM>=0):
		ser.write(bytes('6', 'utf-8'))
	elif (rightPWM>=0) and (leftPWM<0):
		ser.write(bytes('4', 'utf-8'))
	elif (rightPWM<0) and (leftPWM<0):
		ser.write(bytes('2', 'utf-8'))
	else:
		ser.write(bytes('5', 'utf-8')) # Should never happen

	#The C code on the bot accepts left motor velocity first.
	ser.write(struct.pack('>B',abs(leftPWM)))
	ser.write(struct.pack('>B',abs(rightPWM)))

#def encoderOut():
   # ser.write(bytes_A)
   # encoder = EncoderData()
    # encoder.stamp = 0
    # encoder.encoderR = 0
    # encoder.encoderL = 0
    # encoder.chksum = False
    #enc = ser.read()

    #if enc == bytes_A:
        #encoder.stamp = rospy.get_time() - start_time
        #encoderRightH = ser.read()
        #encoderRightL = ser.read()
        #encoderLeftH = ser.read()
        #encoderLeftL = ser.read()
        #chksum = ser.read()
        #encoder.encoderR = ord(encoderRightH) * 256 + ord(encoderRightL)
        #encoder.encoderL = ord(encoderLeftH) * 256 + ord(encoderLeftL)
        #sum = ord(encoderRightH) + ord(encoderRightL) + ord(encoderLeftH) + ord(encoderLeftL)
        #Byte2 and byte3 are the higher significant and lower significant bits respectively
        #encoder.chksum = ord(chksum) == sum%256
        #rospy.loginfo(encoder)
        #pub_encoder.publish(encoder)
        # sharpH = ser.read()
        # sharpL = ser.read()
        # print(ord(sharpH) * 256 + ord(sharpL))

def wheel_velocities(vel, ang_vel, wheel_base=0.18):
	right_vel = vel + wheel_base*ang_vel/2
	left_vel = 2*vel - right_vel
	return left_vel, right_vel

def vel_to_pwm(cmd_vel):
	print("called")
# def Vel_to_PWM(linear_vel,angular_vel,bot_id):
	linear_vel = cmd_vel.linear.x
	angular_vel = cmd_vel.angular.z
	#bot_id = cmd_vel.bot_id
	left_vel, right_vel = wheel_velocities(linear_vel,angular_vel)
    # if not bot_id == config['bot_id']:
        # return "Wrong bot"
	#if linear_vel > config['max_lin_vel_forward']:
		#linear_vel = config['max_lin_vel_forward']
	#elif linear_vel < config['max_lin_vel_backward']:
		#linear_vel = config['max_lin_vel_backward']
	#if angular_vel > config['max_ang_vel']:
		#angular_vel = config['max_ang_vel']
	#elif angular_vel < config['min_ang_vel']:
		#angular_vel = config["min_ang_vel"]

    #pwm for left wheel
	if left_vel > 0:
		w = config['pwm_left_forward'].split(" ")
		w = np.array(list(map(float,w)))
		leftpwm = np.array([left_vel**2,left_vel,1])
		leftpwm = int(leftpwm.dot(w))
		print(leftpwm)
		if leftpwm > 255:
			leftpwm = 255
	else:
		w = config['pwm_left_backward'].split(" ")
		w = np.array(list(map(float,w)))
		leftpwm = np.array([left_vel**2,left_vel,1])
		leftpwm = int(leftpwm.dot(w))
		if leftpwm < -255:
			leftpwm = -255


    #pwm for right wheel
	if right_vel > 0:
		w = config['pwm_right_forward'].split(" ")
		w = np.array(list(map(float,w)))
		rightpwm = np.array([right_vel**2,right_vel,1])
		rightpwm = int(rightpwm.dot(w))
		if rightpwm > 255:
			rightpwm = 255
	else:
		w = config['pwm_right_backward'].split(" ")
		w = np.array(list(map(float,w)))
		rightpwm = np.array([right_vel**2,right_vel,1])
		rightpwm = int(rightpwm.dot(w))
		if rightpwm < -255:
			rightpwm = -255
	pwm = PwmInput(rightpwm,leftpwm)
	pub_pwm.publish(pwm)
    # return VelToPWMResponse(rightpwm , leftpwm)
	return leftpwm,rightpwm

#def sharp_ir():
    #sharp_ir = Sharp_IR()
    #sharph = ser.read()
    #sharpl = ser.read()
    #sharp = ord(sharph)*256 + ord(sharpl)
    #sharp_ir.obstacle_distance = sharp
    #pub_sharp.publish(sharp_ir)

    
#def Prox_IR():
    #"returns true if there is a object within range of 10 cm"
    #prox_ir = Proximity_IR()
    #for i in range(4):
        #proxh = ser.read()
        #proxl = ser.read()
        #prox = ord(proxh)*256 + ord(proxl)
        #if prox < 145:
            #prox_ir.obstacle[i] = True
        #else:
            #prox_ir.obstacle[i] = False

    #pub_prox_ir.publish(prox_ir)

#def White_sense():
    #Wl_sens = WhiteLine_Sensor()
    #for i in range(3):
        #WL_irh = ser.read()
        #Wl_irl = ser.read()
        #Wl = ord(WL_irh)*256 + ord(Wl_irl)  
        #if Wl > 40:
            #Wl_sens.Whiteline[i] = False
        #else:
            #Wl_sens.Whiteline[i] = True

    #pub_wl.publish(Wl_sens)





if __name__ == '__main__':
	try:
		rospy.init_node('bot_interface', anonymous=True)
		rospy.Subscriber('pwm', PwmInput, callback)
		#print("Hello check check")
		rospy.Subscriber('cmd_vel', Twist, vel_to_pwm)
		#pub_encoder = rospy.Publisher('encoder', EncoderData, queue_size=1)
		#pub_prox_ir = rospy.Publisher('prox_ir',Proximity_IR, queue_size=1)
		#pub_sharp = rospy.Publisher('sharp_ir', Sharp_IR, queue_size=1)
		#pub_wl = rospy.Publisher('wl',WhiteLine_Sensor , queue_size=1)
		pub_pwm = rospy.Publisher('pwm',PwmInput,queue_size=1)
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
