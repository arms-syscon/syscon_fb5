#!/usr/bin/env python3
import rospy
import rospkg

from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from syscon_fb5.msg import PwmInput
from std_msgs.msg import String
dirname = rospkg.RosPack().get_path('syscon_fb5')
bot_name = rospy.get_namespace()

def heading_from_quaternion(x, y, z, w):
    ang_1 = 2 * (w * z + x * y)
    ang_2 = 1 - 2 * (y ** 2 + z ** 2)
    return math.atan2(ang_1, ang_2) % (2 * math.pi)

class CALIBRATE:
	def __init__(self):
		self.iterations = 3
		self.intervals = 5
		self.pwms = range(100, 256, self.intervals)
		self.output_folder = dirname + '/calibration_files'
		self.pos_x = 0.
		self.pos_y = 0.
		self.heading = 0.
		self.current_time = 0.
		self.run_duration = 5.
		self.output_file = open(self.output_folder + '/temp.csv', 'w+')
		rospy.Subscriber('/vicon/{}/{}/'.format(bot_name, bot_name), TransformStamped, self.callback_odom)
		#rospy.Subscriber('/test_1', String, self.callback_test)
		print("init")
		self.execute_pwms()

	def callback_test(self, data):
		print(data.data)
	

	def callback_odom(self, data):
		self.current_time = data.header.stamp.secs
		self.pos_x = data.transform.translation.x
		self.pos_y = data.transform.translation.y
		orientation_q = data.transform.rotation
		self.heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
		print("callback writing")
		#print(" X:{}".format(self.pos_x))
		#print(" Y:{}".format(self.pos_y))
		#print(" theta:{}".format(self.heading))
		try:
			
			self.output_file.write('{}, {}, {}, {}\n'.format(self.current_time, self.pos_x, self.pos_y, self.heading))
			print("writting to {}".format(self.output_file.name))
		except:
			print('No Output File Found!')
			pass

	def execute_pwms(self):
		# print("Here")
		self.output_file.close()
		r =rospy.Rate(100)
		for pwm in self.pwms:
			for run_id in range(self.iterations):
				start = rospy.get_time()
				self.output_file = open(self.output_folder + '/pos_{}_forward_{}.csv'.format(pwm, run_id), 'w+')
				while rospy.get_time() - start < self.run_duration:
					pwm_msg = PwmInput()
					pwm_msg.rightInput = pwm
					pwm_msg.leftInput = pwm
					pub_pwm.publish(pwm_msg)
					# rospy.sleep(0.5)
					r.sleep()
				self.output_file.close()

				pwm_msg.rightInput = 0
				pwm_msg.leftInput = 0
				pub_pwm.publish(pwm_msg)
				rospy.sleep(0.5)

				start = rospy.get_time()
				self.output_file = open(self.output_folder + '/pos_{}_backward_{}.csv'.format(pwm, run_id), 'w+')
				while rospy.get_time() - start < self.run_duration:
					pwm_msg = PwmInput()
					pwm_msg.rightInput = -pwm
					pwm_msg.leftInput = -pwm
					pub_pwm.publish(pwm_msg)
					r.sleep()
					# rospy.sleep(0.05)
				self.output_file.close()

				print('PWM input: {}, run_id: {}'.format(pwm,run_id))
				print(self.pos_x)
				pwm_msg.rightInput = 0
				pwm_msg.leftInput = 0
				pub_pwm.publish(pwm_msg)
				rospy.sleep(0.5)

if __name__ == '__main__':
	try:
		rospy.init_node('auto_calibrate', anonymous=True)
		pub_pwm = rospy.Publisher('pwm', PwmInput, queue_size=2)
		# print("HERE TOO")
		s = CALIBRATE()

	except rospy.ROSInterruptException:
		pass
