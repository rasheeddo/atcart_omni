
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray, Int8, Int32MultiArray


class CartControl:

	def __init__(self):

		rospy.init_node('atcart_omni_control_node', anonymous=True)

		### Robot parameters ###
		#### cart dimension
		self.L = 0.590 ## from front to rear wheels
		self.T = 0.530 ## from left to right wheels
		self.R_wheel = 0.15
		#### servo attach dimensions
		self.off_ang = np.radians(14.48)
		self.off_ang2 = np.radians(75.52)
		self.AB = 61.0
		self.BC = 180.0
		self.CD = 55.0
		self.AD = 180.0
		#### Kinematics
		self.Vx_max = 2.0
		self.Wz_max = 4.0 #3.0
		self.smallest_R = self.Vx_max/self.Wz_max
		self.biggest_R = self.Vx_max/0.2
		self.max_alp = np.arctan((self.L/2)/self.smallest_R)
		self.min_alp = np.arctan((self.L/2)/self.biggest_R)

		#### Others
		self.str = 1024
		self.thr = 1024
		self.cart_mode = 1

		self.sbus_min = 384.0
		self.sbus_mid = 1024.0
		self.sbus_max = 1680.0
		self.sbus_min_db = 1000.0
		self.sbus_max_db = 1048.0

		## For UMOAB
		# self.pwm_min = 1080.0
		# self.pwm_mid = 1500.0
		# self.pwm_max = 1920.0

		## For PCA9685 JMOAB
		## each motor has different PWM ranges
		## we need to do eyes calibration to check min/mid/max PWM of -90/0/90 deg angle
		self.pwm_min = [1130.0, 1142.0, 1128.0, 1150]
		self.pwm_mid = [1575.0, 1580.0, 1575.0, 1575.0]
		self.pwm_max = [2015.0, 2025.0, 2015.0, 2060.0]

		self.servo_ang_min = -90.0
		self.servo_ang_max = 90.0

		self.opr_mode_ch = 150
		self.left_diag_ang = np.arctan(self.T/self.L)
		self.right_diag_ang = -self.left_diag_ang

		### PUB/SUB ###
		# self.servo_pub = rospy.Publisher("/umoab/servo", Int16MultiArray, queue_size=1)
		self.servo_pub = rospy.Publisher("/pca9685_pwm", Int16MultiArray, queue_size=10)
		self.servo_msg = Int16MultiArray()

		self.cart_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=1)
		self.cart_cmd_msg = Int32MultiArray()

		# rospy.Subscriber("/umoab/sbus_rc_ch", Int16MultiArray, self.sbus_rc_callback)
		rospy.Subscriber("/sbus_rc_ch", Int32MultiArray, self.sbus_rc_callback)
		rospy.Subscriber("/atcart_mode", Int8, self.cart_mode_callback)

		### Loop ###
		self.loop()

		rospy.spin()

	def sbus_rc_callback(self, msg):

		self.str = msg.data[0]
		self.thr = msg.data[1]
		self.opr_mode_ch = msg.data[9]

	def cart_mode_callback(self, msg):

		self.cart_mode = msg.data

	def wheelAng_to_servoAng(self, wheel_ang):

		## Four-bar linkage calculation, refer to diagram
		gamma = wheel_ang #np.radians(wheel_ang)
		mu = np.pi/2.0 - gamma
		beta = np.pi - mu - self.off_ang
		L = np.sqrt(self.CD**2 + self.AD**2 - (2*self.CD*self.AD*np.cos(beta)))
		alpha = np.arcsin((self.CD/L)*np.sin(beta))
		phi = np.arccos(-((self.BC**2 - L**2 - self.AB**2)/(2*L*self.AB)))
		theta = np.pi - self.off_ang2 - alpha - phi

		return np.degrees(theta)

	def servoAng_to_pwm(self, servo_ang, pwm_min, pwm_mid, pwm_max):

		if servo_ang > 0:
			pwm_out = int(self.map_with_limit(servo_ang, 0.0, self.servo_ang_max, pwm_mid, pwm_max))
		elif servo_ang < 0:
			pwm_out = int(self.map_with_limit(servo_ang, self.servo_ang_min, 0.0, pwm_min, pwm_mid))
		else:
			pwm_out = int(pwm_mid)

		return pwm_out 


	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		return out

	def cartWheelAng_to_pwm(self, alp_A, alp_B, alp_C, alp_D):

		front_left_servo_ang = self.wheelAng_to_servoAng(alp_A)
		front_right_servo_ang = self.wheelAng_to_servoAng(-alp_B)
		rear_left_servo_ang = self.wheelAng_to_servoAng(-alp_C)
		rear_right_servo_ang = self.wheelAng_to_servoAng(alp_D)

		front_left_servo_pwm = self.servoAng_to_pwm(front_left_servo_ang, self.pwm_min[0], self.pwm_mid[0], self.pwm_max[0])
		front_right_servo_pwm = self.servoAng_to_pwm(-front_right_servo_ang, self.pwm_min[1], self.pwm_mid[1], self.pwm_max[1])
		rear_left_servo_pwm = self.servoAng_to_pwm(-rear_left_servo_ang, self.pwm_min[2], self.pwm_mid[2], self.pwm_max[2])
		rear_right_servo_pwm = self.servoAng_to_pwm(rear_right_servo_ang, self.pwm_min[3], self.pwm_mid[3], self.pwm_max[3])

		return [front_left_servo_ang, front_right_servo_ang, rear_left_servo_ang, rear_right_servo_ang], [front_left_servo_pwm, front_right_servo_pwm, rear_left_servo_pwm, rear_right_servo_pwm]


	def loop(self):

		rate = rospy.Rate(20) #10

		Vx = 0.0
		Wz = 0.0
		alp_A = 0.0
		alp_B = 0.0
		alp_C = 0.0
		alp_B = 0.0
		V_left = 0.0
		V_right = 0.0
		WL = 0.0
		WR = 0.0
		RO = 0.0

		while not rospy.is_shutdown():

			Vx = self.map_with_limit(self.thr, self.sbus_min, self.sbus_max, -self.Vx_max, self.Vx_max)

			if 144 <= self.opr_mode_ch < 496:
				## Left diagonal
				alp_A = self.left_diag_ang 
				alp_B = self.left_diag_ang 
				alp_C = self.left_diag_ang 
				alp_D = self.left_diag_ang 

				servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True

			elif 496 <= self.opr_mode_ch < 848:
				## Right diagonal
				alp_A = self.right_diag_ang 
				alp_B = self.right_diag_ang 
				alp_C = self.right_diag_ang 
				alp_D = self.right_diag_ang 

				servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True

			elif 848 <= self.opr_mode_ch < 1200:
				## Skidding
				alp_A = -(np.pi/2 - self.left_diag_ang) 
				alp_B =  (np.pi/2 - self.left_diag_ang)
				alp_C =  (np.pi/2 - self.left_diag_ang) 
				alp_D = -(np.pi/2 - self.left_diag_ang)

				servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

				front_left_drive_sign = 1.0
				front_right_drive_sign = -1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = -1.0

				special_steer_flag = True

			elif 1200 <= self.opr_mode_ch < 1552:
				## side-way
				alp_A = np.radians(90.0) 
				alp_B = np.radians(-90.0) 
				alp_C = np.radians(-90.0) 
				alp_D = np.radians(90.0) 

				servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

				front_left_drive_sign = 1.0
				front_right_drive_sign = -1.0
				rear_left_drive_sign = -1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True
			else:
				## Double Ankermann steering mode

				special_steer_flag = False

				if self.str >= self.sbus_max_db:
					alp = self.map_with_limit(self.str, self.sbus_max_db, self.sbus_max, -self.min_alp, -self.max_alp)
					RO = (self.L/2)/np.tan(alp)
					# RO = self.map_with_limit(self.str, self.sbus_max_db, self.sbus_max, -self.biggest_R, -self.smallest_R)
					Wz = Vx/RO
				elif self.str <= self.sbus_min_db:
					alp = self.map_with_limit(self.str, self.sbus_min, self.sbus_min_db, self.max_alp, self.min_alp)
					RO = (self.L/2)/np.tan(alp)
					# RO = self.map_with_limit(self.str, self.sbus_min, self.sbus_min_db, self.smallest_R, self.biggest_R)
					Wz = Vx/RO
				else:
					alp = 0.0
					RO = 0.0
					Wz = 0.0

				str_stick_inDeadBand = self.sbus_min_db < self.str < self.sbus_max_db
				thr_stick_inDeadBand = self.sbus_min_db < self.thr < self.sbus_max_db

				if (not str_stick_inDeadBand) and (not thr_stick_inDeadBand):

					Vx_sign = abs(Vx)/Vx

					RA = np.sqrt((RO - (self.T/2.0))**2 + (self.L/2.0)**2)
					RB = np.sqrt((RO + (self.T/2.0))**2 + (self.L/2.0)**2)
					RC = RA
					RD = RB

					alp_A = np.arctan((self.L/2.0)/(RO-(self.T/2.0)))
					alp_B = np.arctan((self.L/2.0)/(RO+(self.T/2.0)))
					alp_C = -alp_A
					alp_D = -alp_B

					servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

					V_left = Vx_sign*abs(Wz)*RA
					V_right = Vx_sign*abs(Wz)*RB


					WL = V_left/self.R_wheel
					WR = V_right/self.R_wheel

				elif (not str_stick_inDeadBand):

					RA = np.sqrt((RO - (self.T/2.0))**2 + (self.L/2.0)**2)
					RB = np.sqrt((RO + (self.T/2.0))**2 + (self.L/2.0)**2)
					RC = RA
					RD = RB

					alp_A = np.arctan((self.L/2.0)/(RO-(self.T/2.0)))
					alp_B = np.arctan((self.L/2.0)/(RO+(self.T/2.0)))
					alp_C = -alp_A
					alp_D = -alp_B

					servo_ang, servo_pwm = self.cartWheelAng_to_pwm(alp_A, alp_B, alp_C, alp_D)

					V_left = 0.0
					V_right = 0.0

					WL = 0.0
					WR = 0.0

				else:

					Wz = 0.0
					RO = 0.0
					RA = 0.0
					RB = 0.0

					alp_A = 0.0
					alp_B = 0.0
					alp_C = 0.0
					alp_D = 0.0

					servo_ang = [0.0, 0.0, 0.0, 0.0]
					servo_pwm = [int(self.pwm_mid[0]), int(self.pwm_mid[1]), int(self.pwm_mid[2]), int(self.pwm_mid[3])]

					V_left = Vx
					V_right = Vx


					WL = V_left/self.R_wheel
					WR = V_right/self.R_wheel

			if special_steer_flag:
				front_left_speed_sbus = int(self.map_with_limit(front_left_drive_sign*Vx, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
				front_right_speed_sbus = int(self.map_with_limit(front_right_drive_sign*Vx, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
				rear_left_speed_sbus = int(self.map_with_limit(rear_left_drive_sign*Vx, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
				rear_right_speed_sbus = int(self.map_with_limit(rear_right_drive_sign*Vx, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
			else:
				front_left_speed_sbus = int(self.map_with_limit(V_left, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
				front_right_speed_sbus = int(self.map_with_limit(V_right, -self.Vx_max, self.Vx_max, self.sbus_min, self.sbus_max))
				rear_left_speed_sbus = front_left_speed_sbus
				rear_right_speed_sbus = front_right_speed_sbus


			front_left_servo_ang = servo_ang[0]
			front_right_servo_ang = servo_ang[1]
			rear_left_servo_ang = servo_ang[2]
			rear_right_servo_ang = servo_ang[3]

			front_left_servo_pwm = servo_pwm[0]
			front_right_servo_pwm = servo_pwm[1]
			rear_left_servo_pwm = servo_pwm[2]
			rear_right_servo_pwm = servo_pwm[3]
			

			self.servo_msg.data = [front_left_servo_pwm, front_right_servo_pwm, rear_left_servo_pwm, rear_right_servo_pwm]
			self.servo_pub.publish(self.servo_msg)

			self.cart_cmd_msg.data = [front_left_speed_sbus, front_right_speed_sbus, rear_left_speed_sbus, rear_right_speed_sbus]
			self.cart_cmd_pub.publish(self.cart_cmd_msg)


			print("Vx: {:.2f} | alpA: {:.2f} alpB: {:.2f} alpC: {:.2f} alpD: {:.2f} | FL_ang: {:.2f} FR_ang: {:.2f} RL_ang: {:.2f} RR_ang: {:.2f} | FL_pwm: {:d} FR_pwm: {:d} RL_pwm: {:d} RR_pwm: {:d}".format(\
				Vx,\
				np.degrees(alp_A), np.degrees(alp_B), np.degrees(alp_C), np.degrees(alp_D), \
				front_left_servo_ang, front_right_servo_ang, rear_left_servo_ang, rear_right_servo_ang, \
				front_left_servo_pwm, front_right_servo_pwm, rear_left_servo_pwm, rear_right_servo_pwm))


			rate.sleep()


if __name__ == "__main__":

	a = CartControl()