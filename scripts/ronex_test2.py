#!/usr/bin/env python

import rospy
from sr_ronex_msgs.msg import PWM

def flashLED(topic):
	pwm_period = 320
	pwm_on_time_0 = pwm_period
	pwm_on_time_1 = 0
	pub = rospy.Publisher( topic, PWM )
	while not rospy.is_shutdown():
		pwm_on_time_0 -= 10
		if pwm_on_time_0 < 0:
			pwm_on_time_0 = pwm_period
		pwm = PWM()
		pwm.pwm_period = pwm_period
		pwm.pwm_on_time_0 = pwm_on_time_0
		pwm.pwm_on_time_1 = pwm_on_time_1
		pub.publish( pwm )
		rospy.sleep( 0.01 )
#--------------------------------------------------------------------------------

if __name__ == "__main__":
	rospy.init_node('sr_ronex_flash_LED_with_PWM')
	topic = "/ronex/general_io/12/command/pwm/1"
	try:
		flashLED(topic)
	except rospy.ROSInterruptException:
		pass
