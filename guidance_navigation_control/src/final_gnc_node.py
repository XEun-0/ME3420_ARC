#! /usr/bin/env python
import rospy
from guidance_navigation_control.msg import task_desiredAction
from guidance_navigation_control.msg import controlCommand
from guidance_navigation_control.msg import sensorInfo_actuatorStatus
from Subscriber import Subscribe_to
from time import sleep

rospy.init_node('GNC')
gnc_pub = rospy.Publisher('controlCommand', controlCommand, queue_size=10)


class Guidance_Navigation_Control():
	def __init__(self):
		print("starting")
		self.smach_sub = Subscribe_to('task_desiredAction')
		self.sensors_sub = Subscribe_to('sensorInfo_actuatorStatus')
		self.setpoints = controlCommand()


	def data_received(self):
		if (self.smach_sub.was_data_sent() and self.sensors_sub.was_data_sent()):
			return True
		return False


	def main_loop(self):
		print("looping")
		if self.smach_sub.was_data_sent():
			print("data_updated")
			self.smach_data = self.smach_sub.get_data()
			self.sensors_data = self.sensors_sub.get_data()
			if (self.smach_data.bumpIntoBuoy):
				self.bumpIntoBuoy()
			self.update_setpoints()
			gnc_pub.publish(self.setpoints)


	def update_setpoints(self):
		print("updating")
		self.setpoints.yaw_set = self.smach_data.yaw_set + self.sensors_data.yaw_current
		self.setpoints.pitch_set = self.smach_data.pitch_set + self.sensors_data.pitch_current
		self.setpoints.roll_set = self.smach_data.roll_set + self.sensors_data.roll_current
		self.setpoints.depth_set = self.smach_data.depth_set + self.sensors_data.depth_current
		self.setpoints.distance_set = self.smach_data.distance_set


	def bumpIntoBuoy(self):
		print("bumping")
		self.setpoints.pitch_set = 100
		self.setpoints.roll_set = 45
		gnc_pub.publish(self.setpoints)
		sleep(2.5)


def main_loop():
	GNC = Guidance_Navigation_Control()
	sleep(1)
	while not (GNC.data_received()):
		sleep(0.1)
		print "no data"
	print "yes data"
	while not rospy.is_shutdown():
		sleep(0.5)
		GNC.main_loop()
	print("exiting")
	exit()


if __name__ == '__main__':
	main_loop()
