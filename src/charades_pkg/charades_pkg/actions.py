import rospy
from geometry_.msg import Twist

#NYC Subway

def snake_motion():
	rospy.init_node('snake_motion',anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	twist = Twist()
	for i in range(20):
		twist.linear.x = 0.3
		twist.angular.z = 0.5 if i % 2 == 0 else -0.5
		pub.publish(twist)
		rospy.sleep(0.5)
	print("Snake motion completed!")

def hurricane_kick_motion():
	rospy.init_node('hurricane_kick_motion'), anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	sound_client = SoundClient()
	twist = Twist()

	for _ in range(3):
		twist.angular.z = 2.0
		pub.publish(twist)
		rospy.sleep(0.5)
	twist.angular.z = 0.0
	twist.linear.x = 0.8
	pub.publish(twist)
	sound_client.playWave('/path_to_sounds/kick_impact.wav')
	rospy.sleep(1)

	print("Hurricane Kick Completed!")


def crab_motion():
	rospy.init_node('crab_motion', anonymous = True)
	pub = rospy.Publisher('/cmd_vel',Twist, queue_size =10)
	twist = Twist()
	for _ in range(5):
		twist.linear.y = 0.2
		pub.publish(twist)
		rospy.sleep(1)
		twist.linear.y = -0.2
		pub.publish(twist)
		rospy.sleep(1)
