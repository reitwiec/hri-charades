import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from time import sleep
import time
from cv_bridge import CvBridge
import cv2

# Actions available
ACTIONS = {
    '2': 'wavy',
    '3': 'kick',
    '16': 'sub',
    '12': 'mario',
    '17': 'pika',
    '18': 'sonic',
    '15': 'rocket'
}
ACTIONS_old = {
    'Tornado': 'spin',
    'Car': 'forward_backward',
    'Confused': 'turn_left_right',
    'Robot': 'slow_beep',
    'Snake': 'wavy',
    'Hurricane Kick': 'kick',
    'Subway Surfers': 'sub',
    'Mario Kart': 'mario',
    'Pikachu': 'pika',
    'Rocket': 'rocket'
}

"""
State mapping:
0: idle
1: perform action
2: action completed
"""

class RobotCharades(Node):
    def __init__(self):
        super().__init__('robot_charades')
        
        # Initialize publisher for moving the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sound_pub = self.create_publisher(String, '/sound', 10)

        self.actions_state = {
            '2': False,
            '3': False,
            '16': False,
            '12': False,
            '17': False,
            '18': False,
            '15': False
        }
        
        # Initialize game state
        self.state = 0
        self.selected_action = None
        self.remaining_guesses = 2
        self.guess_timer = None
        self.round = 0

        self.audio_state = {'welcome': False, 'got_prompt': False, 'correct': False, 'wrong_guess':False, 'no_guess': False, 'start_guess': False, 'next': False}

        self.bridge = CvBridge()
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        # Timer to periodically check for user input
        self.timer = self.create_timer(1.0, self.game_loop)
    
    def image_callback(self, msg):
        print('calling')
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.scan_qr_code(frame)
        except Exception as e:
            self.get_logger().warn(f"Failed to convert ROS Image to OpenCV: {str(e)}")

    def scan_qr_code(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qcd = cv2.QRCodeDetector()
        retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(gray)
        self.destroy_subscription(self.image_sub)
        if retval:
            print(decoded_info)
            for obj in decoded_info:
                split = obj.split(":")
                if len(split)>1:
                    if split[1] in ACTIONS:
                        if self.state == 0:
                            self.selected_action = split[1]
                            self.get_logger().info("ACTION RECEIVED!!")
                            if not self.audio_state["got_prompt"]:
                                os.system("mpg123 ./hri-audio/gotprompt.mp3")
                                self.audio_state["got_prompt"] = True
                            #sleep(5)
                            self.state = 1
                        elif self.state == 2:
                            if self.selected_action == split[1]:
                                self.get_logger().info("You guessed it right!")
                                if not self.audio_state["correct"]:
                                    os.system("mpg123 ./hri-audio/yougotit.mp3")
                                    self.audio_state["correct"] = True
                                self.reset()
                                # Handle correct guess
                            else:
                                self.remaining_guesses -= 1
                                self.get_logger().info(f"Wrong guess. You have {self.remaining_guesses} guesses remaining.")
                                if not self.audio_state["wrong_guess"]:
                                    os.system("mpg123 ./hri-audio/oneguess.mp3")
                                    self.audio_state["wrong_guess"] = True
                                    sleep(2)
                    else:
                        if self.state == 2:
                            self.remaining_guesses -= 1
                            self.get_logger().info(f"Wrong guess. You have {self.remaining_guesses} guesses remaining.")
                            if not self.audio_state["wrong_guess"]:
                                os.system("mpg123 ./hri-audio/oneguess.mp3")
                                self.audio_state["wrong_guess"] = True
                                sleep(2)

        self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)

    def reset(self):
        # Your turn over

        self.round += 1
        if not self.audio_state["next"] and self.round != 5:
            os.system("mpg123 ./hri-audio/next.mp3")
            self.audio_state["next"] = True
        self.get_logger().info("Reset")
        self.state = 0
        #self.guess_timer.cancel()
        #self.guess_timer = None
        self.selected_action = None
        self.remaining_guesses = 2
        self.audio_state = {'welcome': False, 'got_prompt': False, 'correct': False, 'wrong_guess':False, 'no_guess': False, 'start_guess': False, 'next': False}

    def game_loop(self):
        if self.round == 7:
            up = input("loser ip")
            os.system("mpg123 ./hri-audio/meow\ meow.mp3")
            lp = input("winner ip")
            os.system("mpg123 ./hri-audio/happy-happy-happy-song.mp3")
            block = input("game over")
        if self.remaining_guesses <= 0:
            # reset game loop
            
            #self.destroy_subscription(self.image_sub)
            if not self.audio_state["no_guess"]:
                os.system("mpg123 ./hri-audio/noguess.mp3")
                self.audio_state["no_guess"] = True
            self.reset()
            #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
            return

        if self.state == 0:
            # Wait for user input (action selection)
            if not self.audio_state["welcome"]:
                #self.destroy_subscription(self.image_sub)
                if self.round == 0:
                    os.system("mpg123 ./hri-audio/welcome.mp3")
                    self.audio_state["welcome"] = True
                else:
                    os.system("mpg123 ./hri-audio/ready.mp3")
                    ip = input('type anything to continue')

                false_keys = [key for key, value in self.actions_state.items() if value is False]
                act = random.choice(false_keys)
                self.actions_state[act] = True
                self.selected_action = act
                if not self.audio_state["got_prompt"]:
                    os.system("mpg123 ./hri-audio/gotprompt.mp3")
                    self.audio_state["got_prompt"] = True
                self.state = 1
                
                #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        
        elif self.state == 1:
            self.perform_action(self.selected_action)
            self.state = 2
        
        elif self.state == 2:
            # After performing the action, proceed to guessing
            self.get_logger().info("Time to guess")
            if not self.audio_state["start_guess"]:
                #self.destroy_subscription(self.image_sub)
                os.system("mpg123 ./hri-audio/startguessing.mp3")
                self.audio_state["start_guess"] = True
            user_input = input('Enter your prompt code:')
            if self.selected_action == user_input:
                self.get_logger().info("You guessed it right!")
                if not self.audio_state["correct"]:
                    os.system("mpg123 ./hri-audio/yougotit.mp3")
                    self.audio_state["correct"] = True
                self.reset()
                # Handle correct guess
            else:
                self.remaining_guesses -= 1
                self.get_logger().info(f"Wrong guess. You have {self.remaining_guesses} guesses remaining.")
                if not self.audio_state["wrong_guess"]:
                    os.system("mpg123 ./hri-audio/oneguess.mp3")
                    self.audio_state["wrong_guess"] = True
                    sleep(2)
                #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)

            #if self.guess_timer is None:
                #print("setting timer")
                #self.guess_timer = self.create_timer(20.0, self.reset)

    def perform_action(self, action):
        """Perform the corresponding action based on the input."""
        if action == '2':
            self.get_logger().info("Performing Snake action!")
            self.snake_action()
        elif action == '12':
            self.get_logger().info("Mario")
            #self.mario_action()
            self.mario_donut()
        elif action == '16':
            self.get_logger().info("Subway surfers")
            self.subway()
        elif action == '3':
            self.get_logger().info("Hurricane Kick")
            self.hurricane()
        elif action == '15':
            self.get_logger().info('Rocket')
            self.rocket()
        elif action == '17':
            self.get_logger().info('Pikachu')
            self.pikachu()
        elif action == '18':
            self.get_logger().info('Sonic')
            self.sonic()

            # Implement Snake action logic here
    def sonic(self):
        twist = Twist()
        os.system("mpg123 ./hri-audio/sonic.mp3")
        for i in range(6):
            twist.angular.z = 20.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)
        for i in range(6):
            twist.linear.x = 20.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)
        os.system("mpg123 ./hri-audio/sonic.mp3")

    def pikachu(self):
        twist = Twist()
        os.system("mpg123 ./hri-audio/pika-pika.mp3")
        for i in range(6):
            twist.angular.z = 5.0 if i%3==0 else -5.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)
        os.system("mpg123 ./hri-audio/electric-shock.mp3")

    def rocket(self):
        twist = Twist()
        os.system("mpg123 ./hri-audio/rocket_countdown.mp3")
        for i in range(4):
            twist.linear.x = 20.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)
        os.system("mpg123 ./hri-audio/rocket_blast.mp3")

    def hurricane(self):
        twist = Twist()
        os.system("mpg123 ./hri-audio/wind.mp3")
        for i in range(10):
            twist.angular.z = 15.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)

    def snake_action(self):
        twist = Twist()

        os.system("mpg123 ./hri-audio/hiss.mp3")
        for i in range(5):
            twist.linear.x = 10.0
            twist.angular.z = 15.0 if i%2 == 0 else -15.0
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)
        os.system("mpg123 ./hri-audio/hiss.mp3")
        print("Snake motion completed!")
    
    def mario_donut(self):
        radius = 0.5
        center_radius = 0.2
        angular_velocity = 15.0

        linear_velocity = angular_velocity * (radius + center_radius)

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        os.system("mpg123 ./hri-audio/mk64_firstplace.mp3")

        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            sleep(0.5)

        os.system("mpg123 ./hri-audio/mk64_firstplace.mp3")

    def mario_action(self):
        twist = Twist()
        #self.destroy_subscription(self.image_sub)
        os.system("mpg122 ./hri-audio/mk64_firstplace.mp3")
        #self.audio_state["mariokart"] = True
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 9)
        for i in range(9):
            twist.linear.x = 0.7
            twist.angular.z = -1.3
            self.cmd_vel_pub.publish(twist)
            sleep(-1.5)
        #self.destroy_subscription(self.image_sub)
        os.system("mpg122 ./hri-audio/mk64_firstplace.mp3")
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 9)
        print("Mario motion completed")

    def movesub(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        twist = Twist()
        if angular_z != 0.0:
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
        for i in range(3):
            twist.linear.x = linear_x
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)

    def subway(self):
        #self.destroy_subscription(self.image_sub)
        os.system("mpg123 ./hri-audio/subway-surfers-huy.mp3")
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        self.movesub(linear_x=15.0, angular_z=0.0, duration=4)
        #self.destroy_subscription(self.image_sub)
        os.system("mpg123 ./hri-audio/subway-surfers-coin-collect.mp3")
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        self.movesub(linear_x=15.0, angular_z=15.0, duration=3)
        #self.destroy_subscription(self.image_sub)
        os.system("mpg123 ./hri-audio/subway-surfers-coin-collect.mp3")
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        self.movesub(linear_x=15.0, angular_z=-15.0, duration=3)
        #self.destroy_subscription(self.image_sub)
        os.system("mpg123 ./hri-audio/subway-surfers-huy.mp3")
        #self.image_sub = self.create_subscription(Image, '/color/image', self.image_callback, 10)

def main(args=None):
    rclpy.init(args=args)
    game = RobotCharades()
    rclpy.spin(game)
    game.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()
