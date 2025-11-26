import rclpy
from rclpy.node import Node

# Imports for the custom Hand message type (subscriber)
from aisd_msgs.msg import Hand

# Imports for the standard Twist message type (publisher for movement)
from geometry_msgs.msg import Twist


class Move(Node):
    """
    A ROS2 node that subscribes to Hand messages and publishes Twist messages
    to control a robot's velocity.
    """
    def __init__(self):
        # Call the parent Node constructor with the node name 'move'
        super().__init__('move')

        # Create a subscriber for Hand messages on the 'cmd_hand' topic
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)

        # Create a publisher for Twist messages on the 'cmd_vel' topic (robot velocity command)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg: Hand):
        # Initialize the angular and linear movement variables
        angle = 0.0
        linear = 0.0

        # Check if the index finger x-coordinate is far right (x > 0.55)
        if msg.xindex > 0.55:
            # Log a "right" command
            self.get_logger().info('right')
            # Set negative angular velocity (turn right)
            angle = -0.1
        # Check if the index finger x-coordinate is far left (x < 0.45)
        elif msg.xindex < 0.45:
            # Log a "left" command
            self.get_logger().info('left')
            # Set positive angular velocity (turn left)
            angle = 0.1
        # If the index finger is in the center region
        else:
            # Set angular velocity to zero (no turn)
            angle = 0.0

        # Check for linear movement (move forward/come)
        # If the index finger is further right than the pinky finger (xindex > xpinky)
        if msg.xindex > msg.xpinky:
            # Log a "come" command
            self.get_logger().info('come')
            # Set positive linear velocity (move forward)
            linear = 0.5
        # Otherwise (stay)
        else:
            # Log a "stay" command
            self.get_logger().info('stay')
            # Set linear velocity to zero (stay put)
            linear = 0.0

        # Create a new Twist message
        twist = Twist()
        # Set the forward/backward speed (linear.x is the x-axis velocity)
        twist.linear.x = linear
        # Set the turning speed (angular.z is the z-axis rotation velocity)
        twist.angular.z = angle

        # Check if there are any subscribers to the velocity publisher
        if self.vel_publisher.get_subscription_count() > 0:
            # Publish the Twist message to command movement
            self.vel_publisher.publish(twist)
        # If no one is listening
        else:
            # Log a message indicating the node is waiting for a subscriber
            self.get_logger().info('waiting for subscriber')


def main(args=None):
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create an instance of the Move node
    move_node = Move()

    # Keep the node running
    rclpy.spin(move_node)

    # Clean up and shut down
    move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
