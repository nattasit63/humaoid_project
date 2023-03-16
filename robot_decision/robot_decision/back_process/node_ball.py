import rclpy
import py_trees_ros
from py_trees_ros_interfaces.msg import Ball
import py_trees.console as console

class Ball_class(object):

    def __init__(self):
        self.node = rclpy.create_node(node_name="ball",)
        self.publishers = py_trees_ros.utilities.Publishers(
            self.node,
            [
                ("ball","ball/detection", Ball, False),
            ]
        )
        timer_period = 1  # seconds
        self.ball = Ball()
 
        self.timer = self.node.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback
        )
        self.i = 5



    def timer_callback(self):
        if self.i >= 10 : 
            self.ball.is_detected = True
            console.info("************ Ball is detected ************")
          
            self.i = 5 # for switch T/F
        else :
            self.ball.is_detected = False
            console.info("************ Ball is undetected ************")
 
            self.i += 1
        self.publishers.ball.publish(self.ball)


    def shutdown(self):
        """
        Cleanup ROS components.
        """
        self.node.destroy_node()


def main():

    rclpy.init()  
    ball_class = Ball_class()
    try:
        rclpy.spin(ball_class.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        ball_class.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    
    main()