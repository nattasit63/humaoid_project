#!/usr/bin/env python3

import py_trees
import rclpy.qos
import py_trees.console as console
from py_trees_ros import subscribers
from py_trees_ros_interfaces.msg import Ball
class TOBlackBoard(subscribers.ToBlackboard):

    def __init__(self,name: str,topic_name: str,qos_profile: rclpy.qos.QoSProfile):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=Ball,
                         qos_profile=qos_profile,
                         blackboard_variables={"msg_ball": None},
                         clearing_policy=py_trees.common.ClearingPolicy.NEVER
                         )
        

        self.blackboard.register_key(
            key="is_ball_detected",
            access=py_trees.common.Access.WRITE)
        

      
        self.blackboard.msg_ball = Ball()
        self.blackboard.msg_ball.is_detected  = False  # decision making

        # self.blackboard.is_ball_detected = self.blackboard.ball.is_detected   # decision making


    def update(self):

        status = super(TOBlackBoard, self).update()
        
        if status != py_trees.common.Status.RUNNING:
            self.blackboard.is_ball_detected = self.blackboard.msg_ball.is_detected

            console.info('MSG : ' + str(self.blackboard.msg_ball.is_detected))
        return status
        # self.blackboard.is_ball_detected = self.blackboard.msg_ball.is_detected
        # if self.blackboard.ball.is_detected :
        #     py_trees.common.Status.SUCCESS
        #     return status

        # else:
        #     py_trees.common.Status.RUNNING
        #     return status

        