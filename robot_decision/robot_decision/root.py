#!/usr/bin python3
# -*- coding: utf-8 -*-
import py_trees
import py_trees_ros.trees
import py_trees.console as console
from py_trees_ros import subscribers

import rclpy
import sys

import behavior
from back_process.to_blackboard import TOBlackBoard



def check_is_ball_detected (blackboard: py_trees.blackboard.Blackboard):
    msg= f' msg : {blackboard.is_ball_detected}'
    print(msg)
    return blackboard.is_ball_detected

def create_root():
    main_root = py_trees.composites.Parallel(
        name = "Main_Root", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=False)

    ballToBB = TOBlackBoard(
        name="Ball2BB",
        topic_name="/ball/detection",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(), )
    
    ball_priorities = py_trees.composites.Sequence("Task_Ball",memory=False)

    # ball_check = py_trees.decorators.EternalGuard(
    #     name="Ball Detected?",
    #     condition=check_is_ball_detected,
    #     blackboard_keys={"is_ball_detected"},
    #     child=behavior.Ball_behavior(name='Jerlaw')
    # )

    finding_ball = py_trees.composites.Sequence("Finding Ball",memory=False)
    ball_check_success = behavior.Finding_Ball(name='Ball Deteteced')


    main_root.add_children([topics2bb,ball_priorities])
    topics2bb.add_child(ballToBB)
    ball_priorities.add_child(finding_ball)
    finding_ball.add_child(ball_check_success)
    
    

    
    
    return main_root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="root_node", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
if __name__ == '__main__':
   main()

  

    
