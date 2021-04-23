#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
from tic_crane import msg

def main():
    rospy.init_node('tic_crane_state_machine')

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm:
        deploy_goal = msg.MoveToPositionGoal(position = 500)
        store_goal = msg.MoveToPositionGoal(position = 0)
        measure_goal = msg.MeasurementGoal(measurement_duration = 7)

        sm.add('DEPLOY', SimpleActionState('move_to_target_pose', msg.MoveToPositionAction, goal=deploy_goal), transitions={'succeeded':'MEASURE'})
        sm.add('MEASURE', SimpleActionState('measurement', msg.MeasurementAction, goal=measure_goal), transitions={'succeeded':'STORE'})
        sm.add('STORE', SimpleActionState('move_to_target_pose', msg.MoveToPositionAction, goal=store_goal), transitions={'succeeded':'succeeded'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()

