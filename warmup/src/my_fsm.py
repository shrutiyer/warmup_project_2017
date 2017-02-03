#!/usr/bin/env python

import rospy
import smach
import smach_ros

from drive_square import SquareDrivingController

class StateGetToGoal(smach.State):

    class Outcomes:
        goal_achieved = 'goal_achieved'

    def __init__(self):
        smach.State.__init__(self, outcomes = [self.Outcomes.goal_achieved])

    def execute(self, userdata):
        goal_achieved = True
        if goal_achieved: # TODO: Define this logic.
            return self.Outcomes.goal_achieved

class StateDriveSquare(smach.State):

    class Outcomes:
        square_made = 'square_made'

    def __init__(self):
        self.controller = SquareDrivingController()
        smach.State.__init__(self, outcomes = [self.Outcomes.square_made])

    def execute(self, userdata):
        self.controller.run() # TODO: The while loop internally needs to exit when square is complete.
        square_made = True
        if square_made: # TODO: Define this logic
            return self.Outcomes.square_made

def main():
    rospy.init_node('my_fsm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes =  ['finished'])

    with sm:
        smach.StateMachine.add('GET_TO_GOAL', StateGetToGoal(), transitions = {
            StateGetToGoal.Outcomes.goal_achieved: 'DRIVE_SQUARE'
        })
        smach.StateMachine.add('DRIVE_SQUARE', StateDriveSquare(), transitions = {
            StateDriveSquare.Outcomes.square_made: 'finished'
        })

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
