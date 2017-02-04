#!/usr/bin/env python

import rospy
import smach
import smach_ros

from drive_square import SquareDrivingController
from person_following import PersonFollowing

class StatePersonFollowing(smach.State):

    class Outcomes:
        person_bumped = 'person_bumped'

    def __init__(self):
        self.controller = PersonFollowing()
        smach.State.__init__(self, outcomes = [self.Outcomes.person_bumped])

    def execute(self, userdata):
        self.controller.run() # This will run in a loop till complete.
        return self.Outcomes.person_bumped

class StateDriveSquare(smach.State):

    class Outcomes:
        square_made = 'square_made'

    def __init__(self):
        self.controller = SquareDrivingController()
        smach.State.__init__(self, outcomes = [self.Outcomes.square_made])

    def execute(self, userdata):
        self.controller.run() # This will run in a loop till complete.
        self.controller.reset()
        return self.Outcomes.square_made

def main():
    rospy.init_node('my_fsm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes =  ['finished'])

    with sm:
        smach.StateMachine.add('PERSON_FOLLOWING', StatePersonFollowing(), transitions = {
            StatePersonFollowing.Outcomes.person_bumped: 'DRIVE_SQUARE'
        })
        smach.StateMachine.add('DRIVE_SQUARE', StateDriveSquare(), transitions = {
            StateDriveSquare.Outcomes.square_made: 'PERSON_FOLLOWING'
        })

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
