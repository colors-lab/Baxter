#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import baxter_interface
import tf



from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,

)
from std_msgs.msg import (
    Header,
    Empty,
    String,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import EndpointState

#right_initial = {'right_s0': 0.43358413821737485, 'right_s1': -1.144779605732543, 'right_w0': 2.724185639096819, 'right_w1': 0.5774896108114527, 'right_w2': -2.873215870645183, 'right_e0': 0.14742485311716008, 'right_e1': 1.7214107142134132}

    #left_intitial = {'left_w0': -2.7601373782139476, 'left_w1': 0.6213144532738806, 'left_w2': 2.8623992298514604, 'left_e0': -0.16382219512232066, 'left_e1': 1.8468873184382062, 'left_s0': -0.404506321535548, 'left_s1': -1.2159530533565661}

# I set these variables the arrange the start position of the robot end effector
arrange_x = 0.8
arrange_y = 0.6 #Note for left arm we use positive and for right arm we use negative of this value
arrange_z = 0.4

# Tuluhan: you can skip this class and its methods for now, the functions you need are rightcallback leftcallback and main
class LimbObj(object):

    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
	self._gripper.calibrate()
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled

        print("Enabling robot... ")
        self._rs.enable()

    def move_to_pose(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")


#this methods take the x, y, z components of a position for left arm as string
#I preset the orientation as constant for now feel free to change it (on simulation only)
# the position and preset orientation are used to arrange the end position that will be asked to ik of baxter interface

def leftcallback(data):
    left_limb  = LimbObj('left')
    #parserOnSpace = data.data.split(" ")
    #a = float(parserOnSpace[0])
    a=data.x
    b=data.y
    c=data.z
    #b = float(parserOnSpace[1])
    #c = float(parserOnSpace[2])
    forward_orientation = Quaternion(x=-0.011186349352927234, y=0.7145362704349227, z=-0.006892809125978017, w=0.6994749981260645)
    #forward_orientation = Quaternion(x=0.5, y=-0.5, z=0.5, w=-0.5)

    leftPose = Pose(position=Point(x=a+arrange_x, y=b+arrange_y,z=c+arrange_z), orientation=forward_orientation)

    left_limb.move_to_pose(leftPose)


#basically does the same thing as the leftcallback for the right arm
def rightcallback(data):
    right_limb  = LimbObj('right')
    #parserOnSpace = data.data.split(" ")
    a=data.x
    b=data.y
    c=data.z
    #a = float(parserOnSpace[0])
    #b = float(parserOnSpace[1])
    #c = float(parserOnSpace[2])
    forward_orientation = Quaternion(x=-0.011186349352927234, y=0.7145362704349227, z=-0.006892809125978017, w=0.6994749981260645)
    #forward_orientation = Quaternion(x=0.5, y=-0.5, z=0.5, w=-0.5)
    rightPose =Pose(position=Point(x=a+arrange_x, y=-b-arrange_y,z=c+arrange_z), orientation=forward_orientation)

    right_limb.move_to_pose(rightPose)

# I chose topics as String, you can change this to position or to even pose if you like to.
def main():


    # Initialize the node
    rospy.init_node("tuluhan")
    #ENABLE IN SIMULATION
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.Subscriber("/right_hand_joint",Point,rightcallback,queue_size = 1)
    rospy.Subscriber("/left_hand_joint",Point,leftcallback,queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
