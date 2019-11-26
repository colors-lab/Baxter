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

# I set these variables the arrange the start position of the robot end effector
arrange_x = 0.8
arrange_y = 0.6 #Note for left arm we use positive and for right arm we use negative of this value
arrange_z = 0.4
a=0
b=0
c=0
d=0
e=0
f=0

# Tuluhan: you can skip this class and its methods for now, the functions you need are rightcallback leftcallback and main
class LimbObj(object):

    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
	self._limb.set_joint_position_speed(0.3)
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
    
    def rcallback(self,data):
    #parserOnSpace = data.data.split(" ")
    #a = float(parserOnSpace[0])
	global a
	global b
	global c
        a=data.x
        b=data.y
        c=data.z
    #b = float(parserOnSpace[1])
    #c = float(parserOnSpace[2])

        if a<-0.2:
           a=-0.2
        elif a>0.3:
            a=0.3
        if b<-0.15:
           b=-0.15
        elif b>0.3:
            b=0.3
        if c<-0.2:
            c=-0.2
        elif c>0.25:
            c=0.25


        forward_orientation = Quaternion(x=-0.011186349352927234, y=0.7145362704349227, z=-0.006892809125978017, w=0.6994749981260645)
    #forward_orientation = Quaternion(x=0.5, y=-0.5, z=0.5, w=-0.5)
	
        armPose = Pose(position=Point(x=a+arrange_x, y=-b-arrange_y,z=c+arrange_z), orientation=forward_orientation)

        self.move_to_pose(armPose)

    def lcallback(self,data):
    #parserOnSpace = data.data.split(" ")
    #a = float(parserOnSpace[0])
	global d
	global e
	global f
        d=data.x
        e=data.y
        f=data.z
    #b = float(parserOnSpace[1])
    #c = float(parserOnSpace[2])

        if d<-0.2:
           d=-0.2
        elif d>0.3:
            d=0.3
        if e<-0.15:
           e=-0.15
        elif e>0.3:
            e=0.3
        if f<-0.2:
            f=-0.2
        elif f>0.25:
            f=0.25


        forward_orientation = Quaternion(x=-0.011186349352927234, y=0.7145362704349227, z=-0.006892809125978017, w=0.6994749981260645)
    #forward_orientation = Quaternion(x=0.5, y=-0.5, z=0.5, w=-0.5)
        armPose = Pose(position=Point(x=d+arrange_x,y=e+arrange_y,z=f+arrange_z), orientation=forward_orientation)
        
        self.move_to_pose(armPose)




#this methods take the x, y, z components of a position for left arm as string
#I preset the orientation as constant for now feel free to change it (on simulation only)
# the position and preset orientation are used to arrange the end position that will be asked to ik of baxter interface


# I chose topics as String, you can change this to position or to even pose if you like to.
def main():


    # Initialize the node
    rospy.init_node("tuluhan")
    #ENABLE IN SIMULATION
    #rospy.wait_for_message("/robot/sim/started", Empty)
    right_limb = LimbObj('right')
    left_limb = LimbObj('left')

    rospy.Subscriber("/right_hand_joint",Point,right_limb.rcallback,queue_size= 1)
    rospy.Subscriber("/left_hand_joint",Point,left_limb.lcallback,queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
