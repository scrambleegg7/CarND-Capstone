#!/usr/bin/env python

import numpy as np
import rospy
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial import KDTree

from std_msgs.msg import Int32

import math
from enum import Enum

import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

class State(Enum):
    ACCELERATION = 1
    DECELERATION = 2


LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #initialize before call back 
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.base_waypoints = None
        self.a = None
        self.number_of_waypoints = None
        self.current_velocity_in_mps = None
        self.state_changed = False

        # default current state. (= decrease speed)
        self.current_state = State.DECELERATION
        self.final_waypoints = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

     	# Published to final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.acceleration_limit_in_mps = rospy.get_param('~accel_limit', 1.)
        self.deceleration_limit_max_in_mps = -rospy.get_param('~decel_limit', -5.)
        self.deceleration_limit_min_in_mps = min(1.0, -rospy.get_param('~decel_limit', -5.) / 2.)
        self.max_velocity_in_mps = rospy.get_param('/waypoint_loader/velocity') / 3.6    # 40 / 3.6
        
        self.loop()

    
    def loop(self):
        rate = rospy.Rate(30) # 
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints: 
                # Get closest waypoint                
                #closest_waypoint_idx = self.get_closest_waypoint_id()
                #self.publish_waypoints2()
                self.publish_waypoints3()                
                #rospy.logwarn("a teste: {0}".format(self.publish_waypoints2()))
            rate.sleep()
    
    def publisher_process(self):

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        SAFETY_BUFFER = 0.5

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        closest_idx = self.get_closest_waypoint_id()  # closest index from running vehicle
        furthest_idx = closest_idx + LOOKAHEAD_WPS

        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:furthest_idx] # referred to base_waypoints callback routine
        
        # base waypoints is splitted from closest_idx to furthest poists equal to +LOOKAHEAD  
        base_waypoints = lane.waypoints

        if self.current_state == State.ACCELERATION:

            if self.stopline_wp_idx != -1:  # found Sinal pointer  (self.stopline_wp_idx < furthest_idx)

                #if self.stopline_wp_idx < furthest_idx:
                    #self.current_state = State.DECELERATION
                    #self.state_changed = True
                    #rospy.logwarn("state changed DECELERATION to ACCELERATION. %d current state %s" % (self.state_changed, self.current_state) )

                #closest_idx = self.get_closest_waypoint_id()
                start_car_position = self.pose.pose.position
                trafficlight_position = self.base_waypoints.waypoints[self.stopline_wp_idx].pose.pose.position

                rospy.logwarn("start car pos x:%d y:%d z:%d" % (start_car_position.x,start_car_position.y,start_car_position.z)    )
                rospy.logwarn("traffic pos x:%d y:%d z:%d" % (trafficlight_position.x,trafficlight_position.y,trafficlight_position.z)    )

                brake_distance = dl(start_car_position, trafficlight_position) - SAFETY_BUFFER  # where to put brake 
                rospy.logwarn("brake distance %.2f" % brake_distance    )
                rospy.logwarn("velocity in mps %.2f" % self.current_velocity_in_mps    )

                min_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_max_in_mps
                max_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_min_in_mps
                if max_brake_distance >= brake_distance >= min_brake_distance:
                    # call decelerate_waypoints
                    self.current_state = State.DECELERATION
                    self.state_changed = True
                    rospy.logwarn("state changed DECELERATION to ACCELERATION. %d current state %s" % (self.state_changed, self.current_state) )
                
                rospy.logwarn("min brake distance %.2f max brake distance %.2f"  % (min_brake_distance, max_brake_distance ) )

        elif self.current_state == State.DECELERATION:
            if self.stopline_wp_idx == -1:
                self.current_state = State.ACCELERATION
                self.state_changed = True
                rospy.logwarn("state changed from DECELERATION to ACCELERATION.")

        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        # Handle states
        if self.current_state == State.ACCELERATION and self.state_changed:
            lane.waypoints = self.accelerate(base_waypoints, closest_idx)
        
        elif self.current_state == State.ACCELERATION and not self.state_changed:
            self.continue_with_current_state(base_waypoints, closest_idx, self.max_velocity_in_mps)
        
        elif self.current_state == State.DECELERATION and self.state_changed:
            #self.decelerate(lane, waypoint_index)
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        elif self.current_state == State.DECELERATION and not self.state_changed:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            #self.continue_with_current_state(lane, closest_idx, 0) # current velocity set to ZERO
        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        self.state_changed = False
        self.final_waypoints = copy.deepcopy(lane.waypoints)

        return lane

    def accelerate(self, waypoints, closest_idx):

        acceleration = self.acceleration_limit_in_mps # 40 / 3.6
        current_velocity = self.current_velocity_in_mps
        target_velocity = self.current_velocity_in_mps

        temp = []
        # waypoints = splitted waypoints 
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)

            target_velocity = math.sqrt(current_velocity ** 2.0 + 2.0 * acceleration * dist)
            if target_velocity > self.max_velocity_in_mps:
                target_velocity = self.max_velocity_in_mps

            #rospy.logwarn("target velocity %.2f" % target_velocity)

            p.twist.twist.linear.x = target_velocity
            #p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        #
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx

    def continue_with_current_state(self, lane, closest_index, cv):
        # it is dummy function , nothing to work         
        j = 0
        temp = []


    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_id()
        furthest_idx = closest_idx + LOOKAHEAD_WPS
        #base_waypoints = self.base_lane.waypoints[closest_idx:furthest_idx]
        

        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:furthest_idx]
        base_waypoints = lane.waypoints

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= furthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        self.final_waypoints = copy.deepcopy( lane.waypoints )

        return lane


    def publish_waypoints3(self):
        final_lane = self.publisher_process()
        self.final_waypoints_pub.publish(final_lane)

    def publish_waypoints2(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
     
    #def distance(self, a, b):
    #    return ((a.x-b.x)**2 + (a.y-b.y)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        # waypoints = splitted waypoints 
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

            #rospy.logwarn("decelerate velocity %.2f" % p.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
    	self.pose = msg

    # slicer function to split waypoint pose pose position
    def waypoint_xy(self, waypoint):
        position = waypoint.pose.pose.position
        return [position.x, position.y]

    def waypoints_cb(self, waypoints): 
        self.base_waypoints = waypoints # just storing

        if not self.waypoints_2d:
            self.waypoints_2d = [ self.waypoint_xy(waypoint) for waypoint in waypoints.waypoints]
            self.number_of_waypoints = len(waypoints.waypoints)
            #rospy.logwarn("waypoints (%s,)" % self.waypoints_2d)

            # setup KDTree function of scipy memeber
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def velocity_cb(self, msg):
        self.current_velocity_in_mps = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')