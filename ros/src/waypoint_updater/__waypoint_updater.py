#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

# find good nearest points from given points 
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import numpy as np
import math

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

LOOKAHEAD_WPS = 80 # Number of waypoints we will publish. You can change this number

class VehicleMotionState(object):
    Drive = 0
    Stop = 1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # velocity subscriber 
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoints', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.vehicle = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.nearest_light = None
        self.vehicle_velocity = None # in m/s

        self.motion_state = VehicleMotionState.Drive
        self.deceleration_rate = None
        self.acceleration_rate = 0.75 # m/s

        self.previous_velocity = None        

        # loop every XXX hz :  XXX -> either one of parameters 50 30 20 
        self.loop()

    #
    # newly created
    #
    def loop(self):

        # looping process to publish the final waypoints

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.vehicle and self.waypoints_tree and self.vehicle_velocity:
                self.loop_process()
                #rospy.logwarn("loop[waypoints]: ")

            self.previous_velocity = self.vehicle_velocity
            rate.sleep()

    def loop_process(self):

        # take final lane data
        self.generate_lane()


    def generate_lane(self):

        lane = Lane()
        start_index = self.get_closest_waypoint()


        
        if start_index is not None:
        
            end_index = start_index + LOOKAHEAD_WPS
            lane_waypoints = self.base_waypoints[start_index:end_index]

            if self.nearest_light != None:
                rospy.logwarn("StartIdx %d EndIdx %d nearest_light %d" % (start_index, end_index, self.nearest_light))

            if self.nearest_light != None and self.nearest_light >= start_index and self.nearest_light <= end_index:
                self.motion_state = VehicleMotionState.Stop
                lane_waypoints = self.decelerate(lane_waypoints, start_index)
            
            elif self.motion_state == VehicleMotionState.Stop:
                # We arrive were previously decelerating/stopped but are
                # starting to accelerate/drive again
                self.motion_state = VehicleMotionState.Drive
                self.deceleration_rate = None

            if self.motion_state == VehicleMotionState.Drive:
                if abs(self.vehicle_velocity - self.get_waypoint_velocity(lane_waypoints[0])) > 1.0:
                    if self.previous_velocity == None:
                        start_velocity = self.vehicle_velocity
                    else:
                        start_velocity = max(self.previous_velocity+0.2, self.vehicle_velocity)
                    lane_waypoints = self.accelerate(lane_waypoints, start_velocity)
                else:
                    self.acceleration_start_velocity = None


            lane.waypoints = lane_waypoints

            self.final_waypoints_pub.publish(lane)
        else:
            rospy.logwarn("start index null....")        


    # Returns the index of the closest waypoint ahead of the vehicle
    def get_closest_waypoint(self):

        vehicle = [self.vehicle.pose.position.x, self.vehicle.pose.position.y]
        closest_index = self.waypoints_tree.query(vehicle, 1)[1]

        closest_waypoint = np.array(self.waypoints_2d[closest_index])
        previous_waypoint = np.array(self.waypoints_2d[closest_index - 1])
        vehicle = np.array(vehicle)

        waypoint_vector = closest_waypoint - previous_waypoint
        vehicle_vector = vehicle - closest_waypoint

        val = np.dot(waypoint_vector, vehicle_vector)

        if val > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
            #rospy.logwarn(" val: %.5f closest index: %d" % (val, closest_index) )
            
        return closest_index        

    def accelerate(self, waypoints, start_velocity):
        processed_waypoints = []
        for i, waypoint in enumerate(waypoints):
            p = Waypoint()
            p.pose = waypoint.pose

            distance = self.distance(waypoints, 0, i)
            target_speed = start_velocity + distance * self.acceleration_rate
            if target_speed < 0.5:
                target_speed = 0.5

            target_speed = min(target_speed, self.get_waypoint_velocity(waypoint))


            #rospy.logwarn("target speed %.4f" % target_speed)

            p.twist.twist.linear.x = target_speed
            processed_waypoints.append(p)


        return processed_waypoints


    def decelerate(self, waypoints, start_index):
        
        stop_index = self.nearest_light - start_index - 2 
        rospy.logwarn("stop_index(decelerate) %d", stop_index)

        processed_waypoints = []
        speed = self.vehicle_velocity
        
        for i, waypoint in enumerate(waypoints):
        
            p = Waypoint()
            p.pose = waypoint.pose

            distance = self.distance(waypoints, i, stop_index)
            if i >= stop_index:
                target_speed = 0
            elif distance < 15:
                if self.deceleration_rate == None:
                    self.deceleration_rate = self.vehicle_velocity / distance
                target_speed = self.deceleration_rate * distance
                if target_speed <= 1:
                    target_speed = 0
                target_speed = min(target_speed, self.get_waypoint_velocity(waypoint))
            else:
                target_speed = self.get_waypoint_velocity(waypoint)

            # for setting next velocity = waypoints.twist.twist.linear.x

            p.twist.twist.linear.x = target_speed
            processed_waypoints.append(p)
        
        return processed_waypoints

    #
    # callback section -- Subscriber
    #
    def pose_cb(self, msg):
        # TODO :
        # msg saved into self.vehicle
        self.vehicle = msg
        rospy.logwarn("pose callback.")

    def get_waypoint_xy(self,wp):
        return [wp.pose.pose.position.x, wp.pose.pose.position.y]

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.logwarn("waypoints callback.")
        self.base_waypoints = waypoints.waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [self.get_waypoint_xy(waypoint) for waypoint in waypoints.waypoints]
            #rospy.logwarn("waypoints_2d[waypoints_cb]: (%s,)" % self.waypoints_2d  )
            self.waypoints_tree = KDTree(self.waypoints_2d)        


    #
    # from traffic classifier (tl_detector)
    #
    def traffic_cb(self, msg):
        rospy.logwarn("traffic cb start.")
        #self.stopline_wp_idx = msg.data
        if msg.data == -1:
            self.nearest_light = None
        else:
            self.nearest_light = msg.data
            rospy.logwarn("traffic waypoints  %d" % self.nearest_light)

    # optional 
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def velocity_cb(self,msg):
        self.vehicle_velocity = msg.twist.linear.x
        rospy.logwarn("velocity %d", self.vehicle_velocity)

    def publish_waypoints():
        return 1





    #
    #   velocity manupulation
    # 
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
