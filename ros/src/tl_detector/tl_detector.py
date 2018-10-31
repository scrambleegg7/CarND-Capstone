#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
from keras.models import load_model

STATE_COUNT_THRESHOLD = 3
DIST_FORWARD_VISIBILE = 200
TL_COLOR = ['RED', 'YELLOW', 'GREEN', 'UNKNOWN', 'NODETECT']
TL_COLOR_ARRAY = [(255, 0, 0),  (255, 255, 0), (0, 255, 0), (255, 255, 255), (255, 255, 255)]


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.simulation = self.config["is_sim"]
        rospy.logwarn( "Simulation software ?  : %d" % self.simulation  )


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.simulation)
        self.listener = tf.TransformListener()


        # set simulation flag from traffic_light_config
        #         


        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0



        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg

        state = TrafficLight.UNKNOWN

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state != TrafficLight.GREEN else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        if self.waypoints is None:
            return

        # loop through the base waypoints and find the closest one
        min_dist = float("inf")
        closest_idx = 0

        for idx, waypoint in enumerate(self.waypoints):
            dist = self.calc_dist(x,waypoint.pose.pose.position.x, y, waypoint.pose.pose.position.y)

            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx

    # Euclidean distance.
    def calc_dist(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # Get traffic light state and bbox
        # tl_state, tf_box = self.light_classifier.get_classification(cv_image)
        tl_state = self.light_classifier.get_classification(cv_image)

        # set rectangle pos and draw rectangle
        #pos_lup = (tf_box[1], tf_box[0])  #left  x, y
        #pos_rdwn = (tf_box[3], tf_box[2]) #right x, y

        # set traffic color
        #color = TL_COLOR_ARRAY[tl_state]

        #cv2.rectangle(cv_image, pos_lup, pos_rdwn, color, 2)

        # display image of front camrera(BGR form. for Opencv)
        #img_out = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        #img_out = cv2.resize(img_out, None, fx=.5, fy=.5)
        #cv2.imshow("image of front camera", img_out)
        #cv2.waitKey(1)

        return tl_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        light = None
        light_wp = None

        if self.waypoints is None:
            rospy.logwarn('no self.waypoints (process_traffic_lights)')
            return (-1, TrafficLight.UNKNOWN)

        if not self.pose:
            rospy.logwarn('no self.pose (process_traffic_lights)')
            return (-1, TrafficLight.UNKNOWN)

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        #TODO find the closest visible traffic light (if one exists)
        min_dist = float('inf')
        for light_idx, stop_line in enumerate(stop_line_positions):
            line_idx = self.get_closest_waypoint(stop_line[0], stop_line[1])

            if line_idx > car_position and abs(line_idx-car_position) < DIST_FORWARD_VISIBILE and abs(line_idx-car_position) < min_dist:
                light = self.lights[light_idx]
                light_wp = line_idx


        if light:
#            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
#            img_out = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
#            img_out = cv2.resize(img_out, None, fx=.5, fy=.5)
#            cv2.imshow("detected", img_out)
#            cv2.waitKey(1)


            light_state = self.get_light_state(light)
            rospy.loginfo("Traffic Light. Current State: %s", TL_COLOR[light_state])
            return (light_wp, light_state)

        return (-1, TrafficLight.UNKNOWN)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
