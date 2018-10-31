from styx_msgs.msg import TrafficLight
import tensorflow as tf  
import numpy as np


from datetime import datetime

class TLClassifier(object):
    def __init__(self, is_simulation=True):
        #TODO load classifier

        if is_simulation:
            PATH_TO_GRAPH = r'light_classification/model/ssd_sim/sim_frozen_inference_graph.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/model/ssd_real/real_frozen_inference_graph.pb'

        self.graph = tf.Graph()
        self.threshold = .5

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

            self.sess = tf.Session(graph=self.graph)

    def init_classifier(self, model, width, height, channels=3):
        print("init_classifier / tl_classifer", width)
        print("init_classifier / tl_classifer", height)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        print("Classification..")
        with self.graph.as_default():

            # image should be numpy array 

            img_expand = np.expand_dims(image, axis=0)
            start = datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.now()
            c = end - start
            print(c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        print('SCORES: ', scores[0])
        print('CLASSES: ', classes[0])

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('YELLOW')
                return TrafficLight.YELLOW


        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
