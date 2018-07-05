from styx_msgs.msg import TrafficLight
import tensorflow as tf
import datetime
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        model_path = r'light_classification/sim_model.pb'
        self.graph = tf.Graph()
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as f:
                graph_def.ParseFromString(f.read())
                tf.import_graph_def(graph_def, name='')
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            rospy.loginfo('Light Inference time: {}'.format(c.total_seconds()))

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        rospy.loginfo('SCORES: {0}'.format(scores[0]))
        rospy.loginfo('CLASSES: {0}'.format(classes[0]))

        if scores[0] > 0.5:
            if classes[0] == 1:
                rospy.loginfo('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                rospy.loginfo('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                rospy.loginfo('YELLOW')
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
