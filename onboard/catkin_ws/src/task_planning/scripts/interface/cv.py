import rospy
import yaml
import resource_retriever as rr
from custom_msgs.msg import CVObject


class CVInterface:
    MODELS_PATH = "package://cv/models/depthai_models.yaml"
    CV_CAMERA = "front"
    CV_MODEL = "yolov7_tiny_2023"

    def __init__(self):
        self.cv_data = {}

        with open(rr.get_filename(self.MODELS_PATH, use_protocol=False)) as f:
            model = yaml.safe_load(f)[self.CV_MODEL]
            for model_class in model['classes']:
                self.cv_data[model_class] = None
                topic = f"{model['topic']}{self.CV_CAMERA}/{model_class}"
                rospy.Subscriber(topic, CVObject, self._on_receive_cv_data, model_class)

    def _on_receive_cv_data(self, cv_data, object_type):
        self.cv_data[object_type] = cv_data

    # TODO add useful methods for getting data
    def get_data(self, name):
        return self.cv_data[name]
