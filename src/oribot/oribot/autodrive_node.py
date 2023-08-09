import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from oribot.scripts.settings import settings, TrainingConfig
from oribot_interfaces.srv import Toggle
import oribot.scripts.cmd_utils as cmd_utils
import cv2
import oribot.scripts.image_utils as image_utils
import os
import numpy as np
import torch
import torch.nn.functional as F
import torchvision
from enum import Enum


torch.hub.set_dir(settings.training_config.model_root)

class DriveState(Enum):
    STOPPED = 0
    DRIVING = 1
    AVOIDING = 2

class AutodriveNode(Node):

    def __init__(self):
        super().__init__("autodrive_node")

        self.log("Started autodrive node.")
        self.config: TrainingConfig = settings.training_config
        self.drive_state = DriveState.STOPPED
        self.mean = 255.0 * np.array([0.485, 0.456, 0.406])
        self.stdev = 255.0 * np.array([0.229, 0.224, 0.225])
        self.normalize = torchvision.transforms.Normalize(self.mean, self.stdev)
        self._load_model()
        self.running = False
        

        # publishers
        self.drivetrain_publisher = self.create_publisher(Twist, '/oribot/cmd_vel', 10)
        self.log("Created Drivetrain Publisher")

        # subscriptions
        self.image_subscription = self.create_subscription(Image, "/video_source/raw", self.autodrive, 10)
        self.log("Subscribed to Image Topic")

        # service
        self.svc = self.create_service(Toggle, 'toggle_autodrive', self.toggle_status)
        self.log("Initialized autodrive node.")
        

    def toggle_status(self, request, response):
        self.log(f"Setting autodrive running state to {request.on}")
        self.start() if request.on else self.stop()
        response.status = self.running
        return response

    def log(self, txt: str):
        self.get_logger().info(txt)


    def stop_drivetrain(self):
        msg = Twist()
        self.drivetrain_publisher.publish(msg)
        self.drive_state = DriveState.STOPPED

    def start(self):
        self.running = True
        self.drive_state = DriveState.STOPPED


    def stop(self):
        self.running = False

    def _load_model(self):
        self.log("Preparing model...")
        
        model_file = self.config.best_model_file  

        self.log(f"Looking for saved model {model_file}")  
        has_model = os.path.isfile(model_file)

        if has_model: 
            self.log(f"Found saved model.") 
        else:
            self.log(f"Did not saved model. Proceeding without it.") 

        self.model = self.config.load_model(pretrained=(not has_model))
        self.device = torch.device('cuda')

        cat_count = self.config.num_categories

        # create model
        if self.config.classifier == "alexnet":
            self.model.classifier[6] = torch.nn.Linear(self.model.classifier[6].in_features, cat_count)
        elif self.config.model_name == "resnet18":
            self.model.fc = torch.nn.Linear(512, cat_count)
            self.model.eval().half()

        if has_model:
            self.log(f"Loading saved state ... {model_file}")
            self.model.load_state_dict(torch.load(model_file))
        else:
            self.log("Skipping saved state load, model does not exist yet.")

        self.model = self.model.to(self.device)

        self.log("model ready...")

    def _preprocess(self, sensor_image):

        x = image_utils.sensor_image_to_cv2(sensor_image)
        x = cv2.resize(x, (224,224))
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
        x = x.transpose((2, 0, 1))
        x = torch.from_numpy(x).float()
        x = self.normalize(x)
        x = x.to(self.device)
        x = x[None, ...]
        return x
    
    '''
        cuda_image = image_utils.sensor_image_to_cuda(sensor_image)
        x = image_utils.resize_cuda(cuda_image, 224, 224)
        x = np.transpose(x, (2, 0, 1))
        x = torch.as_tensor(x, device='cuda').float()
        x = self.normalize(x)
        x = x[None, ...]
        return x
    '''

    def predict(self, sensor_image):
        input = self._preprocess(sensor_image=sensor_image)
        output = self.model(input)
        output = F.softmax(output, dim=1)
        return output

    def _assign_predictions(self, y, categories):
        categories = sorted(self.config.categories.copy())
        d = {}
        for index, cat in enumerate(categories):
            d[cat] = float(y.flatten()[index])
        predictions = sorted(d.items(),key=lambda x:x[1], reverse=True)
        return predictions

    def autodrive(self, sensor_image: Image):
        if not self.running:
            self.drive_state = DriveState.STOPPED
            return

        y = self.predict(sensor_image)

        predictions = self._assign_predictions(y, self.config.categories)

        self.log(f"Prediction: {predictions}")

        k,v = predictions[0]

        new_state = DriveState.DRIVING if k == "forward" else DriveState.AVOIDING
        
        if new_state != self.drive_state:
            self.drive_state = new_state
            if not settings.debug:
                self.drivetrain_publisher.publish(cmd_utils.generate_twist_message(k, settings.robot_drive_speed))
    

def main(args=None):
    rclpy.init(args=args)
    node = AutodriveNode()
    rclpy.spin(node=node)
    node.stop()
    torch.cuda.empty_cache()
    rclpy.shutdown()