from pydantic import BaseModel
from typing import List, Optional, Any
import torchvision
from pydantic_settings import BaseSettings
import os


class CameraConfig(BaseModel):
    width: int = 1280
    height: int = 720
    fps: float = 30
    capture_width: int = 1280
    capture_height:int = 720
    stereo: bool = False

class TrainingConfig:

    def __init__(self, data_root: str, name: str, classifier: str, categories: List[str], epochs: int = 30, learning_rate: float = 0.001, momentum: float = 0.9):
        self.name = name
        self.filename = name.lower().replace(" ","_")
        self.classifier = classifier
        self.categories = categories
        self.data_root = data_root
        self.model_root = os.path.join(data_root,"models")
        self.best_model_folder = os.path.join(self.model_root,"best")
        self.best_model_file = os.path.join(self.best_model_folder,self.filename+".pth")
        self.training_data_path = os.path.join(self.data_root, self.filename)
        self.num_categories = len(self.categories)


    def load_model(self, *args, **kwargs):
        if self.classifier.lower() == "alexnet":
            return torchvision.models.alexnet(*args, **kwargs)
        elif self.classifier.lower() == "resnet18":
            return torchvision.models.resnet18(*args, **kwargs)
        else:
            raise "Invalid model specified, please use 'alexnet' or 'resnet18"
        
class AppSettings(BaseSettings):

    i2c_port: int = 7
    # Drivetrain Settings
    motor_1_alpha: float = 1.0
    motor_2_alpha: float = 1.0
    motor_3_alpha: float = 1.0
    motor_4_alpha: float = 1.0
    motor_max_rpm: float = 200

    wheel_radius_meters: float = 65/2.0/1000
    wheel_base_meters: float = 175/1000.0

    robot_drive_speed: float = 0.50
    robot_turn_speed: float = 0.50

    # Training Settings

    training_config: TrainingConfig = TrainingConfig(
        data_root="/oribot/data",
        name="obstacle3d",
        classifier="alexnet",
        categories=["forward", "left", "right"]
    )

    camera_config: CameraConfig = CameraConfig(
        width = 1280, height=720, fps=30, capture_width=1280, capture_height=720, stereo=False
    )
    # Input Settings

    
    use_detectnet: bool = False 
    detectnet_overlay: str = "box,labels,conf"
    detect_net_model: str = "ssd-mobilenet-v2"
    detect_net_threshold: float = 0.5

    debug: bool = False

settings = AppSettings()