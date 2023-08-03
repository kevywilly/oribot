import atexit
import rclpy
from rclpy.node import Node
import torch
import torch.nn.functional as F
import torch.optim as optim
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from oribot.scripts.settings import settings, TrainingConfig
import torchvision.models as models
import os

torch.hub.set_dir(settings.training_config.model_root)

class TrainerNode(Node):

    def log(self, txt: str):
        self.get_logger().info(txt)

    def __init__(self):

        super().__init__("trainer_node")

        self.config = settings.training_config
        self.epochs = 60
        self.cam_index = 1
        self.train_pct: float = 0.5
        self.learning_rate: float = 0.001
        self.momentum: float = 0.9

        
        try:
            os.makedirs(self.config.best_model_folder)
        except FileExistsError:
            pass
        except Exception as ex:
            print(ex)
            raise ex
            
        self.log(f"Trainer loaded: {self.config.name}, epochs: {self.epochs}")
        

    def train(self):

        self.log("loading datasets...")

        model = models.alexnet(pretrained=True)

        self.log(f"using path {self.config.training_data_path} for training data.")

        dataset = datasets.ImageFolder(
            self.config.training_data_path,
            transforms.Compose([
                transforms.ColorJitter(0.1, 0.1, 0.1, 0.1),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
            ]))

        datapoints = len(dataset)
        train_size = int(self.train_pct * datapoints)
        test_size = datapoints - train_size

        self.log(f"found {datapoints} datapoints.")

        train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size,test_size])

        train_loader = torch.utils.data.DataLoader(
            train_dataset,
            batch_size=8,
            shuffle=True,
            num_workers=0
        )

        test_loader = torch.utils.data.DataLoader(
            test_dataset,
            batch_size=8,
            shuffle=True,
            num_workers=0
        )

        print(f"loading model...{self.config.name}")

        print(f"Categories: {self.config.num_categories}")
        
        model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, self.config.num_categories)

        print("training model...")

        NUM_EPOCHS = 30
    
        best_accuracy = 0.0
        
        self.log(f"Best model path: {self.config.best_model_file}")

        if os.path.isfile(self.config.best_model_file):
                print(f"loading best model from {self.config.best_model_file}")
                model.load_state_dict(torch.load(self.config.best_model_file))

        device = torch.device('cuda')
        model = model.to(device)

        optimizer = optim.SGD(model.parameters(), lr=self.learning_rate, momentum=self.momentum)

        for epoch in range(NUM_EPOCHS):

            for images, labels in iter(train_loader):
                images = images.to(device)
                labels = labels.to(device)
                optimizer.zero_grad()
                outputs = model(images)
                loss = F.cross_entropy(outputs, labels)
                loss.backward()
                optimizer.step()

            test_error_count = 0.0

            for images, labels in iter(test_loader):
                images = images.to(device)
                labels = labels.to(device)
                outputs = model(images)
                #test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))
                #test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))
                #err = len(torch.nonzero(outputs.argmax(1) - labels).flatten())
                test_error_count += len(torch.nonzero(outputs.argmax(1) - labels).flatten())

            self.log(f"error_count: {test_error_count}, dataset_len: {len(test_dataset)}")

            test_accuracy = 1.0 - float(test_error_count) / float(len(test_dataset))

            self.log('EPOCH %d: ACCURACY %f' % (epoch, test_accuracy))

            if test_accuracy > best_accuracy:
                self.log(f"saving best model... with accuracy: {test_accuracy}")
                torch.save(model.state_dict(), self.config.best_model_file)
                best_accuracy = test_accuracy

            

def main(args=None):
    rclpy.init(args=args)
    node = TrainerNode()
    node.train()
    torch.cuda.empty_cache()
    rclpy.shutdown()