import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as SensorImage
#from jetson_utils import cudaImage, cudaFromNumpy, cudaMemcpy, cudaToNumpy, cudaAllocMapped, cudaConvertColor, cudaDeviceSynchronize, cudaResize

bridge = CvBridge()

def sensor_image_to_cv2(sensor_image: SensorImage):
    return bridge.imgmsg_to_cv2(sensor_image, "bgr8")

def cv2_to_jpeg(cv2_img):
    return cv2.imencode('.jpg', cv2_img)

def cv2_to_jpeg_bytes(cv2_img):
    return bytes(cv2_to_jpeg(cv2_img)[1])

#def sensor_image_to_cuda(sensor_image: SensorImage):
#    return cudaFromNumpy(bridge.imgmsg_to_cv2(sensor_image, "rgb8"))
     

def sensor_image_to_jpeg(sensor_image: SensorImage):
    return cv2_to_jpeg(sensor_image_to_cv2(sensor_image))

def sensor_image_to_jpeg_bytes(sensor_image: SensorImage):
    return cv2_to_jpeg_bytes(sensor_image_to_cv2(sensor_image))

#def resize_cuda(img: cudaImage, width, height):
#    resized = cudaAllocMapped(width=width, height=height, format=img.format)
#    cudaResize(img, resized)
#    return resized