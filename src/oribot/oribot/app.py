import rclpy
import signal
import threading
import flask
from flask_cors import CORS
from flask import Flask, Response, jsonify
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from oribot_interfaces.srv import Toggle
from oribot.scripts.image_collector import ImageCollector
import oribot.scripts.cmd_utils as cmd_utils
import oribot.scripts.image_utils as image_utils

class Api(Node):
    def __init__(self):
        super().__init__('api_node')
        self.image: Image = None
        self.jpeg_bytes: bytes = None
        self.collector = ImageCollector()
        self.autodrive_on = False

        # publishers
        self.drivetrain_publisher = self.create_publisher(Twist, '/oribot/cmd_vel', 10)

        # subscriptions
        self.create_subscription(Image, "/video_source/raw", self.image_callback, 10)

        # client
        self.autodrive_client = self.create_client(Toggle, "toggle_autodrive")
        while not self.autodrive_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('autodrive service not available, waiting again...')
       
    
    def toggle_autodrive(self):
        req = Toggle.Request()
        status = not self.autodrive_on
        req.on = status
        result = self.autodrive_client.call(req)
        
        if result:
            self.get_logger().info(f"got autodrive result {str(result.status)}")
            self.autodrive_on = result.status
        else:
            self.get_logger().error(f"autodrive timed out")
            self.autodrive_on = status
        
        return self.autodrive_on
    

    def collect_image(self, category):
        try:
            self.get_logger().info(f"received collect image request {category}")
            img = self.get_jpeg()
            return self.collector.collect(category, img)
        
        except Exception as ex:
            self.get_logger().error(str(ex))

    def drive(self, direction: str, speed: float):
        msg = cmd_utils.generate_twist_message(direction, speed)
        self.drivetrain_publisher.publish(msg)

    def image_callback(self, msg: Image):
        self.image = msg
        self.jpeg_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)

    def get_image(self):
        return self.image
    
    def get_jpeg(self):
        return self.jpeg_image_bytes
            

def ros2_thread(node: Api):
    print('entering ros2 thread')
    rclpy.spin(node)
    node.destroy_node()
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


rclpy.init(args=None)
app_node = Api()

app = Flask(__name__)
CORS(app)
cors = CORS(app, resource={
    r"/*": {
        "origins": "*"
    }
})

prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)


def _get_stream(input: str = "input1"):
    while True:
        # ret, buffer = cv2.imencode('.jpg', frame)
        try:
            yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + app_node.get_jpeg() + b'\r\n'
            )  # concat frame one by one and show result
        except Exception as ex:
            pass
    
@app.route('/api/stream')
def stream():
    return Response(_get_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/stream/<input>')
def stream_input(input: str):
    return Response(_get_stream(input), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/drive/<direction>/<speed>')
def drive_cmd(direction, speed):
    app_node.drive(direction, float(speed))
    return {'drive': direction, 'speed': float(speed)}

@app.route('/api/categories/<category>/collect')
def collect(category: str):
    try:
        resp = app_node.collect_image(category)
        return {'count': resp}
    except Exception as ex:
        return {'count': -1, 'error': str(ex)}

@app.get('/api/categories/counts')
def category_counts():
    results = app_node.collector.get_categories()
    return jsonify(results)

@app.get('/api/categories/<category>/images')
def category_images(category: str):
    return {"images": app_node.collector.get_images(category)}


@app.get('/api/categories/<category>/images/<name>')
def get_image(category, name):
    bytes_str = app_node.collector.load_image(category, name)
    response = flask.make_response(bytes_str)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

@app.get('/api/categories/<category>/images/<name>/<cam_index>')
def get_image2(category, name, cam_index):
    bytes_str = app_node.collector.load_image(category, name)
    response = flask.make_response(bytes_str)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

@app.delete('/api/categories/<category>/images/<name>')
def delete_image(category, name):
    resp = app_node.collector.delete_image(category, name)
    return {"status": resp}

@app.route('/api/autodrive')
def toggle_autodrive():
    #status = app_node.toggle_autodrive()
    
    return jsonify({"autodrive": False})


def main(args=None):
    threading.Thread(target=ros2_thread, args=[app_node]).start()
    app.run(host="0.0.0.0", debug=True, use_reloader = False)

if __name__=="__main__":
    main()