from geometry_msgs.msg import Twist

def generate_twist_message(direction: str, speed: float) -> Twist:

    msg = Twist()

    dir = direction.upper()
    
    if dir == "FORWARD":
        msg.linear.x = speed
    if dir == "FORWARD_LEFT":
        msg.linear.x = speed/2
        msg.angular.z = -speed/2
    if dir == "FORWARD_RIGHT":
        msg.linear.x = speed/2
        msg.angular.z = speed/2
    if dir == "BACKWARD_LEFT":
        msg.linear.x = -speed/2
        msg.angular.z = speed/2
    elif dir == "BACKWARD_LEFT":
        msg.linear.x = -speed/2
        msg.angular.z = -speed/2
    elif dir == "BACKWARD":
        msg.linear.x = -speed
    elif dir == "LEFT" or dir == "SLIDE_LEFT":
        msg.angular.z = -speed
    elif dir == "RIGHT" or dir == "SLIDE_RIGHT":
        msg.angular.z = speed
    else:
        msg.angular.z = 0.0
        msg.angular.x = 0.0

    return msg