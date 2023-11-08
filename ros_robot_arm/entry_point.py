""" Entry point for ros_robot_arm """

import rclpy
from . import Node

def main(args=None):
    
    rclpy.init()
    
    try:
        node = Node()
        
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        finally: node.destroy_node()
        
    finally: rclpy.shutdown()
    
    return
   
if __name__ == '__main__': main()