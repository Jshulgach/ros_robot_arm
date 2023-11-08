
import serial
import rclpy
import rclpy.node
import rclpy.qos
#from rclpy import qos

from geometry_msgs.msg import Point, Vector3
from example_interfaces.msg import Int32, Float64


# Declare default quality-of-service settings for ROS2.
#DEFAULT_QOS = qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

# ROS2 node 
class Node(rclpy.node.Node):    

    def __init__(self, 
                 *args, 
                 node_name='ros_robot_driver',
                 serialport='COM5',
                 baudrate=115200,
                 verbose=False,
                 **kwargs,
                 ):
        """ 
        A ROS2 node that serves as a driver to interface the robot to the ROS2 network
     
        Parameters:
        ------------
        node_name       : (str) A specific node name
        serialport      : (str) Port number to allow serial connection
        baudrate        : (int) Baudrate for serial communication
        verbose         : (bool) Enable/disable verbose output text
     
        """
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, **kwargs)
        
        # Initialize the node.
        self.logger("Initializing robot interface")        
        self.initialize_parameters()
        self.initialize_publishers()
        self.initialize_subscriptions()
    
        # Initialize local variables.
        self._update_timer = None
        self._current_position = [0.0, 0.0, 0.0]
        self.usb_serial = self.connect_to_serial(serialport, baudrate)
        if self.usb_serial:
            self.initialize_timer()
            self.logger("Successful initialization")  
                    

    def logger(self, msg):
        """ Helper function for outputting to ROS2 terminal """
        self.get_logger().info(str(msg))
        
    def initialize_subscriptions(self):
        """ Initialize ROS2 subscriptions for robot position messages.
        """
        pass

    def initialize_publishers(self):
        """ Initialize ROS2 publishers for robot output messages. """
        
        # Initialize a publisher for the effector position.
        self._position_pub \
          = self.create_publisher(Point, 
                                  '/robot/feedback/position',
                                  1)
        
        # Initialize a publisher for the effector gripper.
        self._gripper_pub \
            = self.create_publisher(Float64,
                                    '/robot/feedback/gripper_angle',
                                    1)
        
        
    def initialize_parameters(self):
        """ Declare ROS2 parameters used by the node.
        """

        # Define a sample interval in seconds. This determines the frequency 
        # at which commands will be sent and received. Values less than 50Hz 
        # are supported by the Raspbery Pi Pico W
        self.declare_parameter('sample_interval_s', 0.25)
        
        # Define the serial port name and baudrate. This is used to establish communication to the serial device
        self.declare_parameter('serial_port', 'COM5')
        self.declare_parameter('baud_rate', 9600)
        
        
    def initialize_timer(self):
        """ Initialize a timer for the main update callback. """
        
        rate = self.get_parameter('sample_interval_s').value
        self._update_timer = self.create_timer(timer_period_sec=rate,
                                               callback=self.update)
                                               
        self.logger("Sample timer initialized. Current interval (s): {}".format(str(rate)))

        
    def connect_to_serial(self, port, baud):
        """ Attempt to connect to serial port 
        """
        
        s = None
        #try:
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        s = serial.Serial(port=port, baudrate=baud, timeout=1)
        self.logger("Successful connection to device on port {}".format(port))
        #except:
        self.logger("Failure to connect to device on port {}".format(port))
        
        return s
    

    def update(self):
        """ Callback from timer which handels a majority of the robot controller communication
        """
        
        #Send the command to get robot info
        self.usb_serial.write("robotinfo".encode('utf8'))
        msg = self.usb_serial.read()
        
        # Parse the message to get the position and gripper components
        self._current_position, self._gripper_angle = self.parse_robot_info(msg)
        
        # Update ROS2 topics
        # Update the robot's current position
        msg = Point()
        msg.data = self._current_position
        self.position_pub.publish(msg)
        
        # Update the robot's current position
        msg = Int64()
        msg.data = self._gripper_angle
        self._gripper_pub.publish(msg)
        
    
    def parse_robot_info(self, data):
        """ Helper function to parse the messsage output from the robot after calling 'robotinfo'
        
        
        =================================== Robot Info ===================================
        Current joint state: [90, 90, 84, 90, 90, 90]
        Current pose:   cords: [x: 285.00000, y: 0.00000, z: 215.00000]
          angles: [Roll: 0.00000, Pitch: -0.00000, Yaw:0.00000]
          tool: 100
        ==================================================================================

        """
        x = re.search("(?<=x: )(.*)(?=, y)", data)
        x = re.search("(?<=y: )(.*)(?=, z)", data)
        y = re.search("(?<=z: )(.*)(?=])", data)
        gripper = re.search("(?<=tool: )(.*)", data)
        
        return [x, y, z], gripper
        