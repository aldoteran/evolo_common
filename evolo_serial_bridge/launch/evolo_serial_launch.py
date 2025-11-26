from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )


    serial_bridge_node = Node(
        package='evolo_serial_bridge',
        namespace=robot_ns,
        executable='bridge',
        name="evolo_serial_bridge",
        parameters=[]
    )

    captain_interface = Node(
        package='evolo_captain_interface',
        namespace=robot_ns,
        executable='interface',
        name="captain_interface",
        parameters=[]
    )

    serial_odom_init_node = Node(
        package='evolo_captain_to_odom',
        namespace=robot_ns,
        executable='odom_initializer',
        name="captain_odom_initializer",
        parameters=[{"update_rate": 0.1,
                     "verbose": True,
                     "captain_topic" : evoloTopics.EVOLO_CAPTAIN_STATE
                     }]
    )

    serial_odom_node = Node(
        package='evolo_captain_to_odom',
        namespace=robot_ns,
        executable='captain_odom',
        name="captain_odom_node",
        parameters=[{"correct_meridian_convergence" : True,
                     "publish_tf" : True,
                     "output_rate" : 5.0,
                     "verbose_setup" : False,
                     "verbose_conversion" : False,
                     "input_topic" : evoloTopics.EVOLO_CAPTAIN_STATE
                    }]
    )
    '''
    odom_splitter_node = Node(
        package='odom_splitter',
        namespace=robot_ns,
        executable='odom_splitter',
        name='odom_splitter',
        parameters=[{"robot_name": robot_ns
        }]
    )
    '''


    return LaunchDescription([
        robot_ns_launch_arg, 
        serial_bridge_node,
        serial_odom_init_node,
        serial_odom_node,
        captain_interface
    ])
