from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from lolo_msgs.msg import Topics as LoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )


    mqtt_bridge_node = Node(
        package='evolo_mqtt_bridge',
        namespace=robot_ns,
        executable='bridge',
        name="evolo_mqtt_bridge",
        parameters=[]
    )

    mqtt_odom_init_node = Node(
        package='evolo_mqtt_to_odom',
        namespace=robot_ns,
        executable='odom_initializer',
        name="evolo_odom_initializer",
        parameters=[{"update_rate": 0.1,
                     "verbose": True
                     }]
    )

    mqtt_odom_node = Node(
        package='evolo_mqtt_to_odom',
        namespace=robot_ns,
        executable='mqtt_odom',
        name="evolo_odom_node",
        parameters=[{"correct_meridian_convergence" : True,
                     "publish_tf" : True,
                     "output_rate" : 5.0,
                     "verbose_setup" : False,
                     "verbose_conversion" : False
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
        mqtt_bridge_node,
        mqtt_odom_init_node,
        mqtt_odom_node
    ])
