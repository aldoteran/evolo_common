
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

from evolo_msgs.msg import Topics as evoloTopics
from evolo_msgs.msg import CaptainState

import random
import json





class Evolo_captain_interface(Node):

    def __init__(self):
        super().__init__('Evolo_captain_interface')

        #TODO ROSPARAM

        # Create ROS publisher
        self.publisher_ = self.create_publisher(CaptainState, evoloTopics.EVOLO_CAPTAIN_STATE, 10)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(String,evoloTopics.EVOLO_CAPTAIN_FROM, self.captain_callback,10)
        self.subscription # prevent unused variable warning


    def captain_callback(self, msg:String):
        #self.get_logger().info(f"Received ROS message '{msg}'")
        
        try:
            data = json.loads(msg.data)
            if("state" in data.keys()):
                state = data["state"]
                #self.get_logger().info(f"Evolo state: {state}")

                msg = CaptainState()

                msg.lat = float(state[0])
                msg.lon = float(state[1])
                msg.sog = float(state[2])
                msg.cog = float(state[3])
                msg.roll = float(state[4])
                msg.pitch = float(state[5])
                msg.ms = float(state[6])
                msg.freq = float(state[7])
                msg.hdop = float(state[8])
                msg.health_percent = float(state[9])
                msg.mode = float(state[10])
                msg.cef = float(state[11])
                msg.voltage = float(state[12])
                msg.current = float(state[13])
                msg.percent_throttle = float(state[14])
                msg.altitude = float(state[15])
                msg.virtual_elevator = float(state[16])
                msg.virtual_aileron = float(state[17])
                msg.alt_aim = float(state[18])
                msg.healt_bits = state[19]
                msg.elevator_bias = float(state[20])
                msg.aileron_bias = float(state[21])
                msg.status_bits = state[22]
                msg.hs = float(state[23])
                msg.pitch_aim = float(state[24])
                msg.roll_aim = float(state[25])
                msg.sog_aim = float(state[26])
                msg.four_g_data = float(state[27])
                msg.free_mbon_sd = float(state[28])
                msg.speed_limit = float(state[29])
                msg.left_throttle_couch = float(state[30])
                msg.right_throttle_couch = float(state[31])
                msg.status_bits2 = state[32]
                msg.radar1_confidence = float(state[33])
                msg.radar2_confidence = float(state[34])
                msg.roll_rate = float(state[35])
                msg.pitch_rate = float(state[36])

                # Health bits
                bits = msg.healt_bits
                msg.health_loop_freq = (bits & 0b0000000000000001) != 0
                msg.health_voltage = (bits & 0b0000000000000010) != 0
                msg.health_hdop = (bits & 0b0000000000000100) != 0
                msg.health_gps_rate = (bits & 0b0000000000001000) != 0
                msg.health_imu = (bits & 0b0000000000010000) != 0
                msg.health_radar = (bits & 0b0000000000100000) != 0
                msg.health_tilt = (bits & 0b0000000001000000) != 0
                msg.health_radar_x = (bits & 0b0000000010000000) != 0
                msg.health_pitch = (bits & 0b0000000100000000) != 0
                msg.health_roll = (bits & 0b0000001000000000) != 0
                msg.health_ax = (bits & 0b0000010000000000) != 0
                msg.health_throttle = (bits & 0b0000100000000000) != 0
                msg.health_time_lock = (bits & 0b0001000000000000) != 0
                msg.health_hil = (bits & 0b0010000000000000) != 0
                msg.health_sd_memory = (bits & 0b0100000000000000) != 0
                msg.health_network_timeout = (bits & 0b1000000000000000) != 0



                # StatusBits
                bits = msg.status_bits
                msg.status_throttle_disarmed = (bits & 0b0000000000000001) != 0
                msg.status_throttle_manual = (bits & 0b0000000000000010) != 0
                msg.status_throttle_auto = (bits & 0b0000000000000100) != 0
                msg.status_guidance_manual = (bits & 0b0000000000001000) != 0
                msg.status_guidance_target = (bits & 0b0000000000010000) != 0
                msg.status_guidance_route = (bits & 0b0000000000100000) != 0
                msg.status_paused = (bits & 0b0000000001000000) != 0
                msg.status_hil = (bits & 0b0000000010000000) != 0
                msg.status_sim = (bits & 0b0000000100000000) != 0
                msg.status_telemetry_4g = (bits & 0b0000001000000000) != 0
                msg.status_rc_required = (bits & 0b0000010000000000) != 0
                msg.status_route_finished = (bits & 0b0000100000000000) != 0
                msg.status_route_looping = (bits & 0b0001000000000000) != 0
                msg.status_route_ongoing = (bits & 0b0010000000000000) != 0
                msg.status_joystick_present = (bits & 0b0100000000000000) != 0
                msg.status_rc_active = (bits & 0b1000000000000000) != 0
                msg.status_radar1_active = (bits & 0b10000000000000000) != 0
                msg.status_radar2_active = (bits & 0b100000000000000000) != 0
                msg.status_gamepad_xxx = (bits & 0b1000000000000000000) != 0
                msg.status_allow_backseat_ctrl = (bits & 0b10000000000000000000) != 0

                self.publisher_.publish(msg)
                '''
                self.get_logger().info(f"lat: {msg.lat}")
                self.get_logger().info(f"lon: {msg.lon}")
                self.get_logger().info(f"sog: {msg.sog}")
                self.get_logger().info(f"cog: {msg.cog}")
                self.get_logger().info(f"roll: {msg.roll}")
                self.get_logger().info(f"pitch: {msg.pitch}")
                self.get_logger().info(f"ms: {msg.ms}")
                self.get_logger().info(f"freq: {msg.freq}")
                self.get_logger().info(f"hdop: {msg.hdop}")
                self.get_logger().info(f"health_percent: {msg.health_percent}")
                self.get_logger().info(f"mode: {msg.mode}")
                self.get_logger().info(f"cef: {msg.cef}")
                self.get_logger().info(f"voltage: {msg.voltage}")
                self.get_logger().info(f"current: {msg.current}")
                self.get_logger().info(f"percent_throttle: {msg.percent_throttle}")
                self.get_logger().info(f"altitude: {msg.altitude}")
                self.get_logger().info(f"virtual_elevator: {msg.virtual_elevator}")
                self.get_logger().info(f"virtual_aileron: {msg.virtual_aileron}")
                self.get_logger().info(f"alt_aim: {msg.alt_aim}")
                self.get_logger().info(f"healt_bits: {msg.healt_bits}")
                self.get_logger().info(f"elevator_bias: {msg.elevator_bias}")
                self.get_logger().info(f"aileron_bias: {msg.aileron_bias}")
                self.get_logger().info(f"status_bits: {msg.status_bits}")
                self.get_logger().info(f"hs: {msg.hs}")
                self.get_logger().info(f"pitch_aim: {msg.pitch_aim}")
                self.get_logger().info(f"roll_aim: {msg.roll_aim}")
                self.get_logger().info(f"sog_aim: {msg.sog_aim}")
                self.get_logger().info(f"four_g_data: {msg.four_g_data}")
                self.get_logger().info(f"free_mbon_sd: {msg.free_mbon_sd}")
                self.get_logger().info(f"speed_limit: {msg.speed_limit}")
                self.get_logger().info(f"left_throttle_couch: {msg.left_throttle_couch}")
                self.get_logger().info(f"right_throttle_couch: {msg.right_throttle_couch}")
                self.get_logger().info(f"status_bits2: {msg.status_bits2}")
                self.get_logger().info(f"radar1_confidence: {msg.radar1_confidence}")
                self.get_logger().info(f"radar2_confidence: {msg.radar2_confidence}")
                self.get_logger().info(f"roll_rate: {msg.roll_rate}")
                self.get_logger().info(f"pitch_rate: {msg.pitch_rate}")
                self.get_logger().info(f"----------------------------------")

                self.get_logger().info(f"health_loop_freq: {msg.health_loop_freq}")
                self.get_logger().info(f"health_voltage: {msg.health_voltage}")
                self.get_logger().info(f"health_hdop: {msg.health_hdop}")
                self.get_logger().info(f"health_gps_rate: {msg.health_gps_rate}")
                self.get_logger().info(f"health_imu: {msg.health_imu}")
                self.get_logger().info(f"health_radar: {msg.health_radar}")
                self.get_logger().info(f"health_tilt: {msg.health_tilt}")
                self.get_logger().info(f"health_radar_x: {msg.health_radar_x}")
                self.get_logger().info(f"health_pitch: {msg.health_pitch}")
                self.get_logger().info(f"health_roll: {msg.health_roll}")
                self.get_logger().info(f"health_ax: {msg.health_ax}")
                self.get_logger().info(f"health_throttle: {msg.health_throttle}")
                self.get_logger().info(f"health_time_lock: {msg.health_time_lock}")
                self.get_logger().info(f"health_hil: {msg.health_hil}")
                self.get_logger().info(f"health_sd_memory: {msg.health_sd_memory}")
                self.get_logger().info(f"health_network_timeout: {msg.health_network_timeout}")
                self.get_logger().info(f"----------------------------------")
                self.get_logger().info(f"status_throttle_disarmed: {msg.status_throttle_disarmed}")
                self.get_logger().info(f"status_throttle_manual: {msg.status_throttle_manual}")
                self.get_logger().info(f"status_throttle_auto: {msg.status_throttle_auto}")
                self.get_logger().info(f"status_guidance_manual: {msg.status_guidance_manual}")
                self.get_logger().info(f"status_guidance_target: {msg.status_guidance_target}")
                self.get_logger().info(f"status_guidance_route: {msg.status_guidance_route}")
                self.get_logger().info(f"status_paused: {msg.status_paused}")
                self.get_logger().info(f"status_hil: {msg.status_hil}")
                self.get_logger().info(f"status_sim: {msg.status_sim}")
                self.get_logger().info(f"status_telemetry_4g: {msg.status_telemetry_4g}")
                self.get_logger().info(f"status_rc_required: {msg.status_rc_required}")
                self.get_logger().info(f"status_route_finished: {msg.status_route_finished}")
                self.get_logger().info(f"status_route_looping: {msg.status_route_looping}")
                self.get_logger().info(f"status_route_ongoing: {msg.status_route_ongoing}")
                self.get_logger().info(f"status_joystick_present: {msg.status_joystick_present}")
                self.get_logger().info(f"status_rc_active: {msg.status_rc_active}")
                self.get_logger().info(f"status_radar1_active: {msg.status_radar1_active}")
                self.get_logger().info(f"status_radar2_active: {msg.status_radar2_active}")
                self.get_logger().info(f"status_gamepad_xxx: {msg.status_gamepad_xxx}")
                self.get_logger().info(f"status_allow_backseat_ctrl: {msg.status_allow_backseat_ctrl}")
                self.get_logger().info("===================================")
                '''

        except Exception as e:
            print(e)

        #TODO parse message







def main(args=None):

    rclpy.init(args=args)

    captain_interface = Evolo_captain_interface()

    rclpy.spin(captain_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    captain_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()