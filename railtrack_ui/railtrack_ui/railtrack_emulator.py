#!/usr/bin/env python3
import math
import threading
from pathlib import Path

import rclpy
import os
import json

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from functools import partial

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool;
from railway_interfaces.msg import LocomotiveControl  
from railway_interfaces.msg import LocomotiveState  
from railway_interfaces.msg import TurnoutControl  
from railway_interfaces.msg import TurnoutState 

import threading
import time


class locomotive_emulator(Node):

    def __init__(self, locomotive_descr, state_publisher, locomotive_images_path):
        #super().__init__('railtrackgui')

        self.locomotive_descr = locomotive_descr
        self.state_publisher = state_publisher
        self.locomotive_msg = LocomotiveState()

        self.locomotive_msg.direction = False
        self.locomotive_msg.speed = 0

        match locomotive_descr['protocol']:
            case "MM1":
                self.locomotive_msg.address = locomotive_descr['address']
                self.number_of_functions = 4
            case "MM2":    
                self.locomotive_msg.address = locomotive_descr['address']
                self.number_of_functions = 4
            case "DCC":
                self.locomotive_msg.address = locomotive_descr['address'] + 0xC000
                self.number_of_functions = 16
            case "MFX":
                self.locomotive_msg.address = locomotive_descr['address'] + 0x4000
                self.number_of_functions = 32
            case _:
                pass

    def handle_control(self, control) -> None:
        if control.address == self.locomotive_msg.address:
            if control.command == LocomotiveControl().__class__.SET_SPEED:
                self.locomotive_msg.speed = control.speed
            elif control.command == LocomotiveControl().__class__.SET_DIRECTION:
                self.locomotive_msg.direction = control.direction
                self.locomotive_msg.speed = 0
            elif control.command == LocomotiveControl().__class__.SET_FUNCTION:
                pass
            else:
                pass
        pass

    def publish_status(self):
        self.state_publisher.publish(self.locomotive_msg)
        pass


class turnout_emulator(Node):

    def __init__(self, turnout_number, state_publisher):
        self.turnout_msg = TurnoutState()
        self.turnout_msg.number = turnout_number
        self.state_publisher = state_publisher
        self.turnout_number = turnout_number
        self.turnout_state = False;

        self.turnout_msgx = TurnoutControl()

    def handle_control(self, control) -> None:
        if control.number == self.turnout_number:
            self.turnout_state = control.state
        pass

    def publish_status(self):
        self.turnout_msg.state = self.turnout_state
        self.state_publisher.publish(self.turnout_msg)
        pass

class RailTrackEmulatorNode(Node):

    def __init__(self) -> None:
        super().__init__('railtrack_emulator')
        
        self.declare_parameter("config_file", "");
        self.config_file = self.get_parameter("config_file").get_parameter_value().string_value
        self.declare_parameter("locomotive_images_path", "");
        self.locomotive_images_path = self.get_parameter("locomotive_images_path").get_parameter_value().string_value
        self.declare_parameter("railtracklayout_images_path", "");
        self.railtracklayout_images_path = self.get_parameter("railtracklayout_images_path").get_parameter_value().string_value

        with open(self.config_file, 'r', encoding='utf-8') as f:
            self.track_config = json.load(f)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        topic = "/railtrack/turnout/status"
        self.turnout_status_publisher = self.create_publisher(TurnoutState, topic, 1)

        topic = "/railtrack/turnout/control"
        self.turnout_control_subscriber = self.create_subscription(TurnoutControl, topic, self.turnout_emulator_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/locomotive/status"
        self.locomotive_status_publisher = self.create_publisher(LocomotiveState, topic, 1)

        topic = "/railtrack/locomotive/control"
        self.locomotive_control_subscriber = self.create_subscription(LocomotiveControl, topic, self.locomotive_emulator_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/power_status"
        self.power_status_publisher = self.create_publisher(Bool, topic,  1)

        topic = "/railtrack/power_control"
        self.power_control_subscriber = self.create_subscription(Bool, topic,  self.power_emulator_callback, qos_profile=self.qos_profile)

        self.turnouts= []
        self.locomotives = []
        self.turnouts_tmp = []
        
        for turnout in self.track_config["Turnouts"]["railbox_controlled"]:
            self.turnouts_tmp.append(turnout["number"])   
        for turnout in self.track_config["Turnouts"]["ros_controlled"]:
            self.turnouts_tmp.append(turnout["number"])   
        self.turnouts_tmp.sort()
        for turnout in self.turnouts_tmp:
            tc = turnout_emulator(turnout, self.turnout_status_publisher)
            self.turnouts.append(tc)

        for loc in self.track_config['Locomotives']:
            locomotive = locomotive_emulator(loc, self.locomotive_status_publisher, self.locomotive_images_path)
            self.locomotives.append(locomotive)

        self.active_status = False;


        self.power_state = False

        self.sleep_time = 0.5 / (len(self.turnouts) + len(self.locomotives))
        threading.Thread(target=self.run_loop).start()
        

    def run_loop(self):
        while True:
            for turnout in self.turnouts:
                turnout.publish_status()
                time.sleep(self.sleep_time)
            for locomotive in self.locomotives:
                locomotive.publish_status()
                time.sleep(self.sleep_time)

    def turnout_emulator_callback(self, control):
        print("tc")
        #self.track_control.handle_control(status)
        for turnout in self.turnouts:
            turnout.handle_control(control)

    def locomotive_emulator_callback(self, control):
        print("lc")
        #print(status)
        for loc in self.locomotives:
            loc.handle_control(control)

    def power_emulator_callback(self, power):
        pass

    def power(self):
        pass


def main(args=None):
    rclpy.init()

    node = RailTrackEmulatorNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


if __name__ == '__main__':
    main()

