#!/usr/bin/env python3
import math
import threading
from pathlib import Path

import rclpy
import os
import json

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run, events
from functools import partial

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool;
from railway_interfaces.msg import LocomotiveControl  
from railway_interfaces.msg import LocomotiveState  
from railway_interfaces.msg import TurnoutControl  
from railway_interfaces.msg import TurnoutState  

from turnout_control import turnout_control
from locomotive_control import locomotive_control
from railtracklayout_control import railtracklayout_control




class RailTrackNode(Node):

    def __init__(self) -> None:
        super().__init__('railtrack_gui')
        
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
        self.turnout_status_subscription = self.create_subscription(TurnoutState, topic, self.turnout_status_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/turnout/control"
        self.turnout_control_publisher = self.create_publisher(TurnoutControl, topic, 1)

        topic = "/railtrack/locomotive/status"
        self.locomotive_status_subscription = self.create_subscription(LocomotiveState, topic, self.locomotive_status_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/locomotive/control"
        self.locomotive_control_publisher = self.create_publisher(LocomotiveControl, topic, 1)

        topic = "/railtrack/power_status"
        self.power_status_subscription = self.create_subscription(Bool, topic,  self.power_status_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/power_control"
        self.power_control_publisher = self.create_publisher(Bool, topic,  1)

        self.turnoutsui= []
        self.locomotivesui = []
        self.turnouts = []
        with Client.auto_index_client:
            with ui.tabs().classes('w-full') as tabs:
                try:
                    tmp = self.track_config["Locomotives"]
                    self.locomotives_tab = ui.tab('Locomotives')
                except KeyError:
                    pass

                self.turnouts_tab = ui.tab('Turnouts')

                try:
                    tmp = self.track_config["railtrack_layout_image"]
                    self.tracklayouts_tab = ui.tab('Track Layout')
                except KeyError:
                    pass
            with ui.tab_panels(tabs, value=self.turnouts_tab).classes('w-full'):
                with ui.tab_panel(self.turnouts_tab):
                    with ui.grid(columns=3):

                        for turnout in self.track_config["Turnouts"]["railbox_controlled"]:
                            self.turnouts.append(turnout["number"])   
                        for turnout in self.track_config["Turnouts"]["ros_controlled"]:
                            self.turnouts.append(turnout["number"])   
                        self.turnouts.sort()
                        for turnout in self.turnouts:
                            tc = turnout_control(turnout, self.turnout_control_publisher)
                            self.turnoutsui.append(tc)
                try:
                    tmp = self.track_config["Locomotives"]                
                    with ui.tab_panel(self.locomotives_tab):
                        with ui.grid(columns=3):
                            for loc in self.track_config['Locomotives']:
                                locomotive = locomotive_control(loc, self.locomotive_control_publisher, self.locomotive_images_path)
                                self.locomotivesui.append(locomotive)
                except KeyError:
                    pass
                try:
                    tmp = self.track_config["railtrack_layout_image"]
                    with ui.tab_panel(self.tracklayouts_tab):
                        railtracklayout_image_file = self.railtracklayout_images_path + "/"+ self.track_config["railtrack_layout_image"]
                        self.track_control = railtracklayout_control(self.track_config["Turnouts"], railtracklayout_image_file, self.turnout_control_publisher)
                        pass
                except KeyError:
                    pass
            self.power_button = ui.button('STOP', on_click=lambda:self.power()).classes('drop-shadow bg-red')
            self.active = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')
            self.active_status = False;


        self.power_state = False

    def turnout_status_callback(self, status):
        try:
            tmp = self.track_config["railtrack_layout_image"]
            self.track_control.set_status_indicator(status)
        except:
            pass
        for turnout in self.turnoutsui:
            turnout.set_status_indicator(status)

    def locomotive_status_callback(self, status):
        #print(status)
        for loc in self.locomotivesui:
            loc.set_status(status)

    def power_status_callback(self, power):
        if power.data:
            self.power_state = True
            self.power_button.classes('drop-shadow bg-red', remove='bg-green')
            self.power_button.text = 'STOP'
        else:
            self.power_state = False
            self.power_button.classes('drop-shadow bg-green', remove='bg-red') 
            self.power_button.text = 'ENABLE'
        if(self.active_status):
            self.active.classes('text-green', remove='text-red')
            self.active_status = False
        else:
            self.active.classes('text-red', remove='text-green')
            self.active_status = True


        #print("power_callback")
        pass

    def power(self):
        #ui.notify(self.power_button.text)
        msg = Bool()
        if(self.power_button.text == 'STOP'):
            self.power_button.classes('drop-shadow bg-green', remove='bg-red') 
            self.power_button.text = 'ENABLE'
            msg.data = False
            self.power_state = False
        else:
            self.power_button.classes('drop-shadow bg-red', remove='bg-green')
            self.power_button.text = 'STOP'
            msg.data = True
            self.power_state = True
        self.power_control_publisher.publish(msg)  
        notify_text = "Power "
        if(self.power_state):
            notify_text = notify_text + ": Enable"
        else:
            notify_text = notify_text + ": Disable"
        ui.notify(notify_text)
        pass

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:

    rclpy.init()

    node = RailTrackNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
ui.run(title='Dorst central station')


