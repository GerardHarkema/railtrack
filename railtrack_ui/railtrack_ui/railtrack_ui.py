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
from railway_interfaces.msg import PowerControl   
from railway_interfaces.msg import PowerState 
from railway_interfaces.msg import SceneryControl  
from railway_interfaces.msg import SceneryState   
from railway_interfaces.msg import TrackConfig   

from turnout_control import turnout_control
from locomotive_control import locomotive_control
from railtracklayout_control import railtracklayout_control
from scenery_control import scenery_control
from maintenance_control import maintenance_control

import re

class RailTrackNode(Node):

    def __init__(self) -> None:
        super().__init__('railtrack_gui')

        # Declare the 'package_directory' parameter with a default value (empty string)
        self.declare_parameter('package_directory', '')

        # Retrieve the 'package_directory' parameter value
        self.package_directory = self.get_parameter('package_directory').get_parameter_value().string_value
        workspace_root = Path(self.package_directory).parents[3]  # Go up three levels (install/ -> workspace/)
        self.config_directory = str(workspace_root / 'src' / 'railtrack' / 'config')

        self.railtrack_ui_directory = str(workspace_root / 'src' / 'railtrack' / 'railtrack_ui' )
        self.program_settings_file = self.railtrack_ui_directory + '/' + 'settings.json'

        # Read programm settings
        with open(self.program_settings_file, 'r', encoding='utf-8') as f:
            self.program_settings = json.load(f)

        # Read track configuration
        self.config_file =  self.railtrack_ui_directory + "/" + self.program_settings["config_file"]           
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.track_config = json.load(f)

        else:
            print(f"File {self.config_file} does not exist.")
            return

        self.locomotive_images_path = self.config_directory + '/' + self.track_config["locomotive_images_path"]
        #self.get_logger().info(f"Loc path {self.locomotive_images_path}")

        self.railtracklayout_images_path = self.config_directory + '/' + self.track_config["railtrack_layout_image"]
        #self.get_logger().info(f"railtracklayout_images_path {self.railtracklayout_images_path}")

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
        self.power_status_subscription = self.create_subscription(PowerState, topic,  self.power_status_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/power_control"
        self.power_control_publisher = self.create_publisher(PowerControl, topic,  1)

        topic = "/railtrack/track_config"
        self.track_config_publisher = self.create_publisher(TrackConfig, topic,  1)

        topic = "/railtrack/scenery/status"
        self.scenery_status_subscription = self.create_subscription(SceneryState, topic,  self.scenery_status_callback, qos_profile=self.qos_profile)

        topic = "/railtrack/scenery/control"
        self.scenery_control_publisher = self.create_publisher(SceneryControl, topic,  1)

        self.power_msg = PowerControl()
        self.power_msg.enable = False

        self.turnoutsui= []
        self.locomotivesui = []
        self.turnouts = []
        self.sceneryui = []
        with Client.auto_index_client:
            with ui.tabs().classes('w-full') as tabs:

                try:
                    tmp = self.track_config["Locomotives"]
                    self.locomotives_tab = ui.tab('Locomotives')
                    first_tab = self.locomotives_tab
                except KeyError:
                    pass

                try:
                    tmp = self.track_config["Turnouts"]
                    self.turnouts_tab = ui.tab('Turnouts')
                    if not 'first_tab' in locals():
                        first_tab = self.turnouts_tab
                except KeyError:
                    pass

                try:
                    tmp = self.track_config["Scenerys"]
                    self.scenery_tab = ui.tab('Scenerys')
                except KeyError:
                    pass

                try:
                    tmp = self.track_config["railtrack_layout_image"]
                    self.tracklayouts_tab = ui.tab('Track Layout')
                except KeyError:
                    pass

                try:
                    tmp = self.track_config["display_maintenance_tab"]
                    if tmp:
                        self.maintenance_tab = ui.tab('Maintenance')
                except KeyError:
                    pass

            with ui.tab_panels(tabs, value=first_tab).classes('w-full'):
                try:
                    tmp = self.track_config["Turnouts"]
                    with ui.tab_panel(self.turnouts_tab):
                        with ui.grid(columns=3):

                            try:
                                for turnout in self.track_config["Turnouts"]:
                                    self.turnouts.append(turnout)
                            except KeyError:
                                pass

                            #self.turnouts.sort()
                            for turnout in self.turnouts:
                                tc = turnout_control(turnout, self.turnout_control_publisher)
                                self.turnoutsui.append(tc)
                except KeyError:
                    pass

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
                    tmp = self.track_config["Scenerys"]                
                    with ui.tab_panel(self.scenery_tab):
                        with ui.grid(columns=3):
                            for sc in self.track_config['Scenerys']:
                                scenery = scenery_control(sc, self.scenery_control_publisher)
                                self.sceneryui.append(scenery)
                except KeyError:
                    pass


                try:
                    tmp = self.track_config["railtrack_layout_image"]
                    with ui.tab_panel(self.tracklayouts_tab):
                        #railtracklayout_image_file = self.locomotive_images_path + "/"+ self.track_config["railtrack_layout_image"]
                        self.track_control = railtracklayout_control(self.track_config["Turnouts"], self.railtracklayout_images_path, self.turnout_control_publisher)
                        pass
                except KeyError:
                    pass

                with ui.tab_panel(self.maintenance_tab):
                    self.maintenance_control = maintenance_control(self.track_config  ,self.track_config_publisher)
            with ui.grid(columns=3):
                with ui.card():
                    ui.label("Control")
                    ui.label("Track Power")
                    self.power_button = ui.button('STOP', on_click=lambda:self.power()).classes('drop-shadow bg-red')
                    ui.label("Connection state (Flash)")
                    self.active = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')
                with ui.card():
                    ui.label("Status")
                    with ui.grid(columns=3):                
                        with ui.card():
                            ui.label("Current")
                            self.current = ui.label("0.0 A")
                            ui.label("Overload")
                            self.current_overload = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')
                        with ui.card():
                            ui.label("Voltage")
                            self.voltage = ui.label("0.0 V")
                            ui.label("Overload")
                            self.voltage_overload = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')
                        with ui.card():
                            ui.label("Temperature")
                            self.temperature = ui.label("0.0 °C")
                            ui.label("Overload")
                            self.temperature_overload = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')
            self.active_status = False;

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

    def scenery_status_callback(self, status):
        #print(status)
        for scenery in self.sceneryui:
            scenery.set_status(status)

    def power_status_callback(self, status):
        if status.state:
            self.power_msg.enable = True
            self.power_button.classes('drop-shadow bg-red', remove='bg-green')
            self.power_button.text = 'STOP'
        else:
            self.power_msg.enable = False
            self.power_button.classes('drop-shadow bg-green', remove='bg-red') 
            self.power_button.text = 'ENABLE'
        if(self.active_status):
            self.active.classes('text-green', remove='text-red')
            self.active_status = False
        else:
            self.active.classes('text-red', remove='text-green')
            self.active_status = True
        text = str(round(status.current, 1)) + " A"
        self.current.text = text
        text = str(round(status.voltage, 1)) + " V"
        self.voltage.text = text
        text = str(round(status.temperature, 1)) + " °C"
        self.temperature.text = text
        if(status.current_overload):
            self.current_overload.classes('text-red', remove='text-green')
        else:
            self.current_overload.classes('text-green', remove='text-red')
        
        if(status.voltage_overload):
            self.voltage_overload.classes('text-red', remove='text-green')
        else:
            self.voltage_overload.classes('text-green', remove='text-red')
        
        if(status.temperature_overload):
            self.temperature_overload.classes('text-red', remove='text-green')
        else:
            self.temperature_overload.classes('text-green', remove='text-red')
        
        pass

    def power(self):
        #ui.notify(self.power_button.text)
        if(self.power_button.text == 'STOP'):
            self.power_button.classes('drop-shadow bg-green', remove='bg-red') 
            self.power_button.text = 'ENABLE'
            self.power_msg.enable = False
        else:
            self.power_button.classes('drop-shadow bg-red', remove='bg-green')
            self.power_button.text = 'STOP'
            self.power_msg.enable = True
        self.power_control_publisher.publish(self.power_msg)  
        notify_text = "Power "
        if self.power_msg.enable:
            notify_text = notify_text + ": Enable"
        else:
            notify_text = notify_text + ": Disable"
        ui.notify(notify_text)
        pass

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main(args=None) -> None:

    rclpy.init(args=args)

    node = RailTrackNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
ui.run(title='Dorst central station')


