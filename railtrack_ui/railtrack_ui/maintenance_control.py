#!/usr/bin/env python3
import math
import threading
from pathlib import Path

import rclpy
import os
import json

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run
from functools import partial

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool;
from railway_interfaces.msg import LocomotiveControl  
from railway_interfaces.msg import LocomotiveState  


class maintenance_control(Node):
    def __init__(self):
        with ui.card():
            ui.label("Track maintenance")
            self.maintenance_button = ui.button('ENABLE', on_click=lambda:self.maintenance()).classes('drop-shadow bg-red')

        pass

    def maintenance(self):
        #ui.notify(self.maintenance_button.text)
        if(self.maintenance_button.text == 'DISABLE'):
            self.maintenance_button.classes('drop-shadow bg-green', remove='bg-red') 
            self.maintenance_button.text = 'ENABLE'
            self.maintenance_msg.enable = False
        else:
            self.maintenance_button.classes('drop-shadow bg-red', remove='bg-green')
            self.maintenance_button.text = 'DISABLE'
            self.maintenance_msg.enable = True
        #self.maintenance_control_publisher.publish(self.maintenance_msg)  
        notify_text = "maintenance "
        if self.maintenance_msg.enable:
            notify_text = notify_text + ": Enable"
        else:
            notify_text = notify_text + ": Disable"
        ui.notify(notify_text)
        pass