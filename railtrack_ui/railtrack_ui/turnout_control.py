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
from railway_interfaces.msg import TurnoutControl  
from railway_interfaces.msg import TurnoutState  


class turnout_control(Node):

    def __init__(self, turnout_number, control_publisher):
        self.turnout_msg = TurnoutControl()
        self.turnout_msg.number = turnout_number
        self.control_publisher = control_publisher

        self.turnout_number = turnout_number
        with ui.card():
            text = 'Turnout ' + str(self.turnout_number)
            ui.label(text)

            with ui.grid(columns=3):
                self.green_button = ui.button('Green', on_click=lambda: self.set_turnout(True)).classes('drop-shadow bg-green')
                self.rood_button = ui.button('Red', on_click=lambda: self.set_turnout(False)).classes('drop-shadow bg-red')
                self.led = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')

    def set_turnout(self, control) -> None:
        self.turnout_msg.state = control
        self.control_publisher.publish(self.turnout_msg)
        notify_text = "Set Turnout " + str(self.turnout_msg.number)
        if(control):
            notify_text = notify_text + ": Green"
        else:
            notify_text = notify_text + ": Red"
        ui.notify(notify_text)
    def set_status_indicator(self, status) -> None:
        #print("set_status_indicator")
        if(self.turnout_msg.number == status.number):
            if status.state:
                #print("set green")
                self.led.classes('text-green', remove='text-red')
                text = 'Set turnout ' + str(self.turnout_number)+ ": green" 
            else:
                #print("set red")
                self.led.classes('text-red', remove='text-green')
                text = 'Set turnout ' + str(self.turnout_number) + ": red" 