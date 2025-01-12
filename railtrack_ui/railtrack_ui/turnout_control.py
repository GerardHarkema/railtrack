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
from railway_interfaces.msg import TurnoutControl  
from railway_interfaces.msg import TurnoutState
from railway_interfaces.msg import TrackProtocolDefines 


class turnout_control(Node):

    def __init__(self, turnout_descr, control_publisher):
        self.turnout_msg = TurnoutControl()
        self.turnout_msg.number = turnout_descr['number']
        self.control_publisher = control_publisher

        match turnout_descr['protocol']:
            case "ROS":
                self.turnout_msg.protocol = TrackProtocolDefines.PROTOCOL_ROS
            case "MM1":
                self.turnout_msg.protocol = TrackProtocolDefines.PROTOCOL_MM1
            case "MM2":    
                self.turnout_msg.protocol = TrackProtocolDefines.PROTOCOL_MM2
            case "DCC":
                self.turnout_msg.protocol = TrackProtocolDefines.PROTOCOL_DCC
            case "MFX":
                self.turnout_msg.protocol = TrackProtocolDefines.PROTOCOL_MFX
            case _:
                pass

        with ui.card():
            text = 'Turnout ' + str(self.turnout_msg.number)
            ui.label(text)

            with ui.grid(columns=3):
                self.green_button = ui.button('Green', on_click=lambda: self.set_turnout(True)).classes('drop-shadow bg-green')
                self.rood_button = ui.button('Red', on_click=lambda: self.set_turnout(False)).classes('drop-shadow bg-red')
                self.led = ui.icon('fiber_manual_record', size='3em').classes('drop-shadow text-green')

    def __lt__(self, other):
         return self.turnout_msg.number < other.turnout_msg.number

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
        #print("set_status_indicator " + str(self.turnout_msg.number))
        #print(status.number)
        #print()
        if((self.turnout_msg.number == status.number) and (self.turnout_msg.protocol == status.protocol)):
            if status.state:
                self.led.classes('text-green', remove='text-red')
                text = 'Set turnout ' + str(self.turnout_msg.number)+ ": green" 
                #print(text)
            else:
                self.led.classes('text-red', remove='text-green')
                text = 'Set turnout ' + str(self.turnout_msg.number) + ": red" 
                #print(text)