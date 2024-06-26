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
        with ui.tabs().classes('w-full') as tabs:
            self.locomotive_tab = ui.tab('Locomotive')
            self.turnout_tab = ui.tab('Turnout')
            self.cv_programming_tab = ui.tab('cv Programing')

        with ui.tab_panels(tabs, value=self.locomotive_tab).classes('w-full'):
            with ui.tab_panel(self.turnout_tab):
                with ui.grid(columns=4):
                    self.turnout_number = ui.number(label='Turnout Number', value=1, format='%i')
                    self.turnout_protocol_select = ui.select({1: 'DCC', 2: 'MM'}, value=1)                        
                    self.green_button = ui.button('Green', on_click=lambda: self.set_turnout(True)).classes('drop-shadow bg-green')
                    self.rood_button = ui.button('Red', on_click=lambda: self.set_turnout(False)).classes('drop-shadow bg-red')
            with ui.tab_panel(self.locomotive_tab):
                with ui.grid(columns=4):
                    self.locomotive_address = ui.number(label='Locomotive address', value=1, format='%i')
                    self.locomotive_protocol_select = ui.select({1: 'MFX', 2: 'DCC', 3:'MM1', 4:'MM2'}, value=1)                        
                    with ui.grid(columns=3):
                        self.decrement_button = ui.button(icon = 'remove', on_click=lambda:self.set_decrement_speed()) 
                        self.speed_slider = ui.slider(min=0, max=1000, value=50, on_change=lambda:self.set_speed())
                        self.speed_slider.on(type = 'update:model-valuex', leading_events = False, trailing_events = False, throttle = 5.0) 
                        self.increment_button = ui.button(icon = 'add', on_click=lambda:self.set_increment_speed()) 
                        self.direction_button = ui.button('FORWARD', on_click=lambda:self.set_direction()).classes('drop-shadow bg-red')
                        self.stop_button = ui.button('STOP', on_click=lambda:self.stop())
                        with ui.dialog() as dialog, ui.card():
                            self.set_functions = []
                            self.function_buttons = []
                            ui.label('Functions')
                            
                            with ui.grid(columns=2):
                                for number in range(1, 32):
                                    set_function = partial(self.set_function, number)
                                    self.set_functions.append(set_function)
                                    text = "F" + str(number)
                                    button = ui.button(text, on_click=set_function).classes('drop-shadow bg-red')
                                    self.function_buttons.append(button)
                                    # see icons https://fonts.google.com/icons
                            ui.button('Close', on_click=dialog.close)
                        ui.button('Functions', on_click=dialog.open)


        pass

    def set_speed():
        pass
    def set_decrement_speed():
        pass
    def set_increment_speed():
        pass
    def stop():
        pass
    def set_direction():
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

    def set_turnout(self, control):
        pass

    def set_function(self, function_index):
        if 0:
            found = False
            index = 0
            for number in self.function_numbers:
                if number == function_index:
                    found = True
                    break;
                index = index + 1
            if not found:
                return
            #ui.notify(str(index))
            if  self.function_status[index]:
                self.function_status[index] = False
                self.function_buttons[index].classes('drop-shadow bg-red', remove='bg-green')
            else:
                self.function_status[index] = True
                self.function_buttons[index].classes('drop-shadow bg-green', remove='bg-red')

            notify_text = "Set Function " + str(function_index) + ": " + str(self.function_status[index])
            self.locomotive_msg.command = LocomotiveControl().__class__.SET_FUNCTION;
            self.locomotive_msg.function_index = function_index;
            self.locomotive_msg.function_state = self.function_status[index];
            self.control_publisher.publish(self.locomotive_msg);
            if(self.once[index]):
                self.function_status[index] = False
                self.function_buttons[index].classes('drop-shadow bg-red', remove='bg-green')
                self.locomotive_msg.function_state = self.function_status[index];
                self.control_publisher.publish(self.locomotive_msg);

            ui.notify(notify_text)