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
from railway_interfaces.msg import TrackProtocolDefines 
from railway_interfaces.msg import DccSpeedstepsDefines



class locomotive_control(Node):

    def __init__(self, locomotive_descr, control_publisher, locomotive_images_path):
        #super().__init__('railtrackgui')

        self.locomotive_descr = locomotive_descr
        self.control_publisher = control_publisher;
        self.locomotive_msg = LocomotiveControl()
        match locomotive_descr['protocol']:
            case "ROS":
                self.locomotive_msg.address = locomotive_descr['address']
                self.locomotive_msg.protocol = TrackProtocolDefines.PROTOCOL_ROS
                self.number_of_functions = 4
            case "MM1":
                self.locomotive_msg.address = locomotive_descr['address']
                self.locomotive_msg.protocol = TrackProtocolDefines.PROTOCOL_MM1
                self.number_of_functions = 4
            case "MM2":    
                self.locomotive_msg.address = locomotive_descr['address']
                self.locomotive_msg.protocol = TrackProtocolDefines.PROTOCOL_MM2
                self.number_of_functions = 4
            case "DCC":
                self.locomotive_msg.address = locomotive_descr['address']
                self.locomotive_msg.protocol = TrackProtocolDefines.PROTOCOL_DCC
                self.number_of_functions = 16
                match locomotive_descr['speed_steps']:
                    case 128:
                        self.locomotive_msg.dcc_speed_step = DccSpeedstepsDefines.DCC_SPEED_STEP_128
                    case 28:
                        self.locomotive_msg.dcc_speed_step = DccSpeedstepsDefines.DCC_SPEED_STEP_28
                    case 14:
                        self.locomotive_msg.dcc_speed_step = DccSpeedstepsDefines.DCC_SPEED_STEP_14
                    case _:
                        self.locomotive_msg.dcc_speed_step = DccSpeedstepsDefines.DCC_SPEED_STEP_128 # default
            case "MFX":
                self.locomotive_msg.address = locomotive_descr['address']
                self.locomotive_msg.protocol = TrackProtocolDefines.PROTOCOL_MFX
                self.number_of_functions = 32
            case _:
                pass

        self.speed = 0
        self.max_speed = 1000;
        self.max_speed = locomotive_descr['max_speed']
        self.increment_speed_step = 100
        self.increment_speed_step = locomotive_descr['increment_speed_step']
        self.decrement_speed_step = 100
        self.decrement_speed_step = locomotive_descr['decrement_speed_step']
        self.function_status = []
        self.function_numbers = []

        with ui.card():
            text = str(locomotive_descr['type']) + ': ' + str(locomotive_descr['name'])
            ui.label(text)
            image = locomotive_images_path + "/" + locomotive_descr["image"]

            ui.image(image).classes('w-64')
            with ui.grid(columns=3):
                self.decrement_button = ui.button(icon = 'remove', on_click=lambda:self.set_decrement_speed()) 
                self.speed_slider = ui.slider(min=0, max=1000, value=50, on_change=lambda:self.set_speed()).props('label-always')
                self.speed_slider.on(type = 'update:model-valuex', leading_events = False, trailing_events = False, throttle = 5.0) 
                self.increment_button = ui.button(icon = 'add', on_click=lambda:self.set_increment_speed()) 
                self.direction_button = ui.button('FORWARD', on_click=lambda:self.set_direction()).classes('drop-shadow bg-red')
                self.stop_button = ui.button('STOP', on_click=lambda:self.stop())
                try:
                    functions = locomotive_descr['functions']
                    with ui.dialog() as dialog, ui.card():
                        self.set_functions = []
                        self.function_buttons = []
                        self.once = []
                        ui.label('Functions')
                        
                        with ui.grid(columns=2):
                            for function in functions:
                                number = function['number']
                                set_function = partial(self.set_function, number)
                                self.set_functions.append(set_function)
                                text = "F" + str(number) + " " + function['title']
                                try:
                                    icon = function['icon']
                                    button = ui.button(text, icon = icon, on_click=set_function).classes('drop-shadow bg-red')
                                except KeyError:
                                    button = ui.button(text, on_click=set_function).classes('drop-shadow bg-red')
                                self.function_buttons.append(button)
                                self.function_status.append(False)
                                self.function_numbers.append(number)
                                try:
                                    once = function['once'] 
                                except KeyError:
                                    once = False;
                                self.once.append(once)         
                                # see icons https://fonts.google.com/icons
                        ui.button('Close', on_click=dialog.close)
                    ui.button('Functions', on_click=dialog.open)
                except KeyError:
                    pass
    
    
    def set_speed(self):
        self.locomotive_msg.command = LocomotiveControl.SET_SPEED
        self.locomotive_msg.speed = self.speed_slider.value
        self.control_publisher.publish(self.locomotive_msg)
        notify_text = "Set Speed, Protocol: " + self.locomotive_descr['protocol'] \
                    + ", Loc ID: " + str(self.locomotive_descr['address'])         \
                    + ", Speed: " + str(self.speed_slider.value)
        ui.notify(notify_text)                 
        pass

    def set_increment_speed(self):
        new_speed = self.speed + self.increment_speed_step
        if new_speed > self.max_speed:
            new_speed = self.max_speed
        self.speed = new_speed
        # generates callback
        self.speed_slider.value = self.speed  
        pass

    def set_decrement_speed(self):
        new_speed = self.speed - self.decrement_speed_step
        if new_speed < 0:
            new_speed = 0
        self.speed = new_speed
        # generates callback
        self.speed_slider.value = self.speed          
        pass

    def set_direction(self):
        #print(self.locomotive_msg)
        self.locomotive_msg.command = LocomotiveControl.SET_DIRECTION
        self.locomotive_msg.speed = 0
        if(self.direction_button.text == 'FORWARD'):
            self.direction_button.text ='REVERSE'
            self.locomotive_msg.direction = LocomotiveControl.DIRECTION_FORWARD
        else:
            self.direction_button.text ='FORWARD'
            self.locomotive_msg.direction = LocomotiveControl.DIRECTION_REVERSE
        self.control_publisher.publish(self.locomotive_msg)
        notify_text = "Set Direction, Protocol: " + self.locomotive_descr['protocol'] \
            + ", Loc ID: " + str(self.locomotive_descr['address'])         \
            + ", Direction: " + self.direction_button.text
        ui.notify(notify_text)
        #disable events
        self.speed_slider.disable()
        self.speed_slider.value = 0
        self.speed_slider.enable()
        self.speed = 0
        pass

    def stop(self):
        self.speed = 0
        self.speed_slider.value = self.speed
        pass

    def set_function(self, function_index):
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
        self.locomotive_msg.command = LocomotiveControl.SET_FUNCTION;
        self.locomotive_msg.function_index = function_index;
        self.locomotive_msg.function_state = self.function_status[index];
        self.control_publisher.publish(self.locomotive_msg);
        if(self.once[index]):
            self.function_status[index] = False
            self.function_buttons[index].classes('drop-shadow bg-red', remove='bg-green')
            self.locomotive_msg.function_state = self.function_status[index];
            self.control_publisher.publish(self.locomotive_msg);

        ui.notify(notify_text) 
        
        pass

    def set_status(self, status) -> None:
        #print("set_status_indicator")
        if((self.locomotive_msg.address == status.address) and (self.locomotive_msg.protocol == status.protocol)):
            #print(status)
            self.speed_slider.disable()
            self.speed_slider.value = status.speed
            self.speed = status.speed
            self.speed_slider.enable()
            if status.direction == LocomotiveState.DIRECTION_FORWARD:
                self.direction_button.text ='REVERSE'
            else:
                self.direction_button.text ='FORWARD'
            index = 0
            #print(self.function_numbers)
            for function_number in self.function_numbers:
                self.function_status[index] = status.function_state[function_number]
                if  self.function_status[index]:
                    self.function_buttons[index].classes('drop-shadow bg-red', remove='bg-green')
                else:
                    self.function_buttons[index].classes('drop-shadow bg-green', remove='bg-red')
                index = index + 1
