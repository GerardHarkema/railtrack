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
from railway_interfaces.msg import SceneryControl  
from railway_interfaces.msg import SceneryState  


class scenery_control(Node):

    def __init__(self, scenery_descr, control_publisher):
        #super().__init__('railtrackgui')

        self.scenery_descr = scenery_descr
        self.control_publisher = control_publisher
        self.scenery_msg = SceneryControl()
        self.scenery_msg.number = self.scenery_descr['number']
        self.max_brightness = 255
        self.max_brightness_slider = 100
        self.increment_decrement_brightness_step = 5



        with ui.card():
            text = str(scenery_descr['name'])
            ui.label(text)

            with ui.grid(columns=3):
                self.decrement_button = ui.button(icon = 'remove', on_click=lambda:self.set_decrement_brightness()) 
                self.brightness_slider = ui.slider(min=0, max=self.max_brightness_slider, value=50, on_change=lambda:self.set_brightness())
                self.brightness_slider.on(type = 'update:model-valuex', leading_events = False, trailing_events = False, throttle = 0)#5.0) 
                self.increment_button = ui.button(icon = 'add', on_click=lambda:self.set_increment_brightness()) 
                self.off_button = ui.button('OFF', on_click=lambda:self.off())
                if 0:
                    try:
                        functions = scenery_descr['functions']
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
    
    
    def set_brightness(self):
        brightness = int((self.brightness_slider.value/100) * self.max_brightness)
        self.scenery_msg.brightness = brightness
        self.scenery_msg.mode = self.status.mode
        self.scenery_msg.color = self.status.color
        self.control_publisher.publish(self.scenery_msg)
        notify_text = "Set Brightness of "  \
                    + str(self.scenery_descr['name']) \
                    + " to " \
                    + str(self.brightness_slider.value)
        ui.notify(notify_text)  


    def set_increment_brightness(self):
        brightness = self.brightness_slider.value + self.increment_decrement_brightness_step
        if brightness > self.max_brightness_slider:
            brightness = self.max_brightness_slider
        # generates callback
        self.brightness_slider.value = brightness  
        pass

    def set_decrement_brightness(self):
        brightness = self.brightness_slider.value - self.increment_decrement_brightness_step
        if brightness < 0:
            brightness = 0
        # generates callback
        self.brightness_slider.value = brightness  

    def off(self):
        # generates callback
        self.brightness_slider.value = 0

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
            self.scenery_msg.command = LocomotiveControl().__class__.SET_FUNCTION;
            self.scenery_msg.function_index = function_index;
            self.scenery_msg.function_state = self.function_status[index];
            self.control_publisher.publish(self.scenery_msg);
            if(self.once[index]):
                self.function_status[index] = False
                self.function_buttons[index].classes('drop-shadow bg-red', remove='bg-green')
                self.scenery_msg.function_state = self.function_status[index];
                self.control_publisher.publish(self.scenery_msg);

            ui.notify(notify_text) 
            
            pass

    def set_status(self, status) -> None:
        if(status.number == int(self.scenery_descr['number'])):
            self.status = status
            brightness = int((status.brightness/self.max_brightness) * self.max_brightness_slider)
            #print(brightness)
            self.brightness_slider.disable()
            #self.brightness_slider.value = brightness
            self.brightness_slider.enable()
