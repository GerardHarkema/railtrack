#!/usr/bin/env python3
import math
import threading
from pathlib import Path
from time import sleep

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

from ws2812_effects import ws2812_effects


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

        self.max_effect_speed = 255
        self.max_effect_speed_slider = 100
        self.increment_decrement_effect_speed_step = 5


        with ui.card():
            text = str(scenery_descr['name'])
            ui.label(text)

            with ui.grid(columns=3):
                self.decrement_button = ui.button(icon = 'remove', on_click=lambda:self.set_decrement_brightness()) 
                self.brightness_slider = ui.slider(min=0, max=self.max_brightness_slider, value=50, on_change=lambda:self.set_brightness()).props('label-always')
                self.brightness_slider.on(type = 'update:model-valuex', leading_events = False, trailing_events = False, throttle = 5.0) 
                self.increment_button = ui.button(icon = 'add', on_click=lambda:self.set_increment_brightness()) 
                self.off_button = ui.button('OFF', on_click=lambda:self.off())
                #self.color_picker = ui.color_picker(on_pick=lambda e: button.classes(f'!bg-[{e.color}]'))

                with ui.dialog() as dialog, ui.card():
                    ui.label("Options")
                    with ui.grid(columns=3):
                        with ui.button(icon='colorize') as button:
                            self.color_picker = ui.color_picker(on_pick=lambda e: self.set_color(e.color))
                        with ui.dropdown_button('Effect mode', auto_close=True):
                            for effect in ws2812_effects:
                                #self.effectx.append(mode)
                                set_effect_function = partial(self.set_effect, effect)
                                ui.item(effect, on_click=set_effect_function)
                        self.effect_speed_slider = ui.slider(min=0, max=self.max_effect_speed_slider, value=50, on_change=lambda:self.set_effect_speed()).props('label-always')
                        self.effect_speed_slider.on(type = 'update:model-valuex', leading_events = False, trailing_events = False, throttle = 5.0) 

                    ui.button('Close', on_click=dialog.close)
                ui.button('Options', on_click=dialog.open)


    
    def set_brightness(self):
        brightness = int((self.brightness_slider.value/100) * self.max_brightness)
        self.scenery_msg.brightness = brightness
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
    
    def set_color(self, color):
 
        r = int(color[1:3], 16)
        g = int(color[3:5], 16)
        b = int(color[5:7], 16)

        self.scenery_msg.color.r = r
        self.scenery_msg.color.g = g
        self.scenery_msg.color.b = b
        self.control_publisher.publish(self.scenery_msg)
        notify_text = "Set RGB of "  \
            + str(self.scenery_descr['name']) \
            + " to R:" \
            + str(r)   \
            + ", G:" \
            + str(g)   \
            + ", B:" \
            + str(b)
        ui.notify(notify_text)  

        pass

    def set_effect(self, new_effect):
        i = 0
        for effects in ws2812_effects:
            if new_effect == effects:
                break
            i = i + 1

        self.scenery_msg.effect = i
        self.control_publisher.publish(self.scenery_msg)

        ui.notify(new_effect)
        pass

    def set_effect_speed(self):
        effect_speed = int((self.effect_speed_slider.value/100) * self.max_effect_speed)
        self.scenery_msg.effect_speed = effect_speed
        self.control_publisher.publish(self.scenery_msg)
        notify_text = "Set effect speed of "  \
                    + str(self.scenery_descr['name']) \
                    + " to " \
                    + str(self.effect_speed_slider.value)
        ui.notify(notify_text)  

    def set_status(self, status) -> None:
        if(status.number == int(self.scenery_descr['number'])):
            self.status = status
            self.scenery_msg.brightness = self.status.brightness
            self.scenery_msg.effect_speed = self.status.effect_speed
            self.scenery_msg.effect = self.status.effect
            self.scenery_msg.color = self.status.color
            brightness = int((status.brightness/self.max_brightness) * self.max_brightness_slider)
            #print(brightness)
            hex_color = f"#{self.status.color.r:02x}{self.status.color.g:02x}{self.status.color.b:02x}"
            self.color_picker.set_color(hex_color)

            self.brightness_slider.disable()
            #self.brightness_slider.value = brightness
            self.brightness_slider.enable()

