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

# Region of interest
class rio_class():
    def __init__(self):
        self.xmin = 0;
        self.xmax = 0;
        self.ymin = 0;
        self.ymax = 0;

class point_class():
    def __init__(self):
        self.x = 0;
        self.y = 0;

class turnout_control_on_layout(Node):
    def __init__(self, turnout, image, turnout_control_publisher):
        self.image = image
        self.rios =[]
        self.red_contents = []
        self.green_contents = []
        self.status = False
        self.old_status = False
        self.first_ui_update = True

        self.turnout_msg = TurnoutControl()
        self.turnout_msg.number = turnout["number"]
        self.control_publisher = turnout_control_publisher

        try:
            for layout_position in turnout["layout_positions"]:
                x = layout_position["position"]["x"]
                y = layout_position["position"]["y"]
                radius = layout_position["size"]["radius"]
                
                color = 'Red'
                #red_content = image_content()
                red_content = f'<circle cx="{x}" cy="{y}" r="{radius}" fill="none" stroke="{color}" stroke-width="4" />'
                self.red_contents.append(red_content)
                
                color = 'Green'
                #green_content = image_content()
                green_content = f'<circle cx="{x}" cy="{y}" r="{radius}" fill="none" stroke="{color}" stroke-width="4" />'
                self.green_contents.append(green_content)

                self.image.content += red_content
                
                rio = rio_class()
                rio.xmin = x - radius
                rio.xmax = x + radius
                rio.ymin = y - radius
                rio.ymax = y + radius
                self.rios.append(rio)
                #print("Added")
        except:
            #print("ERROR")
            pass
    
    def is_point_in_rio(self, rio, point):
        if point.x >= rio.xmin and point.x <= rio.xmax:
            if point.y >= rio.ymin and point.y <= rio.ymax:
                return True
        return False
    
    def handle_mouse_event(self, point):
        found = False
        for rio in self.rios:
            found = self.is_point_in_rio(rio, point)
            if found:
                break
        if not found:
            return
        self.status = not self.status
        self.turnout_msg.state = self.status
        self.control_publisher.publish(self.turnout_msg)
        notify_text = "Set Turnout " + str(self.turnout_msg.number)
        if(self.turnout_msg.state):
            notify_text = notify_text + ": Green"
        else:
            notify_text = notify_text + ": Red"
        ui.notify(notify_text)

    def set_status_indicator(self, status):
        self.old_status = False
        self.first_ui_update = True

        if self.first_ui_update or self.old_status != status.state:
            if(self.turnout_msg.number == status.number):
                if status.state:
                    #for content in self.red_contents:
                    #    self.image.content -= content
                    for content in self.green_contents:
                        self.image.content += content
                else:
                    #for content in self.green_contents:
                    #    self.image.content -= content
                    for content in self.red_contents:
                        self.image.content += content
                self.status =  status.state
            self.first_ui_update = False
            self.old_status = status.state
        pass

class railtracklayout_control(Node):
    def __init__(self, turnouts, railtracklayout_image_filemage, turnout_control_publisher):
        self.turnout_control_publisher = turnout_control_publisher;
        self.turnouts = []
        self.layout = ui.interactive_image(railtracklayout_image_filemage, on_mouse=self.mouse_handler, events=['mousedown', 'mouseup'], cross=True).classes('w-512')
        for turnout in turnouts["c-type"]:
            tc = turnout_control_on_layout(turnout, self.layout, self.turnout_control_publisher)
            self.turnouts.append(tc)
        for turnout in turnouts["m-type"]:
            tc = turnout_control_on_layout(turnout, self.layout, self.turnout_control_publisher)
            self.turnouts.append(tc)
        self.notify_mouse_events = False
        self.switch = ui.switch('Show mouse click locations').bind_value(self, 'notify_mouse_events')    

    def mouse_handler(self, e: events.MouseEventArguments):
        if e.type == 'mousedown':
            for turnout in self.turnouts:
                point = point_class()
                point.x = e.image_x
                point.y = e.image_y
                turnout.handle_mouse_event(point)
            if self.notify_mouse_events:
                ui.notify(f'{e.type} at ({e.image_x:.1f}, {e.image_y:.1f})')
                
    def set_status_indicator(self, status):
        for turnout in self.turnouts:
            turnout.set_status_indicator(status)
