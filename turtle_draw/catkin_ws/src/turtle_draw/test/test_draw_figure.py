#!/usr/bin/env python

import unittest
from src import draw_figure
from src.draw_figure import Pose

import math

class Speed():
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular

    def __str__(self):
        return "Speed (linear: %.2f anguler: %.2f)" % (self.linear, self.anguler)

    def __repr__(self):
        return str(self)

class Drawer():
    def __init__(self, start_x, start_y, start_theta):
        self.pose = Pose(start_x, start_y, start_theta)
        self.speed = Speed(0, 0)
        self.drawing = False
        self.drawed_points = []

    def step(self):
        if self.drawing:
            self.drawed_points.append(self.pose)

        # First spin, then move forward
        self.pose.theta += self.speed.angular * 0.1 # The simulation advances 10 times more slowly
        self.pose.linear = Pose(self.pose.x + math.sin(self.pose.theta) * math.speed.linear * 0.1, self.pose.y + math.cos(self.pose.theta) * math.speed.linear * 0.1)

    def __str__(self):
        return "Drawer (%s, %s, drawing: %s)" % (self.pose, self.speed, self.drawing)

    def __repr__(self):
        return str(self)
