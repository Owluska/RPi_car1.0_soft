#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 12 15:56:29 2020

@author: root
"""

from LIBRARY.rpi_car import rpi_movement
from LIBRARY.rpi_telemetry import mb_telemetry
from time import sleep

car = rpi_movement()
car.init()

mb = mb_telemetry()
mb.init_all()

print(mb.telemetry())
print(car.move_forward())
sleep(2)

print(car.move_backward())
sleep(2)

print(car.turn_left())
sleep(2)

print(car.turn_right())
sleep(2)

print(car.turn_center())
print(car.stop())