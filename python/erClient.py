#!/usr/bin/python
# import serial
# import time
# import struct
# import datetime
# import threading
# from collections import deque
from erSerial import *

# input_buffer = []
# stany odczytu bufora
# smieci
# trafiono na E
# frafiono na R
# odczyt headera
# odczyt danych

# class Command():
#         def __init__(self):
#                 pass

class erClient():
        def __init__(self, port, baudrate, node_id):
                self.__er_serial = erSerial(port, baudrate, node_id)
                self.__node_id = node_id

        def __del__(self):
                pass

        def get_command(self):
                return self.__er_serial.get_command()


        def send_response(self, dest_id, register, data):
                self.__er_serial.send_response(dest_id, register, data)