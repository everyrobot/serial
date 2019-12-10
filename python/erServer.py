#!/usr/bin/python
# import serial
# import time
import struct
import numpy as np
# import datetime
# import threading
# from collections import deque
from erSerial import *
import erRegisters


class erServer():
        def __init__(self, port, baudrate, node_id):
                self.__er_serial = erSerial(port, baudrate, node_id)
                self.__node_id = node_id
                self.response = {}

        def __del__(self):
                pass

        def __convert_data_to_send(self, val):
                type_val = type(val)
                data = []
                if type_val == type(np.float32()):
                        ba = bytearray(struct.pack(">f", val))

                elif type_val == type(np.uint16()):
                        ba = bytearray(struct.pack(">H", val))

                elif type_val == type(np.int16()):
                        ba = bytearray(struct.pack(">h", val))

                elif type_val == type(np.int32()):
                        ba = bytearray(struct.pack(">l", val))

                elif type_val == type(np.uint32()):
                        ba = bytearray(struct.pack(">L", val))

                else:
                        print("Unsupported data type")
                        return []
                for i in range(len(ba)):
                        data.append(ba[i])

                return data

        def convert_data_to_send(self, val):
                return self.__convert_data_to_send(val)

        def get_float_from_body(self, start_byte):
                ba = self.response["body"]
                # print(["0x%02x" % b for b in ba])
                val = struct.unpack(">f", ba[start_byte:start_byte + 4])
                val = np.float32(val)
                return val[0]

        def get_uint_from_body(self, start_byte):
                ba = self.response["body"]
                # print(["0x%02x" % b for b in ba])
                val = struct.unpack(">H", ba[start_byte:start_byte + 2])
                val = np.uint16(val)
                return val[0]

        def get_int_from_body(self, start_byte):
                ba = self.response["body"]
                # print(["0x%02x" % b for b in ba])
                val = struct.unpack(">h", ba[start_byte:start_byte + 2])
                val = np.int16(val)
                return val[0]

        def get_iq_from_body(self, start_byte):
                ba = self.response["body"]
                # print(["0x%02x" % b for b in ba])
                val = struct.unpack(">l", ba[start_byte:start_byte + 4])
                val = np.int32(val)
                return val[0]

        def send_command(self, dest_id, register, data):
                self.__er_serial.send_command(dest_id, register, data)

        def get_response(self):
                self.response =  self.__er_serial.get_response()
                return self.response

        def command(self, dest_id, register, data, time):
                start = time.clock()
                self.send_command(dest_id, register, data)
                response = self.get_response()
                elapsed = (time.clock() - start)

        def convert_float_to_iq(self, float_data, n):
                tmp_data = float_data * (2 ** n)
                iq_data = int(tmp_data)
                return iq_data

        def convert_iq_to_float(self, iq_data, n):
                float_data = float(iq_data)
                float_data = float_data * (2 ** (-n))
                return float_data

        def cmd_get_position(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_GET_POSITION, data)
                return self.get_response()

        def cmd_get_speed(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_GET_SPEED, data)
                self.get_response()

        def cmd_get_torque(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_GET_TORQUE, data)
                return self.get_response()

        def cmd_get_accel(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_GET_ACCEL, data)
                return self.get_response()

        def cmd_get_current(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_GET_CURRENT, data)
                return self.get_response()

        def cmd_set_position(self, dest_id, pos):
                iq_data = self.convert_float_to_iq(pos, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_POSITION, data)
                return self.get_response()

        def cmd_set_speed(self, dest_id, speed):
                iq_data = self.convert_float_to_iq(speed, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_SPEED, data)
                return self.get_response()

        def cmd_set_accel(self, dest_id, accel):
                iq_data = self.convert_float_to_iq(accel, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_ACCEL, data)
                return self.get_response()

        def cmd_set_decel(self, dest_id, decel):
                iq_data = self.convert_float_to_iq(decel, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_DECEL, data)
                return self.get_response()

        def cmd_set_speed_limit(self, dest_id, speed_limit):
                iq_data = self.convert_float_to_iq(speed_limit, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_SPEED_LIMIT, data)
                return self.get_response()

        def cmd_set_current_limit(self, dest_id, curr_limit):
                iq_data = self.convert_float_to_iq(curr_limit, 24)
                data = self.__convert_data_to_send(np.int32(iq_data))

                self.send_command(dest_id, erRegisters.ER_REG_SET_CURRENT_LIMIT, data)
                return self.get_response()

        def cmd_init_module(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_INIT_MODULE, data)
                return self.get_response()

        def cmd_ping(self, dest_id):
                data = []
                self.send_command(dest_id, erRegisters.ER_REG_PING, data)
                return self.get_response()


