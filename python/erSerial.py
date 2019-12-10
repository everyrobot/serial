#!/usr/bin/python
import serial
import time
import numpy as np
import datetime
import threading
from collections import deque


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

class BadDestinationID(Exception):
        def __init__(self, value):
                self.value = value
        def __str__(self):
                return repr(self.value)

class BadCRC(Exception):
        def __init__(self, value):
                self.value = value
        def __str__(self):
                return repr(self.value)

class NoResponse(Exception):
        def __init__(self, value):
                self.value = value
        def __str__(self):
                return repr(self.value)

class NoCommand(Exception):
        def __init__(self, value):
                self.value = value
        def __str__(self):
                return repr(self.value)

# class BadCRC(Exception):
#         def __init__(self, value):
#                 self.value = value
#         def __str__(self):
#                 return repr(self.value)


class erSerial():
        def __init__(self, port, baudrate, node_id):
                self.__serial = serial.Serial(port=port, baudrate=baudrate)
                self.__node_id = node_id
                self.__in_buffer = []

                self.__commands  = deque()
                self.__respondes = deque()

                self.__rx_thread = threading.Thread(target=self.__rx_loop, args=())
                self.__rx_thread.start()
                self.__tx_thread = threading.Thread(target=self.__tx_loop, args=())
                self.__tx_thread.start()


        def __del__(self):
                self.__serial.close()

        def __calculate_crc(self, data):
                crc = np.uint16(0)
                for x in range(len(data)):
                        crc += data[x]
                crc ^= 0xFF
                crc &= 0xFF

                return crc

        def __pack_header(self, id_d, register, flags, datasize):
                header = [self.__node_id, id_d, register, flags, datasize]
                return [0x45, 0x52] + header + [self.__calculate_crc(header)]

        def __unpack_header(self, header):
                h0    = header[0]
                if h0 != 0x45:
                        raise BadDestinationID('First byte not E!')
                h1    = header[1]
                if h1 != 0x52:
                        raise BadDestinationID('Second byte not R!')
                id_s  = header[2]
                id_d  = header[3]
                if self.__node_id != id_d:
                        raise BadDestinationID('Bad ID!')
                reg   = header[4]
                flags = header[5] # TODO: obsluga flag
                ds    = header[6] # body size
                crc   = header[7]
                if crc != self.__calculate_crc([id_s, id_d, reg, flags, ds]):
                        raise BadCRC('Bad header CRC!')
                return id_s, reg, flags, ds

        def __unpack_body(self, body):
                _body = bytearray(body[0:-1])
                crc   = body[-1]
                if crc != self.__calculate_crc(_body):
                        raise BadCRC('Bad body CRC!')
                return _body

        def __pack_datagram(self, id_d, register, flags, data):
                header = self.__pack_header(id_d, register, flags, len(data))
                if len(data) > 0:
                        data_crc = self.__calculate_crc(data)
                        return  header + data + [data_crc]
                else:
                        return  header
        
        def __rx_loop(self):
                datagram_ok = False
                header_ok = False
                body_ok = False
                id_s, reg, flags, ds = [0, 0, 0, 0]

                while (True):
                        if (self.__serial.in_waiting > 0):
                                self.__in_buffer += self.__serial.read(self.__serial.inWaiting())

                        hex_string = "".join("0x%02x " % b for b in bytearray(self.__in_buffer))
                        if len(self.__in_buffer) > 0:
                                print( str(datetime.datetime.now()) + " IN:  (" + str(len(self.__in_buffer)) + ") " + hex_string)

                        # analyse buffer - byte after byte
                        # self.__in_buffer += [0x00]
                        # self.__in_buffer += ['R']
                        # self.__in_buffer += ['E']
                        # print len(self.__in_buffer)
                        # if header_ok == False:
                                # test first

                        if len(self.__in_buffer) > 0:
                                # datagram must starts with E
                                if self.__in_buffer[0] != 'E': # 0x45:
                                        # remove first byte
                                        self.__in_buffer.pop(0)
                                        continue

                                if len(self.__in_buffer) > 1 and self.__in_buffer[1] != 'R':
                                        self.__in_buffer.pop(0)
                                        continue

                                if len(self.__in_buffer) > 7 and header_ok == False:
                                        # header test
                                        header = bytearray(self.__in_buffer[0:8])
                                        # crc = self.__calculate_crc(header[2:7])
                                        # hex_string = "".join("0x%02x " % b for b in bytearray(header + [crc]))
                                        # print("H: (" + str(len(self.__in_buffer)) + ") " + hex_string)

                                        try:
                                                id_s, reg, flags, ds = self.__unpack_header(header)
                                                # print "header_ok"
                                                header_ok = True
                                        except (BadDestinationID) as e:
                                                print e.value
                                                # print self.__node_id
                                                # print header[3]
                                                header_ok = False
                                                id_s, reg, flags, ds = [0, 0, 0, 0]
                                                self.__in_buffer.pop(0)
                                                continue

                                if len(self.__in_buffer) > 7 and header_ok:
                                        if ds == 0: # empty datargam - move 8 bytes to ?? FIXME
                                                del self.__in_buffer[:8 + ds + 1]
                                                # self.__respondes.append( {'source_id' : id_s, 'register': reg, 'flags': flags, 'body': body } )
                                                self.__respondes.append( {'source_id' : id_s, 'register': reg, 'flags': flags} )
                                                header_ok = False
                                                body_ok = False
                                                continue
                                        else:
                                                if len(self.__in_buffer) > 8 + ds:
                                                        body = self.__unpack_body(bytearray(self.__in_buffer[8:8 + ds + 1]))  # bytearray
                                                        body_ok = True
                                                pass
                                
                                # remove data from
                                if header_ok and body_ok:
                                        del self.__in_buffer[:8 + ds + 1]
                                        self.__respondes.append( {'source_id' : id_s, 'register': reg, 'flags': flags, 'body': body } )

                                        header_ok = False
                                        body_ok = False

                        # print "INPUT COMMANDS " + str(len(self.__respondes))
                        # print(type(self.__in_buffer)) 
                        time.sleep(0.001)

        def __tx_loop(self):
                while (True):
                        # print len(self.__commands)
                        if len(self.__commands) > 0:
                                command = self.__commands.popleft()
                                hex_string = "".join("0x%02x " % b for b in bytearray(command))
                                print( str(datetime.datetime.now()) + " OUT: (" + str(len(command)) + ") " + hex_string)
                                self.__serial.write(command)
                        time.sleep(0.001)

        def send_command(self, dest_id, register, data):
                if len(self.__commands) > 0 or len(self.__respondes) > 0:
                        raise Exception("Kolejka nie jest pusta!")
                datagram = self.__pack_datagram(dest_id, register, 0x00, data)
                self.__commands.append(datagram)
                # waiting for response
                # time.sleep(0.01)
                # self.
                # if len(self.__respondes) == 0:
                #         pass
                #         # raise Exception("Brak odpowiedzi ze strony klienta!")
                # else:
                #         response = self.__respondes.popleft()
                #         # src_id check
                #         # register check
                # return True, data

        def get_command(self):
                if len(self.__respondes) == 0:
                        raise NoResponse("There is no commands to execute!.")
                else:
                        return self.__respondes.popleft()

        def send_response(self, dest_id, register, data):
                if len(self.__commands) > 0 or len(self.__respondes) > 0:
                        raise Exception("Kolejka nie jest pusta!")
                datagram = self.__pack_datagram(dest_id, register, 0x01, data)
                self.__commands.append(datagram)

        def get_response(self):
                while(len(self.__respondes) == 0):
                        pass
                return self.__respondes.popleft()
                # if len(self.__respondes) == 0:
                        # raise NoResponse("There is no response.")
                # else:
                        # return self.__respondes.popleft()

# def print_datagram(datagram):
#         hex_string = "".join("0x%02x " % b for b in bytearray(datagram))
#         print("DTG: " + hex_string)

# def float_to_bytelist(val):
#         return list(bytearray(struct.pack(">f", val)))

# def send_datagram(datagram):
#         start  = datetime.datetime.now()
#         ser.write(datagram)
#         finish = datetime.datetime.now()
#         delta = finish - start
#         # return int(delta.total_seconds() * 1000)
#         return delta.microseconds
#         # return delta
