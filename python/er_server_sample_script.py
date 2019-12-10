#!/usr/bin/python
import time
import numpy as np
from erServer import erServer, NoResponse

if __name__== "__main__":
        port = '/dev/ttyUSB0'
        baudrate = 115200

        node_id = 0x01
        client_id = 0x64

        srv = erServer(port, baudrate, node_id)

        try:

                # initiate module
                srv.send_command(client_id, 0x79, [])
                srv.get_response()

        except NoResponse as e:
                print e.value

        while True:
                # set position
                srv.send_command(client_id, 0x40, [])
                srv.get_response()

                # get position
                # srv.send_command(client_id, 0x83, [])
                # srv.get_response()

                time.sleep(3)

