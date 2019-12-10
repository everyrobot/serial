#!/usr/bin/python
import time
from erClient import erClient, NoResponse


if __name__== "__main__":
        port = '/dev/ttyUSB1'
        baudrate = 115200
        node_id = 0x03

        client = erClient(port, baudrate, node_id)

        while True:
                try:
                        command = client.get_command()
                        print command
                        client.send_response(command['source_id'], command['register'], [0x03, 0x07, 0x07])

                        time.sleep(0.001)
                except NoResponse as e:
                        # print e.value
                        pass
