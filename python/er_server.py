#!/usr/bin/python
import time
import numpy as np
from erServer import erServer, NoResponse

if __name__== "__main__":
        port = '/dev/ttyUSB3'
        baudrate = 115200
        node_id = 0x01
        client_id = 100

        srv = erServer(port, baudrate, node_id)

        # try:


                # srv.cmd_ping(uc_id)
                # srv.cmd_calibrate_motor(100, motor_poles)

                # motor_poles = 7
                # srv.cmd_calibrate_motor(100, motor_poles)

                # mag_current = srv.get_float_from_body(0)
                # rr   = srv.get_float_from_body(4)
                # rs   = srv.get_float_from_body(8)
                # ls_d = srv.get_float_from_body(12)
                # ls_q = srv.get_float_from_body(16)
                # flux = srv.get_float_from_body(20)

                # iqp  = srv.get_iq_from_body(24)
                # iqi  = srv.get_iq_from_body(28)
                # idp  = srv.get_iq_from_body(32)
                # idi  = srv.get_iq_from_body(36)
                # spdp = srv.get_iq_from_body(40)
                # spdi = srv.get_iq_from_body(44)
                # print ("mag_current = {}".format(mag_current))
                # print ("rr = {}".format(mag_current))
                # print ("rs = {}".format(rs))
                # print ("ls_d = {}".format(ls_d))
                # print ("ls_q = {}".format(ls_q))
                # print ("flux = {}".format(flux))
                # print ("iqp = {}".format(iqp))
                # print ("iqi = {}".format(iqi))
                # print ("idp = {}".format(idp))
                # print ("idi = {}".format(idi))
                # print ("spdp = {}".format(spdp))
                # print ("spdi = {}".format(spdi))


                # motor_poles     = 7
                # mag_current     = 0.0
                # rr              = 0.0
                # rs              = 0.585800170898
                # ls_d            = 0.000484445918119
                # ls_q            = 0.000484445918119
                # flux            = 0.061248447746
                # iqp             = 30478700
                # iqi             = 1352486
                # idp             = 30478700
                # idi             = 1352486
                # spdp            = 55924052
                # spdi            = 372827

                # srv.cmd_set_pole_pairs(uc_id, motor_poles)
                # srv.cmd_set_magnetizing_current(uc_id, mag_current)
                # srv.cmd_set_rotor_resistance(uc_id, rr)
                # srv.cmd_set_stator_resistance(uc_id, rs)
                # srv.cmd_set_ls_d(uc_id, ls_d)
                # srv.cmd_set_ls_q(uc_id, ls_q)
                # srv.cmd_set_flux(uc_id, flux)
                # srv.cmd_set_pid_id(uc_id, idp, idi, 0) # PID dla current
                # srv.cmd_set_pid_iq(uc_id, idp, idi, 0)
                # srv.cmd_set_pid_spd(uc_id, spdp, spdi, 0) # PID dla predkosci

                # srv.cmd_init_module(uc_id)
                # time.sleep(7)
                # srv.cmd_set_accel(uc_id, 500)


                # srv.cmd_set_speed(uc_id, -1000)
                # time.sleep(3)
                # srv.cmd_set_speed(uc_id, 100)
                # time.sleep(3)

                # srv.cmd_set_speed(uc_id, 1000)
                # time.sleep(3)

                # srv.cmd_set_speed(uc_id, 3000)
                # time.sleep(3)
                # srv.cmd_set_speed(uc_id, 1000)
                # time.sleep(3)

                # initiate module
                # srv.send_command(client_id, 0x79, [])
                # srv.get_response()

        # except NoResponse as e:
                # print e.value

        i = 1
        # srv.cmd_set_position(client_id, 100.01)
        while True:
                # set position
                # srv.send_command(uc_id, 0x40, [10])
                # srv.cmd_set_speed(client_id, 0.5)
                # srv.get_response()

                # time.sleep(0.01)

                response = srv.cmd_set_position(client_id,-100.0)                
                # srv.cmd_init_module(client_id)
                # response = srv.get_iq_from_body(0)
                # print(srv.convert_iq_to_float(response, 15))
                # print(srv.convert_float_to_iq(15.012, 24))
                # print (srv.convert_iq_to_float(251859552,24))
                # srv.get_response()

                # time.sleep(0.001)

                # srv.cmd_set_position(client_id, 0.04)
                # # srv.get_response()
                #
                # time.sleep(0.01)
                #
                # srv.cmd_set_position(client_id, 0)
                # # srv.get_response()
                #
                # time.sleep(0.01)

                pass
                # time.sleep(1)

                # try:
                #         srv.cmd_set_accel(100, 100)
                # except NoResponse as e:
                #         print e.value

                # try:
                #         speed = np.int16(-1234)
                #         srv.cmd_set_speed(100, speed)
                # except NoResponse as e:
                #         print e.value
                #
                # time.sleep(1)
                


                # try:
                #         srv.cmd_ping(100)
                # except NoResponse as e:
                #         print e.value

                        # srv.send_command(0x02, 0xFF, [])
                        # time.sleep(0.1)
                        # res = srv.get_response()
                        # time.sleep(0.1)
                        # srv.send_command(0x03, 0x40, [0x03, 0x07, 0x03, 0x07, 0x03, 0x07, 0x03, 0x07, 0x03, 0x07])
                        # time.sleep(0.1)
                        # res = srv.get_response()
                        # print res
                        # srv.send_command(0x02, 0x02, [0x03])
                        # time.sleep(0.1)
                        # res = srv.get_response()
                        # print res
                # except NoResponse as e:
                #         print e.value
