# configuration (W) 0x00 - 0x39
# actions (W) 0x40 - 0x79
ER_REG_SET_POSITION       = 0x40
ER_REG_SET_SPEED          = 0x41
ER_REG_SET_ACCEL          = 0x42
ER_REG_SET_DECEL          = 0x43

ER_REG_SET_SPEED_LIMIT    = 0x50
ER_REG_SET_CURRENT_LIMIT  = 0x51

ER_REG_INIT_MODULE      = 0x79

# results (R) 0x80 - 0xFF
ER_REG_GET_POSITION      =   0x80
ER_REG_GET_SPEED         =   0x81
ER_REG_GET_TORQUE        =   0x82
ER_REG_GET_ACCEL         =   0x83
ER_REG_GET_CURRENT       =   0x84


ER_REG_PING             = 0xFF