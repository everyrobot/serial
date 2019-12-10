(function() {
// We have cases where not every js code exteneded from OCFBase want the behavior shared among I2C, GPIO, SPI, Uart.
// Leave Base as is so that System is not affected. Extend Base to BaseEx for the above sharing purpsoe.
    function OCFBaseEx(packetParser, sendPacket) {
        OCFBase.apply(this, arguments);
        this.unit_state = {}; // {unit: {current: undefined|disabled 0|enabled 1|configured 2, qdef:{}, } }
    };
    OCFBaseEx.prototype = Object.create(OCFBase.prototype);
    OCFBaseEx.prototype.constructor = OCFBaseEx;
    window.OCFBaseEx = OCFBaseEx;
    
    OCFBaseEx.prototype.initSymbolsForDevice = function(settings, registerModel) {
        OCFBase.prototype.initSymbolsForDevice.apply(this, arguments);
        // add additional common logic here, if needed.
        OCFBaseEx.prototype.readSettingPropMap('unit', 'unitMap', settings, registerModel);
    };

    OCFBaseEx.prototype.init = function(info) {
        // must return a promise
        this.info = info;
        this.unit = this.parse_int(info.unit)
        this.config_seq = info.config; // is a configuration sequence
        if (info.crc) {
            this.crc = OCFBaseEx.prototype._get_crc.call(this,info.crc);
            this.crc_user = OCFBaseEx.prototype._crc_user[info.name];
        }
        return this.ensureConfigured(this.config_seq);
    };

    OCFBaseEx.prototype.readSettingPropMap = function(propName, propMapName, settings, registerModel) {
        var propMap = settings[propMapName];
        if (propMap !== undefined) {
            for (var name in propMap) {
                if (propMap.hasOwnProperty(name)) {
                    registerModel.addUserPreference('$'+propName+'.'+name, propMap[name]);
                }
            }
        }
        if (settings[propName] !== undefined) {
            registerModel.addUserPreference('$'+propName, settings[propName]);
        }
    }

    OCFBaseEx.prototype.getPropForRegister = function(propName, info, registerModel) {
        var bindingName = '_'+propName+'Binding';
        info[bindingName] = info[bindingName] || {};

        info[bindingName][registerModel.id] = info[bindingName][registerModel.id] ||
            (info[propName] !== undefined && registerModel.parseModelSpecificBindExpressionWithPrefix('$'+propName+'.', info[propName])) ||
            (info.parentGroup && info.parentGroup[propName] !== undefined && registerModel.parseModelSpecificBindExpressionWithPrefix('$'+propName+'.', info.parentGroup[propName])) ||
            registerModel.getBinding('$'+propName);

        return info[bindingName][registerModel.id].getValue();
    }

    OCFBaseEx.prototype.reset = function() {
        var myname = this.constructor.name;
        for (unit in this.unit_state) {
            var qdef = this.unit_state[unit].qdef;
            for (k in qdef||{}) {
                qdef[k].forEach(function(v,idx,ary) { v.reject(myname + ' reset.'); });
            }
        }
        this.unit_state = {};
    };

    OCFBaseEx.prototype.handlePacket = function(pkt) {
        // controller to host
        //gc.console.debug(this.constructor.name, this.log_packet, 'handlePacket', pkt);
        var packet_type = this.leb2_uint16(pkt.type);
        var unit = pkt.if_type_unit[0];
        var command = this.leb2_uint16(pkt.command);
        var status = this.leb4_uint32(pkt.status);
        var qdefkey = this.get_qdef_key(pkt);
        var qdef = this.unit_state[unit] && this.unit_state[unit].qdef && this.unit_state[unit].qdef[qdefkey] &&
            this.unit_state[unit].qdef[qdefkey].shift();
        var handler = this.get_packet_handler(packet_type, command, pkt);
        // For 'response' of commands, there should be qdef. For event or interrupt, there is no qdef.
        if (handler) {
            handler(this, qdef, unit, status, pkt, packet_type);
        } else if (qdef) {
            qdef.resolve({self: this, unit, status, packet: pkt});
        }
        return true; // for saying I handeled it while let resolve/reject or binding.update/setValue to pass out data/errors
    };

    OCFBaseEx.prototype.get_packet_handler = function(packet_type, command) {
        // If there is no handler for the command, this method must return undefined.
        // If there is a handler for the command, this method must return a function of this form:
        //    function(this, qdef, unit, status, pkt) => undefined;
        //    and that function must use qdef to either resolve or reject, if qdef is provided
        // e.g., you have a function reply_cmd_hdl() defined somewhere like below,
        //    var reply_cmd_hdl = function(this, qdef, unit, status, pkt, packet_type) {
        //      if (status==0) { qdef.resolve(some_result_from_pkt); }
        //      else { qdef.reject('some failure message'); }
        //    }
        // This method default behavior looks up the handler by command (which is an integral type),
        // you can pre-store those command specific handlers into this.handlers in your
        // construnctor function or initSymbolsForDevice, e.g.,
        //    this.handlers = [reply_cmd_hdl, reply_another_cmd_hdlr];
        // Alternatively, you can override the default logic for more advanced usage, i.e.,
        // your overridden get_packet_handler will return reply_cmd_dlr;
        return this.handlers ? this.handlers[command] : undefined; 
    };

    OCFBaseEx.prototype.get_qdef_key = function(pkt) {
        return this.flatten([pkt.if_type_unit, pkt.command, pkt.params]);
    };

    // Reference ocf_common.h
    //typedef struct _PACKED_STRUCT_ATTRIB
    //{
    //    uint16_t     signature;     // Packet signature (must be PACKET_SIGNATURE value)
    //    uint16_t     type;          // Type of packet
    //    uint32_t     transfer_len;  // Total number of bytes in the transfer
    //    uint16_t     packet_num;    // Sequential packet number
    //    OC_STATUS    status;        // Status code returned by command function
    //    uint16_t     payload_len;   // Number of bytes in the payload
    //    uint16_t     if_type_unit;  // Interface type and unit
    //    uint16_t     command;       // Command code
    //    uint16_t     reserved;      // unused (for 32-bit alignment)
    //    uint32_t     param[8];      // eight 32-bit parameters
    //} OCF_PACKET_HEADER;
    var COMMAND_PACKET = 0x0001;
    var PARAM_MAX = 8;

    OCFBaseEx.prototype.h2c_command = function(if_type, unit, command, params, payload) {
        // host to controller
        //Array.prototype.push.apply(params, payload);
        //var params_and_payload = payload_len && params.concat(payload) || params;
        // Someone will insert status and reserved
        var pkt = {
            type: this.uint16_leb2(COMMAND_PACKET),
            transfer_len: this.uint32_leb4(0),
            packet_num: this.uint16_leb2(1),
            payload_len: this.uint16_leb2(payload && payload.length || 0),
            if_type_unit: [unit & 0xff, if_type], // this is 16 bits le // Ref: if_type_unit: I2C_Interface << 8 | unit
            command: this.uint16_leb2(command),
            params: this.fill_param(params, PARAM_MAX-params.length),
            payload: payload
        };
        if (!this.unit_state[unit]) this.unit_state[unit] = {};
        if (!this.unit_state[unit].qdef) this.unit_state[unit].qdef = {};
        var qdef = Q.defer();
        var qdefkey = this.get_qdef_key(pkt);
        if (!this.unit_state[unit].qdef[qdefkey]) this.unit_state[unit].qdef[qdefkey] = [];
        this.unit_state[unit].qdef[qdefkey].push(qdef);
        //gc.console.debug(this.constructor.name, this.log_packet, 'sendPacket', pkt);
        this.sendPacket(pkt);
        return qdef.promise;
    };

    OCFBaseEx.prototype.findConfigured = function() {
        var ans = [];
        for (unit in this.unit_state) {
            if (this.unit_state[unit].current == 2) { ans.push(unit) };
        }
        return ans;
    };

    var flatten = function(ary) {
        return ary.reduce(function(a,b) {return a.concat(Array.isArray(b) ? flatten(b) : b);}, []);
    };
    OCFBaseEx.prototype.flatten = flatten;

    OCFBaseEx.prototype.fill_param = function(ary, n) {
        var ans = this.flatten(ary);
        for (var idx=0; idx<n*4; idx++)  ans.push(0);
        return ans;
    };

    var bytes_hex = OCFBaseEx.prototype.bytes_hex = function(bytes) {
        return Array.from(bytes, function(byte) {
            return ('0' + (byte & 0xFF).toString(16)).slice(-2);
        }).join(' ');
    };
    var bytes_ascii = OCFBaseEx.prototype.bytes_ascii = function(bytes) { return String.fromCharCode.apply(null, bytes) };

    OCFBaseEx.prototype.log_packet = function(label, pkt) {
        return label +'\n' + JSON.stringify(pkt) +'\n' + bytes_hex(pkt.payload||[]) + '\n' + bytes_ascii(pkt.payload||[]);
    };

    OCFBaseEx.prototype.status_msg = function(status, mesg) {
        return mesg + (status !== undefined ? '. Status = ' + status : '');
    };

    OCFBaseEx.prototype.bool_to_int = function(val) {
        if (typeof val === 'string') {
            return val == 'true' ? 1 : 0;
        } else if (typeof val === 'boolean') {
            return val == true ? 1 : 0;
        }
        return val;
    };
    OCFBaseEx.prototype.parse_int = function(val) {
        if (typeof val === 'string') {
            return parseInt(val);
        }
        return val;
    };

// [ CRC - var crc = new CRC(...); crc.checksum(byte[])
    function CRC(params) {
      this.table = CRC.table(params.polynomial);
      this.width_byte = (params.width+7)>>3;
    }
    CRC.prototype.checksum = function(byte_array, previous) {
      var c = previous || 0;
      for (var i = 0; i < byte_array.length; i++ ) {
        c = this.table[(c ^ byte_array[i]) % 256];
      }
      return c;
    }
    CRC.table = function(polynomial) {
      var csTable = [];
      for ( var i = 0; i < 256; ++i ) {
        var curr = i;
        for ( var j = 0; j < 8; ++j ) {
          if ((curr & 0x80) !== 0) {
            curr = ((curr << 1) ^ polynomial) % 256;
          } else {
            curr = (curr << 1) % 256;
          }
        }
        csTable[i] = curr;
      }
      return csTable;
    }
    CRC.POLY = {
      CRC8 : 0xd5,
      CRC8_CCITT : 0x07,
      CRC8_DALLAS_MAXIM : 0x31,
      CRC8_SAE_J1850 : 0x1D,
      CRC_8_WCDMA : 0x9b,
    }
// CRC ]
    gc = gc || {};
    gc.databind = gc.databind || {};
    gc.databind.CRC = OCFBaseEx.prototype.CRC = CRC;
    OCFBaseEx.prototype._shared_crc = {};
    OCFBaseEx.prototype._get_crc = function(params) {
        var x = OCFBaseEx.prototype._shared_crc[params.polynomial];
        if (x === undefined) {
            x = OCFBaseEx.prototype._shared_crc[params.polynomial] = new OCFBaseEx.prototype.CRC(params)
        }
        return x;
    }
    OCFBaseEx.prototype._crc_user = {};
    OCFBaseEx.prototype.register_crc_user = function(impl, interface_name, device) {
        // impl (required) - provides customized logic to embed crc bytes in data, and verify crc bytes and extract data
        // impl is an object  {
        //  embed_crc_data: function(crc, arg) {
        //      return {payload: crc_embedded_payload, num_read_bytes: num_byes_to_read_include_crc}
        //  }
        //  , verify_crc_data: function(crc, arg) {
        //      return {valid: true, payload: payload_without_crc};
        //  }
        // }
        // interface_name (required) - specific interfaces that needs the given crc implementation, e.g. 'i2c'
        // devices (optional) - specific devices that needs the given crc implementation, undefined means all devices
        OCFBaseEx.prototype._crc_user[interface_name] = impl;
    };

    OCFBaseEx.prototype._interface_defs = new Map();
    OCFBaseEx.prototype.add_interface = function(name, impl) {
        OCFBaseEx.prototype._interface_defs.set(name, impl);
    };
    OCFBaseEx.prototype.get_interface = function(name) {
        return OCFBaseEx.prototype._interface_defs.get(name) || OCFCustom;
    };

})();

