/*
 * gc global variable provides access to GUI Composer infrastructure components and project information.
 * For more information, please see the Working with Javascript guide in the online help.
 */
var gc = gc || {};
gc.services = gc.services || {};

/*
 *  Boilerplate code for creating computed data bindings
 */
document.addEventListener('gc-databind-ready', function() {
    /* 
     *   Add custom computed value databindings here, using the following method:
     *
     *   function gc.databind.registry.bind(targetBinding, modelBinding, [getter], [setter]);
     *
     *   param targetBinding - single binding string or expression, or array of binding strings for multi-way binding.
     *   param modelBinding - single binding string or expression, or array of binding strings for multi-way binding.
     *   param getter - (optional) - custom getter function for computing the targetBinding value(s) based on modelBinding value(s).
     *   param setter - (optional) - custom setter function for computing the modelBinding value(s) based on targetBinding value(s).
     */

    // For example, a simple computed values based on simple expression
    // gc.databind.registry.bind('widget.id.propertyName', "targetVariable == 1 ? 'binding is one' : 'binding is not one'");

    // Or a custom two-way binding with custome getter and setter functions.  (setter is optional)  (getter only indicates one-way binding)
    // gc.databind.registry.bind('widget.id.propertyName', "targetVariable", function(value) { return value*5/9 + 32; }, function(value) { (return value-32)*9/5; });

    // Event 1 to n bindings
    /*
    gc.databind.registry.bind('widget.date.value', 
        // dependant bindings needed in order to compute the date, in name/value pairs.
        {
            weekday: 'widget.dayOfWeek.selectedText',
            day: 'widget.dayOfMonth.value',
            month: 'widget.month.selectedText',
            year: 'widget.year.value'
        }, 
        // getter for date computation
        function(values) 
        {
            // compute and return the string value to bind to the widget with id 'date'
            return values.weekday + ', ' + values.month + ' ' + values.day + ', ' + values.year;
        }
    ); 
    */
});

/*
 *  Boilerplate code for creating custom actions
 */
document.addEventListener('gc-nav-ready', function() {
    /* 
     *   Add custom actions for menu items using the following api:
     *
     *   function gc.nav.registryAction(id, runable, [isAvailable], [isVisible]);
     *
     *   param id - uniquely identifies the action, and should correspond to the action property of the menuaction widget.
     *   param runable - function that performs the custom action.
     *   param isAvailable - (optional) - function called when the menu action is about to appear.  Return false to disable the action, or true to enable it.
     *   param isVisible - (optional) - function called when the menu action is about to appear.  Return false to hide the action, or true to make it visible.
     */

    // For example,
    // gc.nav.registerAction('myCustomCloseAction', function() { window.close(); }, function() { return true; }, function() { return true; });

    // Alternatively, to programmatically disable a menu action at any time use:
    // gc.nav.disableAction('myCustomCloseAction);    then enable it again using:  gc.nav.enableAction('myCustomCloseAction');
});

/*
 *  Boilerplate code for working with components in the application gist
 */

const LIBUSB_TRANSFER_TYPE_CONTROL = 0;
const LIBUSB_TRANSFER_TYPE_ISOCHRONOUS = 1;
const LIBUSB_TRANSFER_TYPE_BULK = 2;
const LIBUSB_TRANSFER_TYPE_INTERRUPT = 3;
const LIBUSB_ISO_SYNC_TYPE_NONE = 0;
const LIBUSB_ISO_SYNC_TYPE_ASYNC = 1;
const LIBUSB_ISO_SYNC_TYPE_ADAPTIVE = 2;
const LIBUSB_ISO_SYNC_TYPE_SYNC = 3;
const LIBUSB_ISO_USAGE_TYPE_DATA = 0;
const LIBUSB_ISO_USAGE_TYPE_FEEDBACK = 1;
const LIBUSB_ISO_USAGE_TYPE_IMPLICIT = 2;

const LIBUSB_CLASS_PER_INTERFACE = 0;
const LIBUSB_CLASS_AUDIO = 1;
const LIBUSB_CLASS_COMM = 2;
const LIBUSB_CLASS_HID = 3;
const LIBUSB_CLASS_PTP = 6;
const LIBUSB_CLASS_PRINTER = 7;
const LIBUSB_CLASS_MASS_STORAGE = 8;
const LIBUSB_CLASS_HUB = 9;
const LIBUSB_CLASS_DATA = 10; // CDC Data
const LIBUSB_CLASS_SMART_CARD = 11;
const LIBUSB_CLASS_CONTENT_SECURITY = 13;
const LIBUSB_CLASS_VIDEO = 14;
const LIBUSB_CLASS_PERSONAL_HEALTHCARE = 15;
const LIBUSB_CLASS_AUDIOVIDEO = 16;
const LIBUSB_CLASS_BILLBOARD = 17;
const LIBUSB_CLASS_TYPEC_BRIDGE = 18;
const LIBUSB_CLASS_MISCELLANEOUS = 239;
const LIBUSB_CLASS_APP_SPEC = 254;
const LIBUSB_CLASS_VENDOR_SPEC = 255;
const LIBUSB_ENDPOINT_IN = 0x80;
const LIBUSB_ENDPOINT_OUT = 0x00;

const USB_SS_BCD = 0x300;
const LIBUSB_DT_HUB = 0x29;
const LIBUSB_DT_SUPERSPEED_HUB = 0x2a;

/* Descriptor sizes per descriptor type */
const LIBUSB_DT_DEVICE_SIZE = 18
const LIBUSB_DT_CONFIG_SIZE = 9
const LIBUSB_DT_INTERFACE_SIZE = 9
const LIBUSB_DT_ENDPOINT_SIZE = 7
const LIBUSB_DT_ENDPOINT_AUDIO_SIZE = 9;
const LIBUSB_DT_HUB_NONVAR_SIZE = 7;

const LIBUSB_REQUEST_TYPE_STANDARD = (0x00 << 5);
const LIBUSB_REQUEST_TYPE_CLASS = (0x01 << 5);
const LIBUSB_REQUEST_TYPE_VENDOR = (0x02 << 5);
const LIBUSB_REQUEST_TYPE_RESERVED = (0x03 << 5);

const LIBUSB_RECIPIENT_DEVICE = 0x00;
const LIBUSB_RECIPIENT_INTERFACE = 0x01;
const LIBUSB_RECIPIENT_ENDPOINT = 0x02;
const LIBUSB_RECIPIENT_OTHER = 0x03;

const LIBUSB_REQUEST_GET_STATUS = 0x00;
const LIBUSB_REQUEST_CLEAR_FEATURE = 0x01;
const LIBUSB_REQUEST_SET_FEATURE = 0x03;
const LIBUSB_REQUEST_SET_ADDRESS = 0x05;
const LIBUSB_REQUEST_GET_DESCRIPTOR = 0x06;
const LIBUSB_REQUEST_SET_DESCRIPTOR = 0x07;
const LIBUSB_REQUEST_GET_CONFIGURATION = 0x08;
const LIBUSB_REQUEST_SET_CONFIGURATION = 0x09;
const LIBUSB_REQUEST_GET_INTERFACE = 0x0A;
const LIBUSB_REQUEST_SET_INTERFACE = 0x0B;
const LIBUSB_REQUEST_SYNCH_FRAME = 0x0C;
const LIBUSB_REQUEST_SET_SEL = 0x30;
const LIBUSB_SET_ISOCH_DELAY = 0x31;

var initComplete = false;
var templateObj;
var serviceUsb;
var backplane;
var msTimeout = 1000;
var openDeviceInfo = null;
var openDeviceDescriptors = null;

function showDeviceDescriptor(dD) {
    const result = [];
    switch (dD.bDeviceClass) {
        case LIBUSB_CLASS_PER_INTERFACE:
            result.push("\n\t\t\tDevice class = Use class info in the I/F descriptors.");
            break;
        case LIBUSB_CLASS_AUDIO:
            result.push("\n\t\t\tDevice class = AUDIO.");
            break;
        case LIBUSB_CLASS_COMM:
            result.push("\n\t\t\tDevice class = COMM.");
            break;
        case LIBUSB_CLASS_HID:
            result.push("\n\t\t\tDevice class = HID.");
            break;
        case LIBUSB_CLASS_PTP:
            result.push("\n\t\t\tDevice class = PTP");
            break;
        case LIBUSB_CLASS_PRINTER:
            result.push("\n\t\t\tDevice class = PRINTER");
            break;
        case LIBUSB_CLASS_MASS_STORAGE:
            result.push("\n\t\t\tDevice class = MASS STORAGE");
            break;
        case LIBUSB_CLASS_DATA:
            result.push("\n\t\t\tDevice class = COM PORT (CDC DATA)");
            break;
        case LIBUSB_CLASS_HUB:
            result.push("\n\t\t\tDevice class = HUB.");
            break;
        case LIBUSB_CLASS_MISCELLANEOUS:
            result.push("\n\t\t\tDevice class = Miscellaneous.");
            break;
        case LIBUSB_CLASS_APP_SPEC:
            result.push("\n\t\t\tDevice class = App specific.");
            break;
        case LIBUSB_CLASS_VENDOR_SPEC:
            result.push("\n\t\t\tDevice class = Vendor specific.");
            break;
        default:
            result.push("\n\t\t\tUnknown device class (" + dD.bDeviceClass + ")");
            break;
    }
    result.push("\n\tDEVICE DESCRIPTOR:");
    result.push("\t" + dD.bLength + "\tbLength \t\tDescriptor Size in Bytes");
    result.push("\t" + dD.bDescriptorType + "\tbDescriptorType\t\tDescriptor Type (Device)");
    result.push("\t0x" + Number(dD.bcdUSB).toString(16) + "\tbcdUSB\t\t\tUSB Specification release number (BCD)");
    result.push("\t0x" + Number(dD.bDeviceClass).toString(16) +
        "\tbDeviceClass\t\tClass Code (class defined in interface descriptor)");
    result.push("\t" + dD.bDeviceSubClass + "\tbDeviceSubClass\t\tSubclass code");
    result.push("\t" + dD.bDeviceProtocol + "\tbDeviceProtocol\t\tProtocol code");
    result.push("\t0x" + Number(dD.bMaxPacketSize0).toString(16) + "\tbMaxPacketSize0\t\tEndpoint 0 maximum packet size");
    result.push("\t0x" + Number(dD.idVendor).toString(16) +
        "\tidVendor (" + dD.idVendor + ")\t\tVendor ID (assigned by USB-IF)");
    result.push("\t0x" + Number(dD.idProduct).toString(16) + "\tidProduct\t\tProduct ID (assigned by Vendor)");
    result.push("\t0x" + Number(dD.bcdDevice).toString(16) + "\tbcdDevice\t\tDevice Release Number (BCD)");
    result.push("\t" + dD.iManufacturer + "\tiManufacturer\t\tManufacturer string index");
    result.push("\t" + dD.iProduct + "\tiProduct\t\tProduct string index");
    result.push("\t" + dD.iSerialNumber + "\tiSerialNumber\t\tDevice Serial Number string index");
    result.push("\t" + dD.bNumConfigurations + "\tbNumConfigurations\tNumber of configurations");
    result.push("\n");
    templateObj.$.ti_textarea.value += result.join("\n");
}

function showConfigDescriptor(cD) {
    const result = [];
    try {
        if (cD) {
            result.push("\n\tCONFIG DESCRIPTOR:");
            result.push("\t" + cD.bLength + "\tbLength \t\tDescriptor Size in Bytes");
            result.push("\t" + cD.bDescriptorType + "\tbDescriptorType\t\tDescriptor Type (Configuration)");
            result.push("\t" + cD.wTotalLength + "\twTotalLength\t\tTotal length of this and subordinate descriptors");
            result.push("\t" + cD.bConfigurationValue + "\tbConfigurationValue\tIndex of this configuration");
            result.push("\t" + cD.iConfiguration + "\tiConfiguration\t\tConfiguration string index");
            result.push("\t0x" + Number(cD.bmAttributes).toString(16) +
                "\tbmAttributes\t\tAttributes (self-powered,remote wakeup supported)");
            result.push("\t" + cD.bMaxPower + "\tbMaxPower\t\t" + 100 * cD.bMaxPower / 64 + " mA max bus current requested.");
            templateObj.$.ti_textarea.value += result.join("\n");
        }
    } catch (ex) {
        console.log(`getConfigDescription exception: ex=${ex}`);
    }
    return result;
}

function getInterfaceDescriptor(iDs, interfaceNumber) {
    let result = "";
    if (iDs) {
        const numInterfaces = iDs.length;
        if (numInterfaces > interfaceNumber) {
            result = iDs[interfaceNumber];
        }
    }
    return result;
}

function getTransferAttributes(epD) {
    const result = {
        transferType: LIBUSB_TRANSFER_TYPE_CONTROL,
        isoSyncType: LIBUSB_ISO_SYNC_TYPE_NONE,
    };

    /*
    Bits 0..1 Transfer Type
    00 = Control
    01 = Isochronous
    10 = Bulk
    11 = Interrupt

    Bits 2..7 are reserved. If Isochronous endpoint,
        Bits 3..2 = Synchronisation Type (Iso Mode)
        00 = No Synchonisation
        01 = Asynchronous
        10 = Adaptive
        11 = Synchronous

        Bits 5..4 = Usage Type (Iso Mode)
        00 = Data Endpoint
        01 = Feedback Endpoint
        10 = Explicit Feedback Data Endpoint
        11 = Reserved
    */

    switch (epD.bmAttributes & 0x3) {
        case 0:
            result.transferType = LIBUSB_TRANSFER_TYPE_CONTROL;
            break;
        case 1:
            result.transferType = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
            // tslint:disable-next-line
            switch ((epD.bmAttributes >> 6) & 0x03) {
                case 0:
                    result.isoSyncType = LIBUSB_ISO_SYNC_TYPE_NONE;
                    break;
                case 1:
                    result.isoSyncType = LIBUSB_ISO_SYNC_TYPE_ASYNC;
                    break;
                case 2:
                    result.isoSyncType = LIBUSB_ISO_SYNC_TYPE_ADAPTIVE;
                    break;
                case 3:
                    result.isoSyncType = LIBUSB_ISO_SYNC_TYPE_SYNC;
                    break;
            }
            // tslint:disable-next-line
            switch ((epD.bmAttributes >> 4) & 0x03) {
                case 0:
                    result.isoUsageType = LIBUSB_ISO_USAGE_TYPE_DATA;
                    break;
                case 1:
                    result.isoUsageType = LIBUSB_ISO_USAGE_TYPE_FEEDBACK;
                    break;
                case 2:
                    result.isoUsageType = LIBUSB_ISO_USAGE_TYPE_IMPLICIT;
                    break;
            }
            break;
        case 2:
            result.transferType = LIBUSB_TRANSFER_TYPE_BULK;
            break;
        case 3:
            result.transferType = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
            break;
    }
    return result;
}

function _getEndpoints(device, interfaceIndex) {
    let endpoints = new Array();
    if (device.interfaces && device.interfaces.length > interfaceIndex) {
        const devIf = device.interface(interfaceIndex);
        endpoints = devIf.endpoints;
    }
    return endpoints;
}

function _getErrorMessage(error) {
    var result = "";
    if (error) {
        if (error.message) {
            result = error.message;
            if (error.stack) {
                result += "\n" + error.stack
            }
        } else {
            result = error;
        }
    }
    return result;
}

function showInterfaceDescriptors(iDs) {
    var result = [];
    if (iDs) {
        const numInterfaces = iDs.length;
        result.push("\n\tdevice.interfaces.length = " + numInterfaces + "\n");
        for (let i = 0; i < numInterfaces; i++) {
            result.push("\n\tINTERFACE " + i + " DESCRIPTOR");
            const iD = JSON.parse(getInterfaceDescriptor(iDs, i));
            result.push("\t" + iD.bLength + "\tbLength\t\t\tDescriptor Size in Bytes");
            result.push("\t" + iD.bDescriptorType + "\tbDescriptorType\t\tDescriptor Type (Interface)");
            result.push("\t" + iD.bInterfaceNumber + "\tbInterfaceNumber\tInterface number");
            result.push("\t" + iD.bAlternateSetting + "\tbAlternateSetting\tAlternate setting number");
            result.push("\t" + iD.bNumEndpoints + "\tbNumEndpoints\t\tNumber of endpoints in this interface");
            result.push("\t0x" + Number(iD.bInterfaceClass).toString(16) +
                "\tbInterfaceClass\t\tInterface class (vendor specific)");
            if (iD.bInterfaceSubClass !== undefined) {
                result.push("\t0x" + Number(iD.bInterfaceSubClass).toString(16) +
                    "\tbInterfaceSubclass\tInterface subclass");
            }
            result.push("\t" + iD.bInterfaceProtocol + "\tbInterfaceProtocol\tInterface protocol");
            templateObj.$.ti_textarea.value += result.join("\n");
            result = [];
            const endpoints = iD.endpoints;
            try {
                if (endpoints) {
                    let j = 0;
                    let hasBulkEndpoint = false;
                    for (let n = 0; n < endpoints.length; n++) {
                        var epD = endpoints[n];
                        if (epD) {
                            // Bit 7 of the endpoint address:  Direction, ignored for control endpoints.
                            // 0 = OUT endpoint
                            // 1 = IN endpoint
                            let direction = "IN";
                            if (epD.bEndpointAddress < 0x80) {
                                direction = "OUT";
                            }
                            let xferType = "";
                            const xferTypeValue = getTransferAttributes(epD).transferType;
                            switch (xferTypeValue) {
                                case LIBUSB_TRANSFER_TYPE_CONTROL:
                                    xferType = "CONTROL";
                                    direction = "BIDIRECTIONAL";
                                    break;
                                case LIBUSB_TRANSFER_TYPE_BULK:
                                    xferType = "BULK";
                                    hasBulkEndpoint = true;
                                    break;
                                case LIBUSB_TRANSFER_TYPE_INTERRUPT:
                                    xferType = "INTERRUPT";
                                    break;
                                case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
                                    xferType = "ISOCHRONOUS";
                                    break;
                                default:
                                    xferType = xferTypeValue + " (unknown)";
                                    break;
                            }
                            templateObj.$.ti_textarea.value += result.join("\n");
                            result = [];
                            result.push("\n\t" + "ENDPOINT " + j++ + " direction=" + direction + ", transferType=" + xferType);
                            result.push("\t" + epD.bLength + "\tbLength \t\tDescriptor Size in Bytes");
                            result.push("\t" + epD.bDescriptorType + "\tbDescriptorType\t\tDescriptor Type (Endpoint)");
                            result.push("\t" + epD.bEndpointAddress +
                                "\tbEndpointAddress\tEndpoint Number (0x" + epD.bEndpointAddress.toString(16) + ")");
                            result.push("\t" + epD.bmAttributes + "\tbmAttributes\t\tTransfer Type (" + xferType + ")");
                            /* Bits 0..1 Transfer Type
                            00 = Control
                            01 = Isochronous
                            10 = Bulk
                            11 = Interrupt

                            Bits 2..7 are reserved.If Isochronous endpoint,

                            Bits 3..2 = Synchronisation Type(Iso Mode)
                            00 = No Synchonisation
                            01 = Asynchronous
                            10 = Adaptive
                            11 = Synchronous

                            Bits 5..4 = Usage Type(Iso Mode)
                            00 = Data Endpoint
                            01 = Feedback Endpoint
                            10 = Explicit Feedback Data Endpoint
                            11 = Reserved
                            */
                            // tslint:disable-next-line
                            const maxPktSize = Number(epD.wMaxPacketSize) & 0x7FF;
                            result.push("\t0x" + maxPktSize.toString(16) + "\twMaxPacketSize\t\tMaximum Packet Size");
                            if (Number(epD.maxPacketSize) > 0x0800) {
                                // Bits 12..11 = number of additional transaction opportunities per micro-frame:
                                // 00 = None (1 transaction per micro-frame)
                                // 01 = 1 additional (2 per micro-frame)
                                // 10 = 2 additional (3 per micro-frame)
                                // 11 = Reserved
                                // tslint:disable-next-line
                                const numXactionOpps = Number(epD.maxPacketSize) >> 12;
                                switch (numXactionOpps) {
                                    case 0:
                                        result.push("\t\t\t1 transaction per micro-frame");
                                        break;
                                    case 1:
                                        result.push("\t\t\t1 additional transaction opportunity per micro - frame (2 per micro-frame)");
                                        break;
                                    case 2:
                                        result.push("\t\t\t2 additional transaction opportunity per micro - frame (3 per micro-frame)");
                                        break;
                                    case 3:
                                        result.push("\t\t\tb12..11 of wMaxPacketSize = 3: reserved field - should not be used!");
                                        break;
                                }
                            }
                            result.push("\t" + epD.bInterval + "\tbInterval\t\tPolling Interval (ignored for bulk endpoint)");
                            result.push("\t" + epD.bRefresh + "\tbRefresh");
                            result.push("\t0x" + Number(epD.bSynchAddress).toString(16) + "\tbSynchAddress");
                            if (epD.extra !== undefined) {
                                if (epD.extra.type) {
                                    switch (epD.extra.type) {
                                        case "Buffer":
                                            result.push("\textra: data=" + epD.extra.data);
                                            break;
                                        default:
                                            result.push("\textra: type=" + epD.extra.type);
                                            break;
                                    }
                                }
                            }
                        }
                    }
                    if (hasBulkEndpoint && iD.bInterfaceClass === 255) {
                        result.push(i);
                    }
                    templateObj.$.ti_textarea.value += result.join("\n");
                    result = [];
                }
            } catch (ex) {
                console.log('error displaying endpoint: ex=' + ex);
            }

        }
    } else {
        result.push("No device interface found.");
    }
    return result;
}

function showInfo(item) {
    var result = [];
    if (item) {
        result.push("\t\t\t parentKey: " + item.parentKey);
        result.push("\t\t\t numInterfaces: " + item.numInterfaces);
        result.push("\t\t\t usbPath: " + item.usbPath);
        result.push("\t\t\t canBeOpened: " + item.canBeOpened);
        templateObj.$.ti_textarea.value += result.join("\n");
    }
}

function showDescriptors(descObj) {
    showDeviceDescriptor(descObj.deviceDescriptor);
    showConfigDescriptor(descObj.configDescriptor);
    showInterfaceDescriptors(descObj.interfaceDescriptors);
}

function showVendorIdHelp() {
    var vidpid = templateObj.$.ti_widget_droplist_open.selectedValue;
    if (vidpid) {
        var usIndex = vidpid.indexOf("_");
        var vid = vidpid.substring(0, usIndex);
        var pid = vidpid.substring(usIndex + 1);
        var dollarIndex = pid.indexOf("$");
        if (dollarIndex > 0) {
            pid = pid.substring(0, dollarIndex);
        }
        var dotIndex = pid.indexOf(".");
        if (dotIndex > 0) {
            pid = pid.substring(0, dotIndex);
        }
        window.open('https://www.the-sz.com/products/usbid/index.php?v=0x' + vid + '&p=0x' + pid + '&n=', "_blank");
    }
}

function showProtocolHelp() {
    window.open('http://www.usbmadesimple.co.uk/ums_4.htm', "_blank");
}

function getDeviceInfoFromKey(key) {
    return serviceUsb.deviceInfoMap.get(key);
}

function getStringFromBuffer(data) {
    var strData = "";
    for (var i = 0; i < data.length; i++) {
        strData += String.fromCharCode(data[i]);
    }
    return strData;
}

function getHexFromBuffer(data) {
    var strData = "";
    if (data && data.length) {
        for (var i = 0; i < data.length; i++) {
            strData += "0x" + data[i].toString(16);
            if (i + 1 < data.length) {
                strData += ","
            }
        }
    }
    return strData;
}

function getPaddedString(num, len, radix) {
    str = num.toString(radix);
    return "0".repeat(len - str.length) + str;
}

function getValueFromHexString(hexString) {
    var result = hexString;
    if (!hexString.indexOf) {
        console.log("getValueFromHexString(" + hexString + "): error, must pass in a string");
        return hexString;
    }
    try {
        var temp = hexString.trim().toLowerCase();
        if (temp.indexOf("0x") >= 0) {
            temp = temp.substring(temp.indexOf("0x"));
            result = parseInt(temp, 16);
        } else {
            result = parseInt(temp, 10);
        }
    } catch (ex) {
        console.log("gc.utils.getValueFromHexString(" + hexString + "): ex=" + ex);
    }
    return result;
};

function showMsg(msg, data, isCharArray) {
    var strData = "";

    if (data) {
        if (isCharArray) {
            for (var i = 0; i < data.length; i++) {
                strData += String.fromCharCode(data[i]);
            }
        } else {
            strData = data.toString();
        }
    }
    console.log(msg + strData);
    templateObj.$.ti_textarea.value += "\n" + msg + strData;
}

function showCmdAndResponse(cmd, rxData) {
    if (rxData) {
        showMsg("\nCmd = " + cmd + ": Response = ", rxData.data, true);
    } else {
        if (msTimeout === 0) {
            showMsg("\nCmd = " + cmd + ": No response expected.");
        } else {
            showMsg("\nCmd = " + cmd + ": No response.");
        }
    }
}

function clearTextArea() {
    templateObj.$.ti_textarea.value = "";
}

function pktHdlr(numBytesToCapture, state, rxData) {
    if (state.discardPkts) {
        state.rxCtr += rxData.data.length;
        if (rxData.data.length !== (512 * templateObj.$.ti_widget_droplist_numbufs.selectedValue)) {
            state.numBadPkts++;
        }
        if (state.numPkts % 1000 === 0) {
            if (state.pktTimestamps.length === 0) {
                state.pktTimestamps.push(Date.now());
                state.rxCtr = 0;
            } else {
                if (state.numPkts % 25000 == 0) {
                    clearTextArea();
                }
                var elapsedMs = Date.now() - state.pktTimestamps[0];
                var kBps = state.rxCtr / elapsedMs;
                showMsg("numPkts: " + state.numPkts + ", numBytes: " + state.rxCtr + ", bandwidth: " + Number(kBps).toFixed(1) + " kBytes/s  num bad length Pkts: " + state.numBadPkts);
            }
        }
        state.numPkts++;
        return null;
    }
    const resultObj = {
        errMsg: null,
        result: null
    };
    if (state.hTimeout) {
        clearTimeout(state.hTimeout);
    }
    let done = (numBytesToCapture === undefined);
    if ((!done) && (numBytesToCapture < 0) && (rxData.data.length === 4) && (getStringFromBuffer(rxData.data).indexOf("END") === 0)) {
        resultObj.result = "END. Num Bytes received = " + state.rxCtr;
        var msElapsed = +state.pktTimestamps[state.pktTimestamps.length - 1] - state.pktTimestamps[0];
        resultObj.result += ". Elapsed time for " + state.numPkts + " packets: " + msElapsed + "ms.";
        console.log(resultObj.result);
        done = true;
    }
    if (!done) {
        state.pktTimestamps.push(Date.now());
        state.numPkts++;
        state.pkts.push(rxData.data);
        state.rxCtr += rxData.data.length;
        if ((numBytesToCapture > 0) && (state.rxCtr >= numBytesToCapture)) {
            console.log("Received " + state.rxCtr + " bytes of data - done.");
            resultObj.result = state;
        } else {
            state.hTimeout = setTimeout(() => {
                resultObj.errMsg = "Timeout waiting for data. numPkts = " + state.numPkts + ", pkt timestamps = " + state.pktTimestamps.join(",\n");
                console.log(resultObj.errMsg);
            }, state.msTimeout);
        }
    }
    return resultObj;

}

function pktErrHdlr(rxError) {
    var msg = rxError;
    if (msg.message) {
        msg = rxError.message;
    }
    showMsg(msg);
}

var boundPktHdlr = undefined;

function addRemoveListener(event, forceRelease) {
    var isChecked = templateObj.$.ti_widget_checkbox_addlistener.checked;
    var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
    var selectedIntf = templateObj.$.ti_widget_droplist_claim.selectedValue;
    if (templateObj.$.ti_widget_button_claim.disabled || forceRelease) {
        if (isChecked && !forceRelease) {
            var pktHdlrState = {
                numPkts: 0,
                numBadPkts: 0,
                rxCtr: 0,
                discardPkts: true,
                hTimeout: null,
                msTimeout: 8000,
                pktTimestamps: [],
                pkts: []
            };
            boundPktHdlr = pktHdlr.bind(this, -1, pktHdlrState);
            serviceUsb.addListener(selectedKey, selectedIntf, boundPktHdlr, pktErrHdlr);
        } else {
            serviceUsb.removeListener(selectedKey, selectedIntf, boundPktHdlr, pktErrHdlr);
        }
    }
}

function releaseInterface(event) {
    var ok = true;
    return Q.promise(function(resolve, reject) {
        var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
        var selectedIntf = templateObj.$.ti_widget_droplist_claim.selectedValue;
        var startPolling = templateObj.$.ti_widget_checkbox_polling.checked;
        serviceUsb.releaseInterface(selectedKey, selectedIntf).then(function() {
            if (!templateObj.$.ti_widget_button_release.disabled && templateObj.$.ti_widget_checkbox_addlistener.checked) {
                return serviceUsb.removeListener(selectedKey, selectedIntf, boundPktHdlr, pktErrHdlr);
            } else {
                return;
            }
        }).fail(function(error) {
            ok = false;
            showMsg("Failed to release interface " + selectedIntf + " for device " + selectedKey + ".  Error = " + _getErrorMessage(error));
        }).finally(function() {
            templateObj.$.ti_widget_button_cmds.disabled = true;
            templateObj.$.ti_widget_button_customcmd.disabled = true;
            templateObj.$.ti_widget_button_test.disabled = true;
            templateObj.$.ti_widget_button_claim.disabled = false;
            templateObj.$.ti_widget_button_release.disabled = true;
            if (ok) {
                var isPollingMsg = "";
                if (startPolling) {
                    isPollingMsg = "  Polling Stopped."
                }
                showMsg("Device " + selectedKey + " interface " + " released." + isPollingMsg);
            }
            resolve();
        });
    });
}

function closeDevice() {
    var errMsg;
    var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
    serviceUsb.closeUsbDevice(selectedKey).then(function(msg) {
        showMsg(msg);
    }).fail(function(error) {
        errMsg = error;
        showMsg("Error: " + _getErrorMessage(error));
    }).finally(function() {
        if (!errMsg || errMsg.indexOf("device is not open") >= 0) {

            templateObj.$.ti_widget_button_list.disabled = false;
            templateObj.$.ti_widget_button_open.disabled = false;
            templateObj.$.ti_widget_icon_button_info.disabled = false;
            templateObj.$.ti_widget_label_vendorinfo.disabled = false;
            templateObj.$.ti_widget_button_controltransfer.disabled = true;
            templateObj.$.ti_widget_button_claim.disabled = true;
            templateObj.$.ti_widget_button_release.disabled = true;
            templateObj.$.ti_widget_button_cmds.disabled = true;
            templateObj.$.ti_widget_button_customcmd.disabled = true;
            templateObj.$.ti_widget_button_test.disabled = true;
            templateObj.$.ti_widget_button_close.disabled = true;
            openDeviceInfo = null;
        }
    });
}

// Wait for DOMContentLoaded event before trying to access the application template
var init = function() {
    templateObj = document.querySelector('#template_obj');

    // Wait for the template to fire a dom-change event to indicate that it has been 'stamped'
    // before trying to access components in the application.
    templateObj.addEventListener('dom-change', function() {
        if (initComplete) return;
        this.async(function() {
            initComplete = true;
            console.log("Application template has been stamped.");
            var backplane = gc.services['ti-core-backplane'];
            var connectionMgr = gc.connectionManager;
            connectionMgr.addEventListener('status-changed', function(event) {
                if (connectionMgr.status === "connecting") {
                    if (backplane.currentState.name === 'disconnected') {
                        backplane.updateStateMachine('onConnectBtnClicked');
                    }
                }
                if (connectionMgr.status === "disconnecting") {
                    if ((backplane.currentState.name === 'ready') || (backplane.currentState.name === 'downloadTICloudAgent')) {
                        backplane.updateStateMachine('onDisconnectBtnClicked');
                    }
                }
            });
            // Now that the template has been stamped, you can use 'automatic node finding' $ syntax to access widgets.
            // e.g. to access a widget with an id of 'widget_id' you can use templateObj.$.widgetId
            serviceUsb = templateObj.$.service_usb;
            serviceUsb.addEventListener('usbDeviceListUpdated', function(event) {
                console.log("usbDeviceListUpdated event received");
                templateObj.$.ti_widget_button_list.disabled = false;
                if (templateObj.$.ti_widget_button_open.disabled) {
                    if (serviceUsb.deviceInfoMap.size > 0) {
                        templateObj.$.ti_widget_button_open.disabled = false;
                        templateObj.$.ti_widget_icon_button_info.disabled = false;
                        templateObj.$.ti_widget_label_vendorinfo.disabled = false;
                    }
                }
                templateObj.$.ti_textarea.value = "";
                templateObj.$.ti_widget_droplist_open_labels = "";
                templateObj.$.ti_widget_droplist_open_values = "";

                if (event.detail && event.detail.error) {
                    showMsg("Error when listing devices: " + event.detail.error);
                }
                let i = 0;
                for (const key of serviceUsb.deviceInfoMap.keys()) {
                    const item = serviceUsb.deviceInfoMap.get(key);
                    serviceUsb.getDescriptors(key).then(function(descObj) {
                        showMsg("==== Device " + i++ + ": key=" + key + ", displayName=" + item.displayName + " ====\n");
                        showInfo(item);
                        showDescriptors(descObj);
                        templateObj.$.ti_textarea.value += "\n\n__________________________\n\n";
                    }).fail(function(error) {
                        showMsg("==== Device " + i++ + ": key=" + key + ", displayName=" + item.displayName + " ====\n");
                        var msg = error;
                        if (error.message) {
                            msg = error.message;
                        }
                        showMsg(" Error = " + msg);
                    });
                }
                var keys = Array.from(serviceUsb.deviceInfoMap.keys());
                templateObj.$.ti_widget_droplist_open.labels = keys.join("|");
                templateObj.$.ti_widget_droplist_open.values = templateObj.$.ti_widget_droplist_open.labels;
                templateObj.$.ti_widget_droplist_open.selectedIndex = 0;

            });
            templateObj.$.ti_widget_button_open.addEventListener("click", function(event) {
                templateObj.$.ti_widget_button_claim.disabled = true;
                templateObj.$.ti_widget_button_release.disabled = true;
                templateObj.$.ti_widget_droplist_claim.disabled = true;
                templateObj.$.ti_widget_button_cmds.disabled = true;
                templateObj.$.ti_widget_button_customcmd.disabled = true;
                templateObj.$.ti_widget_button_close.disabled = true;
                var selectedKey;
                if (templateObj.$.ti_widget_droplist_open.selectedValue === undefined) {
                    templateObj.$.ti_widget_droplist_open.selectedIndex = 0;
                }
                selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                if (templateObj.$.ti_widget_droplist_claim.selectedValue !== undefined) {
                    templateObj.$.ti_widget_droplist_claim.selectedIndex = 0;
                }
                console.log("About to call openUsbDevice(" + selectedKey + ")");
                serviceUsb.openUsbDevice(selectedKey).then(function(deviceInfo) {
                    openDeviceInfo = deviceInfo;
                    showMsg("Device " + selectedKey + " opened.");
                    templateObj.$.ti_widget_button_open.disabled = true;
                    templateObj.$.ti_widget_button_claim.disabled = false;
                    templateObj.$.ti_widget_droplist_claim.disabled = false;
                    templateObj.$.ti_widget_button_close.disabled = false;
                    templateObj.$.ti_widget_button_controltransfer.disabled = false;
                    if (deviceInfo.numInterfaces > 0) {
                        var csvValues = [];
                        for (var iIndex = 0; iIndex < deviceInfo.numInterfaces; iIndex++) {
                            csvValues.push(iIndex);
                        }
                        templateObj.$.ti_widget_droplist_claim.labels = csvValues.join(",");
                    } else {
                        templateObj.$.ti_widget_droplist_claim.labels = "0"
                    }
                    templateObj.$.ti_widget_droplist_claim.values = templateObj.$.ti_widget_droplist_claim.labels;
                    templateObj.$.ti_widget_droplist_claim.selectedIndex = 0;
                    serviceUsb.getDescriptors(selectedKey).then(function(descObj) {
                        openDeviceDescriptors = descObj;
                    });
                }).fail(function(error) {
                    showMsg("Failed to open device " + selectedKey + ".  Error = " + _getErrorMessage(error));
                });
            });

            templateObj.$.ti_widget_button_controltransfer.addEventListener("click", function(event) {
                // New for isochronous:
                if (openDeviceInfo) {
                    var selectedKey;
                    if (templateObj.$.ti_widget_droplist_open.selectedValue === undefined) {
                        templateObj.$.ti_widget_droplist_open.selectedIndex = 0;
                    }
                    selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                    const devDesc = openDeviceDescriptors.deviceDescriptor;

                    let descType = LIBUSB_DT_HUB;
                    /* if (devDesc.bDeviceClass === LIBUSB_CLASS_HUB) {

                        serviceUsb.controlTransfer(selectedKey, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_DEVICE,
                            LIBUSB_REQUEST_GET_DESCRIPTOR, descType << 8, 0, 512).then(function(data){
                                showMsg("controlTransfer OK. Data = " + getHexFromBuffer(data));
                            }).fail(function(error){
                                showMsg("controlTransfer error: Error = " + _getErrorMessage(error));
                            });
                    } else {
                    */
                    try {
                        var requestType = getValueFromHexString(templateObj.$.ti_widget_droplist_ct0.selectedValue);
                        var bRequest = getValueFromHexString(templateObj.$.ti_widget_droplist_ct1.selectedValue);
                        var wValue = getValueFromHexString(templateObj.$.ti_widget_textbox_ct2.value);
                        var wIndex = getValueFromHexString(templateObj.$.ti_widget_textbox_ct3.value);
                        var numBytes = getValueFromHexString(templateObj.$.ti_widget_textbox_ct4.value);
                        var param = numBytes;
                        var buf = null;
                        Buffer.alloc(numBytes + 5);
                        if ((requestType & LIBUSB_ENDPOINT_IN) === 0) {
                            buf = Buffer.alloc(numBytes + 5);
                            param = buf;
                        }
                        serviceUsb.controlTransfer(selectedKey, requestType, bRequest, wValue, wIndex, param).then(function(response) {
                            var data = response;
                            if (response && !Array.isArray(response) && response.data) {
                                data = response.data;
                            }
                            showMsg("controlTransfer OK. Data = " + getHexFromBuffer(data));
                            showMsg("  str = " + new Buffer(data).toString("utf16le"));
                        }).fail(function(error) {
                            showMsg("controlTransfer error: Error = " + _getErrorMessage(error));
                        });
                    } catch (ex) {
                        console.log("Exception: bad controlTransfer params! ex=" + ex);
                    }
                    //}
                }

            });
            templateObj.$.ti_widget_button_claim.addEventListener("click", function(event) {
                var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                var selectedIntf = templateObj.$.ti_widget_droplist_claim.selectedValue;
                var startPolling = templateObj.$.ti_widget_checkbox_polling.checked;
                var numBufs = templateObj.$.ti_widget_droplist_numbufs.selectedValue;
                serviceUsb.claimInterface(selectedKey, selectedIntf, startPolling, numBufs).then(function() {
                    clearTextArea();
                    var isPollingMsg = "";
                    if (startPolling) {
                        isPollingMsg = "  Polling Started."
                    }
                    showMsg("Device " + selectedKey + " interface " + " claimed." + isPollingMsg);
                    templateObj.$.ti_widget_button_cmds.disabled = false;
                    templateObj.$.ti_widget_button_customcmd.disabled = false;
                    templateObj.$.ti_widget_button_test.disabled = false;
                    templateObj.$.ti_widget_button_claim.disabled = true;
                    templateObj.$.ti_widget_button_release.disabled = false;

                    if (templateObj.$.ti_widget_checkbox_addlistener.checked) {
                        addRemoveListener(null, false);
                    }
                }).fail(function(error) {
                    showMsg("Failed to open device " + selectedKey + ".  Error = " + _getErrorMessage(error));
                });
            });

            templateObj.$.ti_widget_button_release.addEventListener("click", releaseInterface);

            templateObj.$.ti_widget_button_cmds.addEventListener("click", function(event) {
                var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                var selectedIntf = templateObj.$.ti_widget_droplist_claim.selectedValue;
                msTimeout = +templateObj.$.ti_widget_textbox_timeout.value;

                var pktHdlrState = {
                    numPkts: 0,
                    numBadPkts: 0,
                    rxCtr: 0,
                    discardPkts: false,
                    hTimeout: null,
                    msTimeout: 4000,
                    pktTimestamps: [],
                    pkts: []
                };
                clearTextArea();
                var cmd = "RESET";
                serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "ID";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "WREG 02 48";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "WREG 04 C0";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "SYOCAL";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "REGMAP";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    showCmdAndResponse(cmd, rxData);
                    cmd = "COLLECTSETUP 2400";
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout);
                }).then((rxData) => {
                    if (rxData) {
                        showMsg("\nCmd = " + cmd + "\nResponse = ", rxData.data, false);
                    } else {
                        showMsg("\nCmd = " + cmd + ": No response.", null, false);
                    }
                    cmd = "COLLECT 2400";
                    pktHdlrState.pktTimestamps.push(Date.now());
                    return serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout, pktHdlr.bind(this, -1, pktHdlrState));
                }).then((resultObj) => {
                    if (resultObj) {
                        if (typeof(resultObj) === "string") {
                            showMsg("\nCmd = " + cmd + "\nResponse = ", resultObj, false)
                        } else {
                            if (resultObj) {
                                showMsg("Cmd = " + cmd + " elapsed time for " + resultObj.numPkts + " packets: " + resultObj.pktTimestamps[resultObj.pktTimestamps.length - 1] - resultObj.pktTimestamps[0] + "ms.");
                            }
                        }
                    } else {
                        showCmdAndResponse(cmd, resultObj);
                    }
                }).fail(function(error) {
                    showMsg("Error: " + _getErrorMessage(error));
                });
            });
            templateObj.$.ti_widget_button_customcmd.addEventListener("click", function(event) {
                var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                var selectedIntf = templateObj.$.ti_widget_droplist_claim.selectedValue;
                msTimeout = +templateObj.$.ti_widget_textbox_timeout.value;

                var cmd = templateObj.$.ti_widget_textbox_cmd.value;
                if (cmd.indexOf("COLLECT ") === 0) {
                    var pktHdlrState = {
                        numPkts: 0,
                        numBadPkts: 0,
                        rxCtr: 0,
                        discardPkts: false,
                        hTimeout: null,
                        msTimeout: 4000,
                        pktTimestamps: [],
                        pkts: []
                    };
                    pktHdlrState.pktTimestamps.push(Date.now());
                    serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout, pktHdlr.bind(this, -1, pktHdlrState)).then(function(resultObj) {
                        if (typeof(resultObj) === "string") {
                            showMsg("Cmd = " + cmd + "\nResponse = " + resultObj);
                        } else {
                            if (resultObj) {
                                showMsg("Cmd = " + cmd + " elapsed time for " + resultObj.numPkts + " packets: " + resultObj.pktTimestamps[resultObj.pktTimestamps.length - 1] - resultObj.pktTimestamps[0] + "ms.");
                            }
                        }
                    }).fail(function(error) {
                        showMsg("Error: " + _getErrorMessage(error));
                    });
                } else {
                    serviceUsb.sendCmd(selectedKey, selectedIntf, cmd, msTimeout, null).then(function(rxData) {
                        var isString = true;
                        if ((cmd.indexOf("COLLECTSETUP") === 0) || (cmd.indexOf("0x") === 0)) {
                            isString = false;
                        }
                        if (rxData) {
                            showMsg("Cmd " + cmd + " sent. \nResponse = ", rxData.data, isString);
                        } else {
                            if (msTimeout === 0) {
                                showMsg("Cmd " + cmd + " sent.  No response expected.");
                            } else {
                                showMsg("Cmd " + cmd + " sent.  No response.");
                            }
                        }
                    }).fail(function(error) {
                        showMsg("Error: " + _getErrorMessage(error));
                    });
                }
            });
            templateObj.$.ti_widget_button_test.addEventListener("click", function(event) {
                msTimeout = +templateObj.$.ti_widget_textbox_timeout.value;

                var numPkts = templateObj.$.ti_widget_textbox_numpkts.value;
                var pktSizeInBytes = templateObj.$.ti_widget_textbox_pktsize.value;

                var pktHdlrState = {
                    numPkts: 0,
                    numBadPkts: 0,
                    rxCtr: 0,
                    discardPkts: false,
                    hTimeout: null,
                    msTimeout: 4000,
                    pktTimestamps: [],
                    pkts: []
                };

                showMsg(`Starting UploadTestPackets: ${numPkts} packets, size = ${pktSizeInBytes} bytes (must be multiple of 4)`);
                pktHdlrState.pktTimestamps.push(Date.now());
                serviceUsb.uploadTestPackets(numPkts, pktSizeInBytes, pktHdlr.bind(this, pktSizeInBytes * numPkts, pktHdlrState)).then(function(resultObj) {
                    if (resultObj) {
                        var elapsedMs = resultObj.pktTimestamps[resultObj.pktTimestamps.length - 1] - resultObj.pktTimestamps[0];
                        var numBytes = pktSizeInBytes * numPkts;
                        var kbps = numBytes / elapsedMs;
                        showMsg("UploadTestPackets: elapsed time for " + resultObj.numPkts + " packets (" + numBytes + " Bytes): " + elapsedMs + " ms.  Bandwidth = " + kbps.toFixed(2) + " kBytes/sec");
                        // validate all packet contents:
                        var n = 0;
                        var numErrors = 0;
                        for (var i = 0; i < resultObj.numPkts; i++) {
                            var byteArray = new Uint8Array(resultObj.pkts[i]);
                            var wordArray = new Uint32Array(byteArray.buffer);
                            for (var j = 0; j < wordArray.length; j++) {
                                if (n !== wordArray[j]) {
                                    numErrors++;
                                }
                                n++;
                            }
                        }
                        showMsg("Packet validation: " + n + " values checked, numDataErrors = " + numErrors);
                    }
                }).fail(function(error) {
                    showMsg("Error: " + _getErrorMessage(error));
                });
            });

            templateObj.$.ti_widget_checkbox_addlistener.addEventListener("changed", addRemoveListener.bind(this));
            templateObj.$.ti_widget_icon_button_info.addEventListener("click", function(event) {
                showVendorIdHelp();
            });

            templateObj.$.ti_widget_label_vendorinfo.addEventListener("click", function(event) {
                showVendorIdHelp();
            });

            templateObj.$.ti_widget_icon_button_protocolhelp.addEventListener("click", function(event) {
                showProtocolHelp();
            });
            templateObj.$.ti_widget_label_protocolhelp.addEventListener("click", function(event) {
                showProtocolHelp();
            });

            templateObj.$.ti_widget_button_close.addEventListener("click", function(event) {
                var selectedKey = templateObj.$.ti_widget_droplist_open.selectedValue;
                var errMsg = null;
                if (!templateObj.$.ti_widget_button_release.disabled) {
                    releaseInterface().then(function() {
                        closeDevice();
                    })
                } else {
                    closeDevice();
                }

            });

            templateObj.$.ti_widget_button_clear.addEventListener("click", function(event) {
                clearTextArea();
            });

            backplane = gc.services['ti-core-backplane'];

            if (backplane) {

                templateObj.$.ti_widget_button_connect.addEventListener("click", function(event) {
                    if (templateObj.$.ti_widget_button_connect.label === "CONNECT") {
                        templateObj.$.ti_widget_button_connect.label = "CONNECTING...";
                        backplane.connect();
                    } else {
                        templateObj.$.ti_widget_button_connect.label = "DISCONNECTING...";
                        backplane.disconnect();
                    }
                });

                templateObj.$.ti_widget_button_list.addEventListener("click", function(event) {
                    serviceUsb.vendorIdFilter = templateObj.$.ti_widget_textbox_vendorids.value;
                    clearTextArea();
                    serviceUsb.listDevices().then(function(devMap){
                        if (devMap.size === 0) {
                            showMsg("No devices found for filter=\""+serviceUsb.vendorIdFilter+"\"");
                        }
                    });

                });

                backplane.addEventListener("btnVisibilityChanged", function(event) {
                    var _connected = false;
                    serviceUsb.vendorIdFilter = templateObj.$.ti_widget_textbox_vendorids.value;
                    if (backplane.isDisconnectBtnVisible) {
                        _connected = true;
                        templateObj.$.ti_widget_button_connect.label = "DISCONNECT";
                    }
                    if (!_connected) {
                        templateObj.$.ti_widget_button_list.disabled = true;
                        templateObj.$.ti_widget_button_open.disabled = true;
                        templateObj.$.ti_widget_icon_button_info.disabled = true;
                        templateObj.$.ti_widget_label_vendorinfo.disabled = true;
                        templateObj.$.ti_widget_button_claim.disabled = true;
                        templateObj.$.ti_widget_button_release.disabled = true;
                        templateObj.$.ti_widget_button_cmds.disabled = true;
                        templateObj.$.ti_widget_button_customcmd.disabled = true;
                        templateObj.$.ti_widget_button_test.disabled = true;
                        templateObj.$.ti_widget_button_close.disabled = true;
                    }
                    if (backplane.isConnectBtnVisible) {
                        templateObj.$.ti_widget_button_connect.label = "CONNECT";
                    }
                });
            }
        }, 1);

    });
};

templateObj = document.querySelector('#template_obj');
if (templateObj) {
    init();
} else {
    document.addEventListener('DOMContentLoaded', init.bind(this))
}