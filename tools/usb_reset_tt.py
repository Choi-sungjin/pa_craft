#!/usr/bin/env python3
"""Reset the Transaction Translator in front of a wedged full-speed USB device (an OpenCR after an upload).

After an upload the OpenCR drops off USB at the jump to the new firmware; sometimes it comes back and every
descriptor read times out (kernel: 'usb 1-2.4.3: device descriptor read/64, error -110') until the hub is
re-bound (usbfix, 1-2 min, drops everything on the hub). The OpenCR is full speed behind a high-speed hub,
so it goes through the hub's TT; a transfer cut off by the disconnect can leave that TT busy. This sends the
hub class request RESET_TT (USB 2.0 11.24.2.9) so the kernel's next enumeration retry gets through.
--power-cycle switches the hub port off and on instead (Clear/SetPortFeature PORT_POWER), what the kernel itself
does after four failed retries (~70 s; 2026-11-05 19:31:43 it brought the arm board back in 2 s).

Needs write access to the hub's /dev/bus/usb node (udev rule 50-craft-usb-hub-reset-tt.rules).
Usage: usb_reset_tt.py 1-2.4.3 [--power-cycle]     (the device path from the kernel message)
"""
import errno
import fcntl
import os
import struct
import sys
import time
from pathlib import Path

USBDEVFS_CONTROL = 0xC0185500  # _IOWR('U', 0, struct usbdevfs_ctrltransfer), 24 bytes on 64-bit
RESET_TT = 9
CLEAR_FEATURE, SET_FEATURE, PORT_POWER = 1, 3, 8
HUB_CLASS_OTHER_OUT = 0x23     # host to device, class, recipient other (port)


def control(fd, request, value, index):
    """Hub class request to recipient 'other' (a port / TT), no data stage.
    struct usbdevfs_ctrltransfer: u8 bRequestType, u8 bRequest, u16 wValue, u16 wIndex, u16 wLength,
    u32 timeout [ms], void *data"""
    fcntl.ioctl(fd, USBDEVFS_CONTROL, struct.pack('=BBHHHIxxxxQ', HUB_CLASS_OTHER_OUT, request, value, index, 0,
                                                   1000, 0))


def main():
    args = [a for a in sys.argv[1:] if a != '--power-cycle']
    cycle = '--power-cycle' in sys.argv
    if len(args) != 1 or '-' not in args[0]:
        print(__doc__.strip().splitlines()[-1]); return 2
    dev = args[0]
    hub_path, port = dev.rsplit('.', 1) if '.' in dev.split('-', 1)[1] else (None, None)
    if hub_path is None:
        print(f'{dev} hangs off a root port: no TT to reset'); return 1
    hub = Path('/sys/bus/usb/devices') / hub_path
    read = lambda name: (hub / name).read_text().strip()
    if read('speed') != '480':
        print(f'{hub_path} is not a high-speed hub: no TT'); return 1
    # bDeviceProtocol 1: one TT for all ports (TT_port must be 1); 2: one TT per port.
    tt_port = 1 if read('bDeviceProtocol') == '01' else int(port)
    node = f'/dev/bus/usb/{int(read("busnum")):03d}/{int(read("devnum")):03d}'
    try:
        fd = os.open(node, os.O_RDWR)
    except OSError as e:
        hint = ' (udev rule 50-craft-usb-hub-reset-tt.rules missing?)' if e.errno == errno.EACCES else ''
        print(f'{node}: {e.strerror}{hint}'); return 1
    try:
        if cycle:
            control(fd, CLEAR_FEATURE, PORT_POWER, int(port))
            time.sleep(0.5)
            control(fd, SET_FEATURE, PORT_POWER, int(port))
        else:
            control(fd, RESET_TT, 0, tt_port)
    except OSError as e:
        what = f'power cycle of port {port}' if cycle else f'RESET_TT on TT {tt_port}'
        print(f'{what} on hub {hub_path} failed: {e.strerror}'); return 1
    finally:
        os.close(fd)
    if cycle:
        print(f'port {port} of hub {hub_path} switched off and on'); return 0
    kind = 'single-TT, shared by every port' if tt_port == 1 and read('bDeviceProtocol') == '01' else f'port {tt_port}'
    print(f'RESET_TT sent to hub {hub_path} ({read("idVendor")}:{read("idProduct")}, {kind})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
