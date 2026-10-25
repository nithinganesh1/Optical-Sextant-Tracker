
    
  
"""Diagnose ESP32 firmware serial protocol without triggering motor movement by default.
Usage: python3 -u serial_check.py /dev/ttyUSB0
Optional: python3 -u serial_check.py /dev/ttyUSB0 --home (ONLY after verifying switches)
"""
import argparse, time
import serial

p = argparse.ArgumentParser()
p.add_argument('port', nargs='?', default='/dev/ttyUSB0')
p.add_argument('--home', action='store_true', help='Send homing command; moves motors!')
a = p.parse_args()

with serial.Serial(a.port, 115200, timeout=0.2) as ser:
    print(f'Opened {a.port} at 115200', flush=True)
    time.sleep(2)
    def receive(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            raw = ser.readline()
            if raw:
                print('<-', raw.decode('ascii', errors='replace').rstrip(), flush=True)
    receive(3)
    for cmd in ('L', 'P', 'I') + (('H',) if a.home else ()):
        print('->',cmd,flush=True)
        ser.write((cmd+'\n').encode('ascii'))
        receive(10 if cmd == 'H' else 2)
    print('Finished. H was not sent.' if not a.home else 'H command sent.', flush=True)

