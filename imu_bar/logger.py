import argparse
import os
import sys
import time

import serial


def save_data(lines):
    now = time.strftime('%Y%m%dT%H%M%S')
    file_name = os.path.join('logs', 'log_%s.txt' % now)
    with open(file_name, 'w') as f:
        f.write('\n'.join(lines))
        f.write('\n')
    print 'Data saved in', file_name

def reset():
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)

if __name__ == '__main__':

    port = '/dev/ttyACM0' if os.name == 'posix' else 'COM1'
    parser = argparse.ArgumentParser(description='IMU & Barometer Logger')
    parser.add_argument('-p', '--port', action='store', default=port,
                        help='Server address')
    args = parser.parse_args()
    port = args.port
    bauds = 115200
    try:
        ser = serial.Serial(port, bauds, timeout=5.0)
    except serial.SerialException, e:
        print 'Error opening serial port:', e
        sys.exit(-1)
    reset()
    print 'Connected to port %s at %d bauds' % (port, bauds)
    print 'Waiting for start of stream'
    lines = []
    n = 0
    last_len = 0
    try:
        while True:
            line = ser.readline()
            if line.startswith('ready'):
                start_time = time.asctime()
                print 'Start logging at', start_time
                start = time.time()
                break
            if len(line) == 0 or line[-1] != '\n':
                print 'Timeout!, restarting...'
                reset()
        while True:
            line = ser.readline().strip()
            ts = time.time()
            lines.append('%f,%s' % (ts, line))
            n += 1
            if (n % 50) == 0:
                print 'time: %.2f %s' % ((ts - start), line)
                last_len = len(lines)
    except KeyboardInterrupt:
        end = time.time()
        pass

    f = len(lines) / (end - start)
    print '\n\nFrequency: %.4f' % f

    ser.close()
    print
    print 'Stopped logging at', time.asctime()
    save_data(lines)
