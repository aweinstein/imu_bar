import time

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat
import serial


#def save_data(t, p, alt, t_filt, p_filt, alt_filt):
def save_data(times, t, p, alt):
    now = time.strftime('%Y%m%dT%H%M%S')
    file_name = 'logs/bmp085_%s.npy' % now
    mat_file_name = 'logs/bmp085_%s.mat' % now
    ## np.save(file_name, np.array([t, p, alt, t_filt, p_filt, alt_filt]))
    ## d = {'temperature':t, 'pressure':p, 'altitude':alt,
    ##      'temperature_filt':t_filt, 'pressure_filt':p_filt,
    ##      'altitude_filt':alt_filt}

    np.save(file_name, np.array([times, t, p, alt]))
    d = {'time':times, 'temperature':t, 'pressure':p, 'altitude':alt}
    savemat(mat_file_name, d, oned_as='row')
    print 'Data saved in %s and %s' % (file_name, mat_file_name)

def p2alt(p, p0=1015):
    """Convert barometric pressure to altitude.

    Convertion based on  international barometric formula.

    Parameters
    ----------
    p : array_like
        Pressures in hecto pascals
    p0 : float
        Pressure at sea level

    Returns
    -------
    array_like
        Altitudes over sea level, in meters
    """
    exp = 1 / 5.255
    alt = 44330. * (1 - (p / p0)**exp)
    return alt

def plot_data(t, p, alt, t_filt=[], p_filt=[], alt_filt=[], ups=[], uts=[]):
    time = np.linspace(0, len(t)*(1/5.), len(t))
    #f, (a1, a2, a3, a4, a5) = plt.subplots(5, sharex=True)
    _, (a1, a2, a3) = plt.subplots(3, sharex=True)
    a1.plot(time, t)
    a1.set_ylabel('Temperature [C]')
    if len(t_filt) > 0:
        a1.plot(time, t_filt)
        a1.set_ylim(min(t + t_filt) - 2, max(t + t_filt) + 2)
        a1.legend(('raw', 'filt'))

    a2.plot(time, p)
    a2.set_ylabel('Pressure [Pa]')
    if len(p_filt) > 0:
        a2.plot(time, p_filt)
        a2.set_ylim(min(p + p_filt) - 100, max(p + p_filt) + 100)

    a3.plot(time, alt)
    a3.set_ylabel('Altitude [m]')
    if len(alt_filt) > 0:
        a3.plot(time, alt_filt)
        alts = np.hstack((alt, alt_filt))
        a3.set_ylim(min(alts) - 1, max(alts) + 1)

    ## a4.plot(time, ups)
    ## a4.set_ylabel('UP')

    ## a5.plot(time, uts)
    ## a5.set_ylabel('UT')

    ## for a in (a1, a2, a3, a4, a5):
    ##     a.grid()

if __name__ == '__main__':
    port = '/dev/ttyACM0'
    bauds = 115200
    ser = serial.Serial(port, bauds)
    temps, press =  [], []
    times = []
    ## temps_filt, press_filt = [], [],
    ## ups, uts = [], []
    print 'Start logging at', time.asctime()
    start = time.time()
    i = 0
    try:
        while True:
            line = ser.readline().strip()
            ts = time.time()
            if len(line) == 0: continue
            if line[0] == '#' and line[-1] == '$':
                values = line[1:-1].split(',')
                if len(values) == 2:
                    #t, t_filt, p, p_filt, up, ut = values
                    t, p = values
                    temps.append(float(t) / 10.)
                    press.append(float(p))
                    times.append(ts)
                    ## temps_filt.append(float(t_filt) / 10.)
                    ## press_filt.append(float(p_filt))
                    ## ups.append(up)
                    ## uts.append(ut)
                    i += 1
                    if i == 10:
                        #print t, t_filt, p, p_filt
                        s = 'time: %.2f temperature: %.1f pressure: %.1f'
                        print s % ((ts -  start),
                                   float(t) / 10.,
                                   float(p) / 100.)
                        i = 0
                else:
                    print line
    except KeyboardInterrupt:
        end = time.time()

    ser.close()
    print
    print 'Stopped logging at', time.asctime()
    f = len(temps) / (end - start)
    print 'Logged %d data points with frequency %.1f' % (len(temps), f)
    altitude = p2alt(np.array(press)/100)
    #altitude_filt = p2alt(np.array(press_filt)/100)
    #save_data(temps, press, altitude, temps_filt, press_filt, altitude_filt)
    #plot_data(temps, press, altitude, temps_filt, press_filt, altitude_filt, ups, uts)
    save_data(times, temps, press, altitude)
    plot_data(temps, press, altitude)
    plt.show()
