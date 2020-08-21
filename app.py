#!/usr/bin/python3
"""
The app just does some simple checks to avoid long error messages later, then launches the web app which does all the hard work
"""
import sys
try:
    from pootlestuff import webserv
except:
    print('cannot import webserv from pootlestuff. Is it installed?', file=sys.stderr)
    sys.exit(1)
try:
    import pigpio
except:
    print('cannot import pigpio. Is it installed?', file=sys.stderr)
    sys.exit(1)
pio=pigpio.pi(show_errors=False)
if not pio.connected:
    print('unable to connect to pigpio daemon. Please start it first.', file=sys.stderr)
    sys.exit(1)
try:
    x=pio.wave_create_and_pad
except:
    print('The installed version of pigpio does not support wave_create_and_pad. Please update.\nSee http://abyz.me.uk/rpi/pigpio/download.html for info', file=sys.stderr)
    sys.exit(1)
import time
abs_start=time.time()
abs_thread=time.thread_time()
abs_process=time.process_time()
webserv.runmain()
print('Execution summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (time.time()-abs_start, time.process_time()-abs_process, time.thread_time()-abs_thread))