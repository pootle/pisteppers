#!/usr/bin/python3
"""
a module to log everything on a pigpio.pi instance
"""

import pigpio, time

class plog():
    def __init__(self):
        self.pio=pigpio.pi()
        self.logf=open('piolog.log','w')
        self.basetime=time.monotonic()
        self.lastio=self.basetime
        self.logit('log starts','')
        
    def logit(self, call, value):
        newt=time.monotonic()
        self.logf.write('{:9.4f} {:12.4f} {:20s} {}\n'.format(time.monotonic()-self.basetime, (newt-self.lastio)*1000, call, value))
        self.lastio=newt

    @property
    def connected(self):
        con=self.pio.connected
        self.logit('connected', '')
        return con

    def wave_get_max_micros(self):
        v=self.pio.wave_get_max_micros()
        self.logit('wave_get_max_micros', '%d' % v)
        return v

    def wave_get_max_pulses(self):
        v=self.pio.wave_get_max_pulses()
        self.logit('wave_get_max_pulses', '%d' % v)
        return v

    def wave_get_max_cbs(self):
        v=self.pio.wave_get_max_cbs()
        self.logit('wave_get_max_cbs', '%d' % v)
        return v

    def set_mode(self, gpio, mode):
        self.logit('set_mode', 'pin %2d, mode %s' % (gpio, iomodes[mode]))
        self.pio.set_mode(gpio, mode)

    def write(self, gpio, level):
        self.logit('write', 'pin %2d -> %d' % (gpio, level))
        self.pio.write(gpio,level)

    def gpio_trigger(self, gpio, pulse_len, level):
        self.logit('gpio_trigger', 'pin %2d pulse %s for %d microseconds' % (gpio, 'high' if level > 0 else 'low ', pulse_len))
        self.pio.gpio_trigger( gpio, pulse_len, level)

    def wave_clear(self):
        self.logit('wave_clear', '')
        self.pio.wave_clear()

    def wave_add_generic(self, pulses):
        pl=self.pio.wave_add_generic(pulses)
        if len(pulses) == 0:
            self.logit('wave_add_generic', 'no pulses, length now %d' % pl)
        else:
            self.logit('wave_add_generic', '%d pulses, length now %d' % (len(pulses), pl))
            for p in pulses:
                self.logit('       pulse', 'on: %x, off: %x, delay %d' % (p.gpio_on, p.gpio_off, p.delay))

    def wave_create(self):
        wid=self.pio.wave_create()
        self.logit('wave_create','wave id: %d' % wid)
        return wid

    def stop(self):
        self.pio.stop()
        self.pio=None
        self.logit('stop', '')
        self.logf.close()
        self.logf=None

iomodes={
    pigpio.INPUT    : 'INPUT', 
    pigpio.OUTPUT   : 'output',
    pigpio.ALT0     : 'alt0',
    pigpio.ALT1     : 'alt1',
    pigpio.ALT2     : 'alt2',
    pigpio.ALT3     : 'alt3',
    pigpio.ALT4     : 'alt4',
    pigpio.ALT5     : 'alt5'}
      