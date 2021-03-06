#!/usr/bin/python3

"""
A bunch of classes that wrap gpio pin driving.

The 2 logic states are named and allow meaningful names to be used.

Reversing the names inverts the mapping.

The classes also provide bit settings to allow pins to be driven using DMA for high speed, high precision timings. 
"""
import pootlestuff.watchables as wv
from pootlestuff.watchables import loglvls
from enum import Enum
import pigpio

class pudvals(Enum):
    pud_off   = pigpio.PUD_OFF
    pud_up    = pigpio.PUD_UP
    pud_down  = pigpio.PUD_DOWN

class gpio_out_pin(wv.enumWatch):
    """
    specific extras for a pin used for direct output
    """
    def __init__(self, *, name, value, app, **kwargs):
        """
        defines values that will be used to write 1 and 0 to the pin, and optionally a value
        to write when closed down (leaving the pin in output mode)

        name    : the name is used in log messages
        
        app     : an object with a pigpio instance available, and that provides a log function

        value   : a dict with keys:
                
                value       : (optional - default the first entry in vlist (logic low)) otherwise the initial value for the pin (an entry in vlist)
                pinno       : the broadcom pin no - can be -1 when there is no pin (typically because the relevant pin is hardwired and not accessible to this software
                vlist       : (optional- default (0,1)) a 2-tuple, the 1st entry is the value that will result in the pin set low and the second, the pin set high
                closevalue  : (optional) - the value set on the ouput pin on closedown - the pin is left in output mode
                              if not present / None the pin is reset to input mode on closedown
        """
        self.name=name
        if value is None:
            value={}
        vl=value.get('vlist', (0,1))
        assert len(vl) == 2
        self.closevalue=value.get('closevalue', None)
        if not self.closevalue is None:
            assert self.closevalue in vl
        val=value.get('value', vl[0])
        assert val in vl
        self.pigp=app.pio
        self.pinno = value['pinno']
        assert -1 <= self.pinno <= 31
        super().__init__(value=val, vlist=vl, app=app, **kwargs)
        if self.pinno > 0:
            self.pigp.set_mode(self.pinno, pigpio.OUTPUT)
            self._setPin(self.getIndex())
            self.log(loglvls.INFO, '%s (pin %d) set for output, initialised to %s (%0d)' % (self.name, self.pinno, val, vl.index(val)))
        else:
            self.log(loglvls.INFO, 'no physical pin for %s' % self.name)
        
    def shutdown(self):
        if self.pinno >= 0:
            if self.closevalue is None:
                self.pigp.set_mode(self.pinno, pigpio.INPUT)
                self.log(loglvls.INFO, '%s (pin %d) shutdown, set to input mode' % (self.name, self.pinno))
            else:
                self._setPin(self.vlist.index(self.closevalue))
                self.log(loglvls.DEBUG, '%s (pin %d) closing - set to %s' % (self.name, self.pinno, self.closevalue))

    def _setPin(self, pinval):
        if self.pinno >= 0:
            self.pigp.write(self.pinno, pinval)

    def setValue(self, value, agent):
        changed = super().setValue(value, agent)
        if changed:
            self._setPin(self.getIndex())
        return changed

    def getValue(self, simple=True):
        if simple:
            return super().getValue()
        else:
            return  {k: getattr(self,k) for k in ('name', 'pinno', 'pud', 'vlist', 'value', 'closevalue', 'loglvl')}

    def getBits(self, value=None, agent=None):
        """
        returns the bits-on / bits-off for the pin to use with pigpio waves.
        
        With default value, it returns the current value bit patterns, if the value is provided it returns the bits
        for that value.
        
        This pin values are controlled by the wave maker, so when the wave is finished resetValue should be called so the gpio
        pin and the stored value here are consistent.
        """
        if value is None:
            value=super().getValue()
        if self.pinno < 0:
            return None, None
        bim=1<<self.pinno
        return (0,bim) if value == self.vlist[0] else (bim,0)

    def resetValue(self, value=None, agent=None):
        if value is None:
            value=self.getValue()
        else:
            value=self.validValue(self, value, agent)
            self._val=value
        self._setPin(0 if value==self.vlist[0] else 1)

    def __repr__(self):
        return str(self.getValue())

class gpio_trigger_pin(gpio_out_pin):
    """
    An output pin with trigger capability.
    
    when trigger is called, the pin will be set to 1 if pulse1 else 0, for pulsetime microseconds, then revert to the inverse state
    """
    def __init__(self, *, value, **kwargs):
        """
        value is a dict with the following keys in addition to those specified for an output pin:
            
            pulsetime   : the time in microseconds for the pulse
        
            pulse1      : (default True) True to pulse logic 1 and revert to 0 else pulse logic 0 and revert to 1. Note this bypasses the 
                          value['vlist'] used by gpio_out_pin.
        
        """
        self.pulsetime=value['pulsetime']
        vl=(0,1) if value.get('pulse1', True) else (1,0)
        nval=value.copy()
        del nval['pulsetime']
        nval['vlist']=vl
        super().__init__(value=nval, **kwargs)

    def trigger(self):
        if self.pinno >= 0:
            val= self.vlist[1]
            self.pigp.gpio_trigger(self.pinno, self.pulsetime, val)
            if self.loglevel is loglvls.VAST.value:
                self.log(loglvls.DEBUG, '%s (pin %d) triggered %s for %0d microseconds' % (self.name, self.pinno, val, self.pulsetime))

    def getValue(self):
        rd = {k: getattr(self,k) for k in ('name', 'pinno', 'pulsetime', 'closevalue', 'loglvl')}
        rd['pulse1'] = self.vlist[0]==0
        return rd

class usteplevel_pinset(wv.enumWatch):
    """
    manages the microsteplevel on the motor controller where a set of pins define the step level
    """
    def __init__(self, app, value, loglvl=loglvls.INFO, **kwargs):
        """
        looks after the gpiopins that define the microstepping level
        
        value       : initial value - dict of setup info - see below.
        
        Note: if no pins are available to control the microstep level (because they are hard wired or set via dip switches etc.)
              then value['pins'] should be an empty list and value['microsteps'] should have 1 entry typically:
                  value['microsteps']={1:[]}
        
        the dict passed as value defines the available microstep levels and how they are controlled as well as a set of named 'stepmodes'
        that the motor class uses when driving the motor. Dict keys are:
        
        'pins'      : the (broadcam) pin numbers used to set the microstep levels, a list of pin numbers
        
        'microsteps': a dict that defines the mapping of available microstep levels to pin values. Each entry has:
                (key)       : the name used for this microstep setting.
                'factor'    : the number of steps this setting needs to perform 1 full motor step - used by the motor class to track
                              the motor's position.
                'table'     : a list of pin values (0 or 1) this microstep setting requires. Must be the same length as the list of pins
        
        'value'     : The initial value to be set - must be one of the keys in microsteps
        """
        assert not value is None
        assert 'pins' in value
        self.name='usteplevel'
        pinset=value['pins']
        pincount=len(pinset)
        self.microstepset=value['microsteps']
        initialval=value.get('value', list(self.microstepset.keys())[0])
        self.maxmicrosteps=max([msv['factor'] for msv in self.microstepset.values()])
        for spname, spvals in self.microstepset.items():
            assert len(spvals['table'])==pincount, 'wrong number of pin values in "microsteps" entry for %s' % spname
        self.mspins=[]
        for px, pin_info in enumerate(pinset):
            self.mspins.append(gpio_out_pin(app=app, value=pin_info, name= 'usl %d (gpio %02d)' % (px, pin_info['pinno'])))        
        super().__init__(app=app, value=value['value'], vlist=list(self.microstepset.keys()), **kwargs)
        self.setValue(value['value'], wv.myagents.app)
        self.log(loglvls.INFO,'microstep pins %s created' % str(pinset))

    def shutdown(self):
        for pinv in self.mspins:
            pinv.shutdown()
        self.log(loglvls.INFO, 'microstep pins shutdown')

    def maxusteps(self):
        return self.maxmicrosteps
 
    def setValue(self, value, agent):
        if isinstance(agent,self.app.agentclass):
            super().setValue(value, agent)
            vset=self.microstepset[self.getValue()]['table']
            vpins=self.mspins
            for msi in range(len(vset)):
                vpins[msi].setValue(vset[msi], agent)
            return True
        else:
            self.log(loglvls.CRITICAL, 'agent {} not known in setting var {}'.format(agent, self.name))
        return False

    def pinbits(self, value=None):
        """
        returns a 2 tuple with the bits-on and bits-off needed for a dma wave pulse
        """
        pinvals=self.microstepset[self.getValue() if value is None else value]['table']
        onbits=0
        offbits=0
        for ix, apin in enumerate(self.mspins):
            pon, poff = apin.getBits(pinvals[ix])
            onbits |= pon
            offbits |= poff
        return onbits, offbits