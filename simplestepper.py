#!/usr/bin/python3

"""
A module to drive a stepper motor via a basic driver chip (such as a pololu A4988).

It (optionally) controls pins for drive enable, direction and microstep level, and sends
timed pulses to the step pin.

The step pin can be driven direct from the software (slowstep), which enables pulse by pulse control
of the timing - for example where some sort of feedback / PID is required. The timing of individual
pulses is subject to process / thread scheduling, so pulses can be delayed (typically by up to 
1 or 2 milliseconds on a multicore machine, more on a single core machine), but longer delays can occur.

The step pin can also be driven by pre-prepared memory blocks driving the gpio pins via DMA (faststep).
This provides highly accurate timings (accurate to 1 microsecond). The software prepares these memory blocks
on a JIT basis, but this does mean that feedback control (for example) will have a delay before 'reaching'
the motor. The delay depends on how fast the motor is going and how many motors are being run in parallel.

Both fast and slow modes provide functionality to ramp the motor speedup and down, both to allow faster
speeds and to avoid sudden loading on the motor or its load.

Details:
========

Each motor is setup as an instance of the class stepper, and a configuration file is used to 
define the pins used, and specify the characteristics of the motor (such as the maximum stepping
speed, and control over microstepping, direction etc. This class also implements the slow step
functionality. This functionality runs in a separate thread.

The multimotor class provides coordinated access to multiple motors, in particular it coordinates faststep
across multiple motors, merging the step streams of the motors using faststep into a single sequence of DMA
blocks using pigpio waves. (http://abyz.me.uk/rpi/pigpio/python.html#wave_add_new).

"""
import pootlestuff.watchables as wv
from pootlestuff.watchables import loglvls
import threading, logging, time
from enum import Enum
import queue
import pigpio

class pudvals(Enum):
    pud_off   = pigpio.PUD_OFF
    pud_up    = pigpio.PUD_UP
    pud_down  = pigpio.PUD_DOWN

class gpio_out_pin(wv.enumWatch):
    """
    specific extras for a pin used for bit banging output
    """
    def __init__(self, *, name, value, app, **kwargs):
        """
        defines values that will be used to write 1 and 0 to the pin, and optionally a value
        to write when closed down (leaving the pin in output mode)

        name    : the name is used in log messages
        
        app     : an object with a pigpio instance available, and that provides a log function

        value   : a dict with keys:
                
                value       : (optional - default the first entry in vlist (logic low)) otherwise the initial value for the pin (an entry in vlist)
                pinno       : the gpio pin no - can be -1 when there is no pin (typically because the relevant is hardwired and not accessible to this software
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
        
        This passes control to the wave maker, so when the wave is finished resetValue should be called so the gpio
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
        
            pulse1      : (default True) True to pulse logic 1 and revert to 0 else pulse logic 0 and revert to 1
        
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
    manages the microsteplevel on the motor controller
    """
    def __init__(self, app, value, loglvl=loglvls.INFO, **kwargs):
        """
        looks after the gpiopins that define the microstepping level
        
        value       : initial value - dict of setup info - see 'motor.cfg'
        
        Note: if no pins are available to control the microstep level (because they are hard wired or set via dip switches etc.)
              then value['pins'] should be an empty list and value['microsteps'] should have 1 entry typically:
                  value['microsteps']={1:[]}  
        """
        self.name='usteplevel'
        pinset=value['pins']
        pincount=len(pinset)
        self.mspins=[]
        microstepset={int(sk): sv for sk, sv in value['microsteps'].items()}   # convert keys to ints in case json has made them strings
        value['microsteps']=microstepset
        for pv in microstepset.values():
            assert pincount==len(pv)
        for pk in microstepset.keys():
            assert isinstance(pk, int)
        val = value.get('value', [microstepset.keys()][0])
        assert val in microstepset
        self.microstepset=microstepset
        self.maxmicrosteps=max(microstepset.keys())
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
            super().setValue(int(value), agent)
            vset=self.microstepset[self.getValue()]
            vpins=self.mspins
            for msi in range(len(vset)):
                vpins[msi].setValue(vset[msi], agent)
            return True
        else:
            self.log(loglvls.CRITICAL, 'agent {} not known in setting var {}'.format(agent, self.name))
        return False

class stepgen(wv.watchablesmart):
    """
    base class for tick interval generator
    """
    def __init__(self, name, motor, value, wabledefs, **kwargs):
        """
        base class for step interval generators.
        
        name:   it's the name!
        
        motor:  the motor that owns this generator
        
        mode:   defines if this generator should use software to drive the stepping, or DMA (high precision) to drive the stepping
        
        stepclass: class for the generator (just here to stop it being an unexpected parameter
        """
        self.name=name
        self.motor=motor
        self.mode=value['mode']
        wabledefs.append(('usteps',   wv.enumWatch,   2,     False,    {'vlist': motor.usteplevel.vlist})),     # selects the microstep level to use with these settings)
        super().__init__(value=value, wabledefs=wabledefs, **kwargs)

    def log(self, *args, **kwargs):
        self.motor.log(*args, **kwargs)

    def tickgen(self):
        raise NotImplementedError()

class stepcontrolgrad(stepgen):
    """
    This class implements the details of step timing with parameters to control the various speeds and
    ramping up and down.
    
    Use of a separate class allows multiple different sets of timings to be used, as well as different
    methods for generating the timings.
    
    It provides a generator that yields a sequence of step intervals. If no step is required it returns
    None.
    
    this class ramps speed up by applying a scale factor to difference between the target speed and the current
    speed, this results in an asymptotic approach to the target speed. A small fiddle factor is added to the change
    so we reach the target rather then playing tortoise and hare.
    """
    def __init__(self, **kwargs):
        wables=[
            ('minstep',         wv.floatWatch,      .0017,      False),     # minimum step interval (-> fastest speed)
            ('startstep',       wv.floatWatch,      .006,       False),     # initial interval when starting
            ('rampfact',        wv.floatWatch,      1.1,        False),     # change in interval applied each ramptick when ramping
            ('rampintvl',       wv.floatWatch,      .05,        False),     # interval between speed changes in ramp mode
            ('rampticks',       wv.intWatch,        7700,       False),     # number of ticks to stop from max slow speed
            ('ramputicks',      wv.intWatch,        0,          False),     # actual ticks used to ramp up / down
        ]
        super().__init__(wabledefs=wables, **kwargs)

    def tickgen(self):
        """
        generator function for step times.
        
        yields a sequence of 3-tuples until target is reached, at which point it returns None for the time interval.
        This function keeps it's own time so can generate ticks independent of actual time - the caller must sync
        to a clock if required.
        
        each tuple is:
            0 : None if direction pin is unchanged, else the new value for the direction pin
            1 : time in seconds to the next step pin trigger - None if no step required (e.g. at target pos)
            2 : raw position after this step is issued
            3 : repeat count - 0 means no repeats
            4 : if repeat count > 0, this is the interval to use
            5 : ir repeat count > 0, this is the position increment per tick
        """
        isgoto=self.motor.opmode.getValue()=='going'
                        # set up functions to directly access the various info needed - dynamic fetch allows update on the fly
        if isgoto:
            targetposget= self.motor.targetrawpos.getValue      # target position when in goto mode
        else:
            targetdirget=self.motor.target_dir.getValue         # and the target dir when in run mode (-ve for reverse)
        uslevelv = self.motor.usteplevel.getValue()         # current microstep level
        usteps=self.motor.maxstepfactor // uslevelv
        starttickget= self.startstep.getValue               # minimum time for a first tick
        mintickget=self.minstep.getValue                    # minimum tick time once acceleration complete - fastest we can go
        tickget=self.motor.ticktime.getValue                # current target tick time - 0 gives max speed in this mode (clamped to 'minslow')
        ramptget=self.rampticks.getValue                    # number of ticks needed to slow down from max speed (takes too long to calculate on the fly)
        rampintvlget=self.rampintvl.getValue
        rampfactget=self.rampfact.getValue
        currentposv=self.motor.rawposn.getValue()           # get the current position - we assume we are in control of this so don't re-read it
        utickctr=0
        currenttick=None                                    # set stationary
        elapsedtime=0.0
        self.log(loglvls.DEBUG, '%s !!!!!!!!!!!!! asymp tickgen setup complete %d usteps per step' % (self.name, self.motor.maxstepfactor // uslevelv))
        while True:
            dirset=None
            if isgoto:
                movesteps=targetposget()-currentposv
                if abs(movesteps) < usteps:                     # if less than the current step size stop now before we start
                    self.log(loglvls.DEBUG, 'at target pos')
                    yield (None, None, currentposv, 0, 0, 0)
                    currenttick=None 
            else:
                movesteps=None
            if currenttick is None:                             # been stopped - what to do?
                if self.motor.pending:
                    yield (None, None, currentposv, 0, 0, 0)          # show that we've stopped
                    break
                forwards=movesteps > 0 if isgoto else targetdirget()>0
                dirset = 'F' if forwards else 'R'
                tickv=tickget()
                utickctr=0
                starttickv=starttickget()
                if tickv < starttickv:
                    rintvl=rampintvlget()
                    nextramptick=elapsedtime + rintvl
                    currenttick = starttickv
                else:
                    nextramptick=None  # none means no ramping
                    currenttick = tickv
                self.log(loglvls.DEBUG, '%s starts currenttick %5.4f, scaled to: %6.5f, ramptick %s' % (
                            self.name, currenttick, currenttick/uslevelv, ('none' if nextramptick is None else '%5.4f' %rintvl)))
            else:   # we're moving...
                newfwd=movesteps > 0 if isgoto else targetdirget()>0
                                # slow down if new command pending or direction needs to change or goto reaching target or just going to fast
                targettickv=max(tickget(), mintickget())
                if self.motor.pending or forwards != newfwd or currenttick < targettickv or (isgoto and abs(movesteps) < ramptget()): # slow down?
#                    if self.motor.pending:
#                        print('pending -> slowdown')
#                    if forwards != newfwd:
#                        print('change dir => slowdown')
#                    if currenttick < targettickv:
#                        print('near target => slowdown')
                    starttickv=starttickget()
                    if currenttick >= starttickv: # at min speed - why are we here?
                        if isgoto and abs(movesteps) < ramptget():
                            pass # just carry on till we reach target
                        else:
                            self.log(loglvls.DEBUG,'slowing -> stopped')
                            currenttick=None
                    elif nextramptick is None:
                        self.log(loglvls.DEBUG, 'set nextramptick')
                        nextramptick = elapsedtime+rampintvlget()
                    elif elapsedtime < nextramptick:
                        pass
                    else:
                        currenttick += (currenttick-mintickget()*.95)*rampfactget()
                        if currenttick > starttickv:       # have we passed slowest speed?
                            currenttick = starttickv
                            nextramptick=None
                        else:
                            rampintvlv=rampintvlget()
                            tickstillchange=int(rampintvlv/currenttick)-2
                            if tickstillchange > 1:
                                multiticks = tickstillchange-1 if tickstillchange < 20 else 19
                                currentposv += (usteps if forwards else -usteps) * multiticks
                                utickctr += usteps * multiticks
                                elapsedtime += currenttick * multiticks
                                yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)
                            nextramptick += rampintvlv
                        self.log(loglvls.DEBUG, 'slowing.... %5.4f' % currenttick)
                elif currenttick > targettickv: # speeding up?
                        if nextramptick is None or elapsedtime > nextramptick:   # time for a change?
                            currenttick -= (currenttick-mintickget()*.95)*rampfactget()
                            if currenttick < targettickv:       # too fast?
                                currenttick = targettickv
                                nextramptick=None
                                self.ramputicks.setValue(utickctr, wv.myagents.app)
                            else:
                                rampintvlv=rampintvlget()
                                if nextramptick is None:
                                    nextramptick = elapsedtime+rampintvlv
                                else:
                                    nextramptick += rampintvlv
                                tickstillchange=int(rampintvlv/currenttick)-2
                                if tickstillchange > 1:
                                    multiticks = tickstillchange-1 if tickstillchange < 20 else 19
                                    currentposv += (usteps if forwards else -usteps) * multiticks
                                    utickctr += usteps * multiticks
                                    elapsedtime += currenttick * multiticks
                                    yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)

                else: # constant speed
                    multiticks = 25
                    currentposv += (usteps if forwards else -usteps) * multiticks
                    utickctr += usteps * multiticks
                    elapsedtime += currenttick * multiticks
                    yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)
                    nextramptick=None
            currentposv += usteps if forwards else -usteps
            utickctr += usteps
            yv=.1 if currenttick is None else currenttick/uslevelv
            elapsedtime += yv
            yield (dirset, yv, currentposv, 0, 0, 0)

class stepcontrols(stepgen):
    """
    This class implements the details of step timing with parameters to control the various speeds and
    ramping up and down.
    
    Use of a separate class allows multiple different sets of timings to be used, as well as different
    methods for generating the timings.
    
    It provides a generator that yields a sequence of step intervals. If no step is required it returns
    None.
    
    This class ramps up and down by varying the interval by a constant factor at regular intervals.
    """
    def __init__(self, **kwargs):
        wables=[
            ('minstep',         wv.floatWatch,      .0017,      False),     # minimum step interval (-> fastest speed)
            ('startstep',       wv.floatWatch,      .006,       False),     # initial interval when starting
            ('rampfact',        wv.floatWatch,      1.1,        False),     # change in interval applied each ramptick when ramping
            ('rampintvl',       wv.floatWatch,      .05,        False),     # interval between speed changes in ramp mode
            ('rampticks',       wv.intWatch,        7700,       False),     # number of ticks to stop from max slow speed
            ('ramputicks',      wv.intWatch,        0,          False),     # actual ticks used to ramp up / down
        ]
        super().__init__(wabledefs=wables, **kwargs)

    def tickgen(self):
        """
        generator function for step times.
        
        yields a sequence of 3-tuples until target is reached, at which point it returns None for the time interval.
        This function keeps it's own time so can generate ticks independent of actual time - the caller must sync
        to a clock if required.
        
        each tuple is:
            0 : None if direction pin is unchanged, else the new value for the direction pin
            1 : time in seconds to the next step pin trigger - None if no step required (e.g. at target pos)
            2 : raw position after this step is issued
            3 : repeat count - 0 means no repeats
            4 : if repeat count > 0, this is the interval to use
            5 : ir repeat count > 0, this is the position increment per tick
        """
        isgoto=self.motor.opmode.getValue()=='going'
                        # set up functions to directly access the various info needed - dynamic fetch allows update on the fly
        if isgoto:
            targetposget= self.motor.targetrawpos.getValue      # target position when in goto mode
        else:
            targetdirget=self.motor.target_dir.getValue         # and the target dir when in run mode (-ve for reverse)
        uslevelv = self.motor.usteplevel.getValue()         # current microstep level
        usteps=self.motor.maxstepfactor // uslevelv
        starttickget= self.startstep.getValue               # minimum time for a first tick
        mintickget=self.minstep.getValue                    # minimum tick time once acceleration complete - fastest we can go
        tickget=self.motor.ticktime.getValue                # current target tick time - 0 gives max speed in this mode (clamped to 'minslow')
        ramptget=self.rampticks.getValue                    # number of ticks needed to slow down from max speed (takes too long to calculate on the fly)
        rampintvlget=self.rampintvl.getValue
        rampfactget=self.rampfact.getValue
        currentposv=self.motor.rawposn.getValue()           # get the current position - we assume we are in control of this so don't re-read it
        utickctr=0
        currenttick=None                                    # set stationary
        elapsedtime=0.0
        self.log(loglvls.DEBUG, '%s !!!!!!!!!!!!! asymp tickgen setup complete %d usteps per step' % (self.name, self.motor.maxstepfactor // uslevelv))
        while True:
            dirset=None
            if isgoto:
                movesteps=targetposget()-currentposv
                if abs(movesteps) < usteps:                     # if less than the current step size stop now before we start
                    self.log(loglvls.DEBUG, 'at target pos')
                    yield (None, None, currentposv, 0, 0, 0)
                    currenttick=None 
            else:
                movesteps=None
            if currenttick is None:                             # been stopped - what to do?
                if self.motor.pending:
                    yield (None, None, currentposv, 0, 0, 0)          # show that we've stopped
                    break
                forwards=movesteps > 0 if isgoto else targetdirget()>0
                dirset = 'F' if forwards else 'R'
                tickv=tickget()
                utickctr=0
                starttickv=starttickget()
                if tickv < starttickv:
                    rintvl=rampintvlget()
                    nextramptick=elapsedtime + rintvl
                    currenttick = starttickv
                else:
                    nextramptick=None  # none means no ramping
                    currenttick = tickv
                self.log(loglvls.DEBUG, '%s starts currenttick %5.4f, scaled to: %6.5f, ramptick %s' % (
                            self.name, currenttick, currenttick/uslevelv, ('none' if nextramptick is None else '%5.4f' %rintvl)))
            else:   # we're moving...
                newfwd=movesteps > 0 if isgoto else targetdirget()>0
                                # slow down if new command pending or direction needs to change or goto reaching target or just going to fast
                targettickv=max(tickget(), mintickget())
                if self.motor.pending or forwards != newfwd or currenttick < targettickv or (isgoto and abs(movesteps) < ramptget()): # slow down?
#                    if self.motor.pending:
#                        print('pending -> slowdown')
#                    if forwards != newfwd:
#                        print('change dir => slowdown')
#                    if currenttick < targettickv:
#                        print('near target => slowdown')
                    starttickv=starttickget()
                    if currenttick >= starttickv: # at min speed - why are we here?
                        if isgoto and abs(movesteps) < ramptget():
                            pass # just carry on till we reach target
                        else:
                            self.log(loglvls.DEBUG,'slowing -> stopped')
                            currenttick=None
                    elif nextramptick is None:
                        self.log(loglvls.DEBUG, 'set nextramptick')
                        nextramptick = elapsedtime+rampintvlget()
                    elif elapsedtime < nextramptick:
                        pass
                    else:
                        currenttick+=currenttick*rampfactget() # slow down a bit
                        if currenttick > starttickv:       # have we passed slowest speed?
                            currenttick = starttickv
                            nextramptick=None
                        else:
                            rampintvlv=rampintvlget()
                            tickstillchange=int(rampintvlv/currenttick)-2
                            if tickstillchange > 1:
                                multiticks = tickstillchange-1 if tickstillchange < 20 else 19
                                currentposv += (usteps if forwards else -usteps) * multiticks
                                utickctr += usteps * multiticks
                                elapsedtime += currenttick * multiticks
                                yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)
                            nextramptick += rampintvlv
                        self.log(loglvls.DEBUG, 'slowing.... %5.4f' % currenttick)
                elif currenttick > targettickv: # speeding up?
                        if nextramptick is None or elapsedtime > nextramptick:   # time for a change?
                            currenttick -= currenttick*rampfactget() # speed up a bit
                            if currenttick < targettickv:       # too fast?
                                currenttick = targettickv
                                nextramptick=None
                                self.ramputicks.setValue(utickctr, wv.myagents.app)
                            else:
                                rampintvlv=rampintvlget()
                                if nextramptick is None:
                                    nextramptick = elapsedtime+rampintvlv
                                else:
                                    nextramptick += rampintvlv
                                tickstillchange=int(rampintvlv/currenttick)-2
                                if tickstillchange > 1:
                                    multiticks = tickstillchange-1 if tickstillchange < 20 else 19
                                    currentposv += (usteps if forwards else -usteps) * multiticks
                                    utickctr += usteps * multiticks
                                    elapsedtime += currenttick * multiticks
                                    yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)

                else: # constant speed
                    multiticks = 25
                    currentposv += (usteps if forwards else -usteps) * multiticks
                    utickctr += usteps * multiticks
                    elapsedtime += currenttick * multiticks
                    yield (dirset, yv, currentposv, multiticks-1, currenttick, usteps)
                    nextramptick=None
            currentposv += usteps if forwards else -usteps
            utickctr += usteps
            yv=.1 if currenttick is None else currenttick/uslevelv
            elapsedtime += yv
            yield (dirset, yv, currentposv, 0, 0, 0)

class stepper(wv.watchablepigpio):
    """
    Drives a single stepper motor with 2 different styles of driving. Uses an A4988 style stepper controller.

    It supports:
        direct stepping (for slower speeds and with the potential for dynamic feedback control)
        
        fast stepping using dma control blocks to provide very accurate timing with ramp up and down
        capability

    The motor accepts a small number of commands:
    --------------------------------------------- 
       
    close:  close the motor down - it cannot be used again once closed
    
    stop:   stop the motor - if it is running in will stop in a controlled manner (i.e. it will decelerate to s
    
    goto:   the motor will travel to the target position in the op mode in 'nextopmode'
    
    run:    the motor will run at the target speed until stop is requested

    The motor can be in 1 of a number of (operation) modes. These are set by the motor and show the current state.
    -------------------------------------------------------
    
    closed: Motor has shut down and cannot be used - the motor can only be run again by re-creating the object
    
    stopped:The motor is not currently active.
    
    going:  The motor is moving to the target location - it will accelerate to the max applicable speed
            and decelerate to the location
    
    moving: The motor is running at the target speed - this will use accelerate and decelerate as appropriate
    
    stepmodes provide detailed control of initial speed, maximum speed, acceleration, deceleration (and how they are
    actioned) as well as whether the stepping will by directly from software or using DMA controlled stepping.
    
    The class runs the motor control in its own thread, commands are passed to that thread to process
    
    rawposn is an integer representation of the position in microsteps at the highest microstep level.
    """

    opmodes= ('closed', 'stopped', 'going', 'moving')

    commands=('none', 'close', 'stop', 'goto', 'run')

    def __init__(self, name, app, value, wabledefs=[], **kwargs):
        """
        sets up a stepper motor driver for an A4988 style chip.
        
        name    : used in log messages, and to identify motor in wave processing
        
        app     : multi motor controller
        
        value   : dict with saved values for watchables
        
        wabledefs: extra watchables to be included in the object 
        
        settings must be included in kwargs to define pins and fixed values and limits for the motor.
        see the example file 'motor.cfg'

        kwargs  : other args passed to watchable.
        """
        self.name=name
        self.mthread=None
        self.cmndq=queue.Queue()
        self.respq=queue.Queue()
        self.overrunctr=0
        self.overruntime=0.0
        self.tlogs={}
        modewables=[]
        rmodes=[]
        for smode, svals in value['stepmodes'].items():         # find the stepmodes declared in values (originally from json file) and make fields defs
            modewables.append((smode,   app.classdefs[svals['stepclass']], None, False, {'name': smode, 'motor': self}))
            rmodes.append(smode)
        self.stepmodenames=rmodes
        wables=[
            ('userstepm',   wv.enumWatch,   rmodes[0],          False,  {'vlist': rmodes}),         # user step def for goto and run
            ('targetrawpos',wv.intWatch,    0,                  False),                             # target position when in goto mode
            ('target_dir',  wv.intWatch,    1,                  False),                             # target direction when in run mode - +ve for fwd, -ve for reverse
            ('opmode',      wv.enumWatch,   self.opmodes[1],    False,  {'vlist': self.opmodes}),   # what mode is current - set by the motor
            ('stepstyle',   wv.enumWatch,   'off',              False,  {'vlist': ('off', 'soft', 'dma')}), # shows if motor is stepping and if so which type
            ('drive_enable',gpio_out_pin,   None,               False,  {'name': 'drive enable', 'loglevel': wv.loglvls.DEBUG}),  # sets up drive enable pin
            ('direction',   gpio_out_pin,   None,               False,  {'name': 'direction'}),     # sets up direction pin
            ('step',        gpio_trigger_pin,None,              False,  {'name': 'step'}),          # sets up step pin
            ('holdstopped', wv.floatWatch,  .5,                 False),     # when motor stops, drive_enable goes false after this time (in seconds), 0 means drive always enabled
            ('usteplevel',  usteplevel_pinset,None,             False),     # sets up the pins used to control microstep level
            ('rawposn',     wv.intWatch,    0,                  False),     # current position in microsteps (not totally up to date while fast stepping)
            ('ticktime',    wv.floatWatch,  0,                  False),     # minimum step interval (clamped to minslow for slow stepping)
            ('stepmodes',   wv.watchablesmart,None,             False,  {'wabledefs': modewables}), # the available stepper control modes
        ]+wabledefs
        self.pinsetuplist=('drive_enable','direction','step','usteplevel')
        super().__init__(wabledefs=wables, app=app, value=value, **kwargs)
        self.pending=None
        self.dothis('stop')
        self.mthread=threading.Thread(name='stepper'+self.name, target=self._thrunner)
        self.log(loglvls.INFO,'starting motor %s thread stepper' % self.name)
        self.maxstepfactor = self.usteplevel.maxusteps()
        self.mthread.start()

    def crashstop(self):
        """
        immediate stop eventually
        """
        self.drive_enable.setValue('disable',wv.myagents.app)
        self.mode.setValue('closed',wv.myagents.user)
        
    def cleanstop(self):
        self.dothis(command='close')

    def dothis(self, command, targetpos=None, targetdir=None, stepmode=None):
        assert command in self.commands
        if command is ('goto', 'run'):
            assert self.opmode.getValue() == 'stopped'
            assert stepmode in self.userstepm.vlist
            if command == 'run':
                assert targetdir in ('fwd','rev')
            if command == 'goto':
                assert isinstance(targetpos, (int,float))
        if command != 'none':
            print('putting', command)
            self.cmndq.put_nowait((self._command, {'command': command, 'targetpos': targetpos, 'targetdir': targetdir, 'stepmode': stepmode}))
            if command in ('goto', 'run'):
                mo = getattr(self.stepmodes,stepmode).mode
                print('command %s return mode %s' % (command, mo))
                return mo
            else:
                print('command %s return nothing' % command)            
        return None

    def _command(self, command, targetpos, targetdir, stepmode):
        print('setting pending======================', command)
        self.pending={'command': command, 'targetpos': targetpos, 'targetdir': targetdir, 'stepmode': stepmode}

    def _thrunner(self):
        self.log(loglvls.INFO,'%s motor thread running', self.name)
        self.opmode.setIndex(1, wv.myagents.app)
        self.stepstyle.setIndex(0, wv.myagents.app)
        abs_start=time.time()
        abs_thread=time.thread_time()
        abs_process=time.process_time()
        while self.opmode.getIndex() != 0:  # mode zero means time to exit thread
            cmd='stop' if self.pending is None else self.pending['command']
            if cmd=='close':
                self.opmode.setIndex(0, wv.myagents.app)
            elif cmd=='stop':
                self.pending=None
                self._thrun_stopped()
            else:
                print('_________________', self.pending)
                stepsettings=getattr(self.stepmodes,self.pending['stepmode'])
                self.opmode.setValue('going' if cmd=='goto' else 'moving', wv.myagents.app)
                comdetails=self.pending
                self.pending=None
                if stepsettings.mode=='wave':
                    self._thrun_faststep()
                elif stepsettings.mode=='software':
                    self.stepstyle.setValue('soft', wv.myagents.app)
                    self._slowrun(stepsettings, command=comdetails['command'], targetpos=comdetails['targetpos'], targetdir=comdetails['targetdir'])
                else:
                    self.log(loglvls.ERROR,'%s unknown motor mode %s' % (self.name, stepsettings.mode))
                self.opmode.setValue('stopped', wv.myagents.app)
        self._thcloseIO()
        self.log(loglvls.INFO,'%s exiting motor thread. Execution summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (self.name,
                    time.time()-abs_start, time.process_time()-abs_process, time.thread_time()-abs_thread))
        return

    def _thwaitq(self, waittill=None, delay=None):
        """
        uses delay if set, otherwise calculates required delay.

        waits on the command q till for at most the required delay
        
        if nothing arrived on the q it returns True
        
        if anything returned on the q and has been processed it returns False (to signify the target time has not been reached, but
        something has changed), this allows loops to react quickly to changed settings
        """
        if delay is None:
            delay=waittill-time.time()
        try:
            if delay <= 0:
                self.overrunctr+=1
                self.overruntime-=delay
                method, kwargs = self.cmndq.get_nowait()
            else:
                method, kwargs = self.cmndq.get(timeout=delay)
        except:
            method=None
        if method is None:
            return True
        print(kwargs)
        method(**kwargs)
        return False

    def _thrun_stopped(self):
        """
        this function runs in the motor thread while the mode is 'stopped'.
        
        if holdstopped is zero it sets drive_enable (we arrive here on first setup), otherwise it waits for holdstopped time then 
        disables the drive
        """
        if self.holdstopped.getValue() == 0:
            self.drive_enable.setValue('enable',wv.myagents.app)
            holdtimeout=None
        else:
            holdtimeout=time.time() + self.holdstopped.getValue()
        self.log(loglvls.INFO, "%s entering mode 'stopped'" %self.name)
        while self.pending==None:
            if holdtimeout:
                self._thwaitq(waittill=holdtimeout)
            else:
                self._thwaitq(delay=1)
            if holdtimeout and holdtimeout < time.time():
                self.drive_enable.setValue('disable',wv.myagents.app)
                holdtimeout=None
        self.log(loglvls.INFO, "%s leaving mode 'stopped'" % self.name)

    def _thrun_faststep(self):
        """
        with runfast, the real work is done in the motorset class, so this just idles until the mode changes again
        """
        self.log(loglvls.INFO, "%s entering mode 'faststep'" % self.name)
        while self.pending==None:
            self._thwaitq(delay=1)
        self.log(loglvls.INFO, "%s leaving mode 'faststep'" % self.name)

    def pulsegen(self, stepdef, command, targetpos, targetdir):
        """
        a wrapper for tickgen that yields the parameters for creating a pigpio pulse plus the new absolute position after each pulse.
        
        thus each yield returns a 5 tuple:
            0: mask of bits to turn on
            1: mask of bits to turn off
            2: microsecond time of this pulse or None if the motor is now stationary
            3: raw position of this motor after this pulse
            4: the name of this motor
        """
        self.opmode.setValue({'goto': 'going', 'run': 'moving'}[command], wv.myagents.app)
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        dirvar=self.direction
        setdir={'F':dirvar.getBits('F'),
                'R':dirvar.getBits('R')}
        dirbit=1<<dirvar.pinno
        if dirvar.vlist.index('F') == 1:
            xsetdir={'F': (dirbit,0),
                    'R': (0, dirbit)}
        else:
            xsetdir={'F': (0, dirbit),
                    'R': (dirbit,0)}
        assert xsetdir['F']==setdir['F'], 'oops %s - %s' % (xsetdir['F'], setdir['F'])
        assert xsetdir['R']==setdir['R']
        self.usteplevel.setValue(stepdef.usteps.getValue(), wv.myagents.app)
#        self.drive_enable.setValue('enable', wv.myagents.app)
        self.tickgenactive=True
        tickmaker=stepdef.tickgen()
        stepvar=self.step
        pulselen=stepvar.pulsetime
        stepa=1<<stepvar.pinno if stepvar.vlist[0] == 0 else 0
        stepb=1<<stepvar.pinno if stepa is 0 else 0
        onbits, offbits = stepa, stepb
        dirmask=1<<dirvar.pinno
        usclock=0
        overflow=0.0
        pulseoff=[offbits, onbits, 0, 0, self.name]
        pulseon=[onbits, offbits, 0, 0, self.name]
        repeat=0
        startup=True
        while True:
            if repeat > 0:
                repeat -=1
                newpos += poschange
            else:
                try:
                    dirchange, ticktime, newpos, repeat, tickintvl, poschange = next(tickmaker)
                except StopIteration:
                    break
            if ticktime is None:
                delay, overflow = (None, 0)
            else:
                delay, overflow=divmod(ticktime*1000000+overflow,1)
            if dirchange:
                dirbits=setdir[dirchange]
                if startup:
                    driveenbits=self.drive_enable.getBits('enable')
                    dirbits = (dirbits[0] | driveenbits[0], dirbits[1] | driveenbits[1])
                    startup=False
                print('setting dir', dirchange, dirbits)
                yield (onbits | dirbits[0], offbits | dirbits[1], usclock, newpos, self.name)
            else:
                pulseon[2]=usclock
                pulseon[3]=newpos
                yield pulseon
            usclock += pulselen
            pulseoff[2]=usclock
            pulseoff[3]=newpos
            yield pulseoff
            if delay is None:
                break
            else:
                usclock += int(delay)-pulselen
        self.drive_enable.setValue('disable', wv.myagents.app)

    def _slowrun(self, stepinf, command, targetpos, targetdir):
        """
        runs a simple loop sending trigger pulses to the chip's step pin from code.
        
        It gets the step interval from the tick generator, which monitors both the target mode of the motor and
        the target position to manage ramping, it uses _thwaitq to delay for the required time, _thwaitq also
        picks up incoming commands and processes them.
        """
        self.opmode.setValue({'goto': 'going', 'run': 'moving'}[command], wv.myagents.app)
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        self.usteplevel.setValue(stepinf.usteps.getValue(), wv.myagents.app)
        self.drive_enable.setValue('enable', wv.myagents.app)
        self.log(loglvls.INFO, '%s _slowrun starts' % self.name)
        self.tickgenactive=True
        realstep=True
        self.overrunctr=0
        self.overruntime=0.0
        tickmaker=stepinf.tickgen()
        steptrig=self.step.trigger
        directionset=self.direction.setValue
        posnset = self.rawposn.setValue
        self.starttimelog('slowrun')
        nextsteptime=time.time()
        posupdatetime=nextsteptime+.8
        tickctr=0
        newpos=self.rawposn.getValue()
        stoppedtimer=None
#        tlf=open('tlog.txt','w')
        tstart=time.time()
        repeat=0
        while True:
            if realstep:
                if repeat > 0:
                    repeat -=1
                    newpos += poschange
                else:
                    try:
                        dirchange, ticktime, newpos, repeat, tickintvl, poschange = next(tickmaker)
                    except StopIteration:
                        self.log(loglvls.DEBUG,'StopIteration!!!!!!!')
                        break
                if dirchange:
                    print('setting direction to', dirchange)
                    directionset(dirchange, wv.myagents.app)
                if ticktime is None:
#                    tlf.write('skip at %7.5f\n' % (time.time()-tstart))
                    mode=self.opmode.getIndex()
                    if mode==2:  # its a goto - exit
                        break
                    nextsteptime += .05
                    if stoppedtimer is None:
                        holdtime=self.holdstopped.getValue()
                        if holdtime > 0:
                            stoppedtimer=time.time()+holdtime
                    else:
                        if time.time() > stoppedtimer:
                            self.drive_enable.setValue('disable', wv.myagents.app)
                            stoppedtimer += 1000
                            self.log(loglvls.DEBUG, "{} has turned off drive current.".format(self.name))
                            break
                else:
                    if not stoppedtimer is None:
                        self.drive_enable.setValue('enable', wv.myagents.app)
                    steptrig()
#                    tlf.write('step at %7.5f\n' % (time.time()-tstart))
                    stoppedtimer=None
                    tickctr+=1
                    nextsteptime += ticktime
                if time.time() > posupdatetime:
                    posnset(newpos, wv.myagents.app)
                    posupdatetime += .8
            realstep=self._thwaitq(nextsteptime)
#        tlf.close()
        self.log(loglvls.INFO, "%s leaving mode 'slowxxx', now at %s, %d overruns of %d ticks, total overrun: %7.3f." % (self.name, newpos, self.overrunctr, tickctr, self.overruntime))
        if not newpos is None:
             posnset(newpos, wv.myagents.app)
        self.log(loglvls.INFO, self.reporttimelog('slowrun'))

    def _thcloseIO(self):
        for pinname in self.pinsetuplist:
            getattr(self,pinname).shutdown()

    def starttimelog(self, logname):
        withlog={}
        self.tlogs[logname]=withlog
        withlog['startclock'] = time.perf_counter_ns()
        withlog['startthread']= time.thread_time_ns()

    def reporttimelog(self, logname):
        if logname in self.tlogs:
            started=self.tlogs[logname]
            return 'elapsed: {clkt:7.3f}, thread: {thrt:7.3f}'.format(
                    clkt=(time.perf_counter_ns()-started['startclock'])/1000000000,
                    thrt=(time.thread_time_ns()-started['startthread'])/1000000000)
        else:
           return None

    def fastgoto(self, stepmode, **kwargs):
        """
        prepare to move or goto position
        
        Check if this motor mode uses waves and if so return a generator
        The main work is done in the controller.   
        """
        stepgen=getattr(self.stepmodes,stepmode)
        mtype=stepgen.mode
        if mtype=='wave':
            return self.pulsegen(stepgen, **kwargs)
        return None

controllermodes=('closed', 'off', 'faststep')

class multimotor(wv.watchablepigpio):
    def __init__(self, gpiolog=False, loglvl=loglvls.INFO, **kwargs):
        wables=[
            ('pigpmspw',    wv.intWatch,    0,                  False),
            ('pigpppw',     wv.intWatch,    0,                  False),
            ('pigpbpw',     wv.intWatch,    0,                  False),
            ('mode',        wv.enumWatch,   controllermodes[1], False,  {'vlist': controllermodes}),
            ('gotonow',     wv.btnWatch,    'goto now',         False),
            ('wavepulses',  wv.intWatch,    1000,               True,   {'minv':100}),
        ]
        if gpiolog:
            from pigpiolog import plog
            pio=plog()
        else:
            pio=None
        super().__init__(wabledefs=wables, pigp=pio, loglevel=loglvl, **kwargs)
        self.pigpmspw.setValue(self.pio.wave_get_max_micros(), wv.myagents.app)
        self.pigpppw.setValue(self.pio.wave_get_max_pulses(), wv.myagents.app)
        self.pigpbpw.setValue(self.pio.wave_get_max_cbs(), wv.myagents.app)
        self.motors={}
        for motname, motinfo in self.startsettings.items():
            newmotor=self.makeChild(defn=(motname, self.classdefs[motinfo['motorclass']], None, False, {'name': motname, 'loglevel': wv.loglvls.DEBUG}), value=self.startsettings.get(motname,{}))
            if newmotor is None:
                raise ValueError('motor construction failed for %s' % motname)
            self.motors[motname]=newmotor
        self.cmndq=queue.Queue()
        self.cthread=threading.Thread(name='controller(%s)' % type(self).__name__, target=self._thrunner)
        self.cthread.start()

    def close(self):
        self.cleanstop()

    def cleanstop(self):
        for th in threading.enumerate():
            print('thread' + th.name)
        for motor in self.motors.values():
            motor.cleanstop()
        self.mode.setValue('closed', wv.myagents.app)

    def runfast(self, motorlist):
        self.cmndq.put((self._threadfaststep, {'motorlist': motorlist}))

    def _thrunner(self):
        self.log(loglvls.INFO,'multimotor control thread running' )
        abs_start=time.time()
        abs_thread=time.thread_time()
        abs_process=time.process_time()
        curmode=self.mode.getValue()
        while curmode != 'closed':
            self._thwaitq(time.time()+3)
            curmode=self.mode.getValue()
        self.log(loglvls.INFO,'exiting control. Summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (
                    time.time()-abs_start, time.process_time()-abs_process, time.thread_time()-abs_thread))

    def _thwaitq(self, waittill):
        """
        waits on the command q till at most waittill.
        
        if nothing arrived on the q it returns True
        
        if anything returned on the q and has been processed it returns False (to signify the target time has not been reached)
        """
        delay=waittill-time.time()
        try:
            if delay <= 0:
                self.overrunctr+=1
                self.overruntime-=delay
                method, kwargs = self.cmndq.get_nowait()
            else:
                method, kwargs = self.cmndq.get(timeout=delay if delay <=1 else 1)
        except:
            method=None
        if method is None:
            return True
        method(**kwargs)
        return delay <= 0

    def _threadfaststep(self, motorlist):
        """
        fast step request - get the motors involved and .....
        motorposns: a dict with a key for each motor to use, and the value is the target position for that motor
        """
        self.log(loglvls.INFO, "starting mode 'fast")
        timestart=time.time()
        mgens=[]
        motors={}
        for motor, moveparams in motorlist:
            tgen=motor.fastgoto(**moveparams)
            if not tgen is None:
                mgens.append((motor.name, tgen))
                motor.opmode.setValue({'goto': 'going', 'run': 'moving'}[moveparams['command']], wv.myagents.app)
                motors[motor.name]=motor

        mstime=0
        if len(mgens) > 0:
            mergegen=self.pulsemerge(mgens)
            try:
                thisp=next(mergegen)
            except StopIteration:
                thisp=None

            self.pio.wave_clear()
            moredata=True
            pendingbufs=[]
            buffends=[]
            logf=open('tnewmllog.txt','w')
#            logf=None
            while moredata:
                while len(pendingbufs) < 3 and moredata:
                    nextbuff=[]
                    mposns={}
                    while len(nextbuff) < self.wavepulses.getValue():
                        try:
                            nextp=next(mergegen)
                        except StopIteration:
                            nextp=None
                        if nextp is None:
                            nextbuff.append(pigpio.pulse(thisp[0], thisp[1], 1))
                            moredata=False
                            break
                        else:
                            dtime=nextp[2]-thisp[2]
                            nextbuff.append(pigpio.pulse(thisp[0], thisp[1], dtime))
                            if not logf is None and dtime != 2:
                                logf.write('%5d  %x   %x\n' % (dtime,thisp[0],thisp[1]))
                        thisp=nextp
                        mposns[thisp[4]]=thisp[3]
                    if len(nextbuff) > 0:
                        pcount=self.pio.wave_add_generic(nextbuff)
                        waveid=self.pio.wave_create()
                        pendingbufs.append(waveid)
                        buffends.append(mposns)
                        self.log(loglvls.DEBUG,'wave %d, duration %7.5f, size %d, cbs: %d' % (waveid, self.pio.wave_get_micros()/1000000, len(nextbuff), self.pio.wave_get_cbs()))
                        self.pio.wave_send_using_mode(waveid, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
                        if timestart:
                            self.log(loglvls.DEBUG,'startup time:::::::::::::: %6.3f' % (time.time()-timestart))
                            timestart=None

                if len(pendingbufs) > 0:
                    self._thwaitq(time.time()+.04)
                    current=self.pio.wave_tx_at()
                    if current == 9999 or pendingbufs.index(current) != 0:
                        donebuf = pendingbufs.pop(0)
                        try:
                            self.pio.wave_delete(donebuf)
                            self.log(loglvls.DEBUG,'wave %d complete, remains: %s' % (donebuf, pendingbufs))
                        except pigpio.error:                            
                            self.log(loglvls.DEBUG,'wave delete failed for wave %d with %s' % (donebuf, pendingbufs))
                        endposns = buffends.pop(0)
                        for mn, mp in endposns.items():
                            self.motors[mn].rawposn.setValue(mp, wv.myagents.app)
                    elif current == pendingbufs[0]:
                        pass
                    else:
                        self.log(loglvls.DEBUG,'AAAAAAAAAAAAAAAAAAAAAAAAAAAAArg')

            self.log(loglvls.DEBUG, 'final waves %d' % len(pendingbufs))
            while len(pendingbufs) > 0:
                self._thwaitq(time.time()+.2)
                current=self.pio.wave_tx_at()
                if current == 9999 or pendingbufs.index(current) != 0:
                    donebuf = pendingbufs.pop(0)
                    self.log(loglvls.DEBUG,'wave %d complete' % donebuf )
                    self.pio.wave_delete(donebuf)
                    endposns = buffends.pop(0)
                    for mn, mp in endposns.items():
                        self.motors[mn].rawposn.setValue(mp, wv.myagents.app)
                elif current ==pendingbufs[0]:
                    pass
#                    self.log(loglvls.DEBUG,'wave %d running' % current)
                else:
                    self.log(loglvls.DEBUG,'BBBBBBBBBBBBBBBBBBBBBBBBBBAArg')
#            self.pigp.wave_clear()
            if not logf is None:
                logf.close()
        self.log(loglvls.INFO, "leaving mode 'fast")
        for motor in motors.values():
            motor.opmode.setIndex(1, wv.myagents.app)
        self.mode.setValue('off', wv.myagents.app)
#            self.log(logging.INFO, self.reporttimelog('fastrun'))

    def pulsemerge(self, mgens):
        """
        produces an ordered sequence of pulse requests from all sources
        """
        mpulses=[]
        newgens=[]
        for ix, gen in enumerate(mgens):
            try:
                mpulses.append(next(gen[1]))
                newgens.append(gen)
            except StopIteration:
                pass
        if len(mpulses) == 0:
            pass
        elif len(mpulses) == 1:
            while True:
                yield mpulses[0]
                try:
                    mpulses[0]=next(newgens[0][1])
                except StopIteration:
                    break
        elif len(mpulses)==2:
            genA=newgens[0]
            genB=newgens[1]
            pulseA=mpulses[0]
            pulseB=mpulses[1]
            while True:
                if pulseA is None:
                    if pulseB is None:
                        break
                    else:
                        yield pulseB
                        try:
                            pulseB=next(genA[1])
                        except StopIteration:
                            pulseB = None
                else:
                    if pulseB is None or pulseA[2] < pulseB[2]:
                        yield pulseA
                        try:
                            pulseA = next(genA[1])
                        except StopIteration:
                            pulseA = None
                    else:
                         yield pulseB
                         try:
                             pulseB = next(genB[1])
                         except StopIteration:
                             pulseB = None
        else:
            while True:
                nextix=None
                for ix, npulse in enumerate(mpulses):
                    if not npulse is None:
                        if nextix is None:
                            nextix=ix
                            nextt=npulse[2]
                        elif npulse[2] < nextt:
                            nextix=ix
                            nextt=npulse[2]
                        elif npulse[2] == nextt:
                            if isinstance(nextix, int):
                                nextix=[nextix, ix]
                            else:
                                nextix.append(ix)
                if nextix is None:
                    break
                if isinstance(nextix, int):
                    yield mpulses[nextix]
                    try:
                        mpulses[nextix] = next(newgens[nextix][1])
                    except StopIteration:
                        mpulses[nextix] = None
                else:
                    for nx in nextix:
                        yield mpulses[nx]
                        try:
                            mpulses[nx] = next(newgens[nx][1])
                        except StopIteration:
                            mpulses[nx] = None
