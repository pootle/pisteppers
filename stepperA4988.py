#!/usr/bin/python3

"""
A module to drive stepper motors via a basic driver chip like the A4988.

This (optionally) controls pins for drive enable, direction and microstep
level, and sends timed pulses to the step pin.

The step pin can be driven direct from the software (slowstep), which enables pulse by pulse control
of the timing - for example where some sort of feedback / PID is required. The timing of individual
pulses is subject to process / thread scheduling, so pulses can be delayed (typically by up to 
1 or 2 milliseconds on a multicore machine, more on a single core machine), but longer delays can occur.

The step pin can also be driven by pre-prepared memory blocks driving the gpio pins via DMA (faststep).
This provides highly accurate timings (accurate to 1 microsecond). The software prepares these memory blocks
on a JIT basis, but this does mean that feedback control (for example) will have a delay before 'reaching'
the motor. The delay depends on how fast the motor is going and how many motors are being run in parallel.

"""
from stepperbase import basestepper
import pootlestuff.watchables as wv
import gpiopins as gpp

class A4988stepper(basestepper):
    """
    Extends basestepper for A4988 / DRV8825 style stepper driver chips with signals controlled directly by gpio pins.
    """
    def __init__(self, wabledefs, **kwargs):
        """
        Extends basestepper with specifics to drive step controller chips like A4988 and DRV8825
        """
        wables=wabledefs+[
            ('drive_enable',gpp.gpio_out_pin,None,              False,  {'name': 'drive enable', 'loglevel': wv.loglvls.DEBUG}),  # sets up drive enable pin
            ('direction',   gpp.gpio_out_pin,None,              False,  {'name': 'direction'}),     # sets up direction pin
            ('step',        gpp.gpio_trigger_pin,None,          False,  {'name': 'step'}),          # sets up step pin
            ('usteppins',   gpp.usteplevel_pinset,None,         False),                             # sets up the pins used to control microstep level
        ]
        self.pinsetuplist=('drive_enable','direction','step','usteppins')
        super().__init__(wabledefs=wables, **kwargs)

    def getmaxusteplevel(self):
        return self.usteppins.maxusteps()

    def getusteplevel(self, usteplevelname):
        """
        returns number of (micro) steps per full step for the named level
        """
        return self.usteppins.microstepset[usteplevelname]['factor']

    def getustepnames(self):
        return list(self.usteppins.microstepset.keys())

    def endstepping(self):
        """
        called when a software step or dma step has completed to reset motor state
        """
        self.drive_enable.setValue('disable', wv.myagents.app)
        
    def crashstop(self):
        """
        immediate stop.
        """
        self.drive_enable.setValue('disable', wv.myagents.user)
        self.mode.setValue('closed',wv.myagents.user)

    def getstepfunc(self, stepinf):
        """
        called when a new software step run is starting.
        """
        return self.step.trigger

    def getusteplevelint():
        return self.usteppins.getValue()

    def pulsegen(self, stepdef, command, targetpos, targetdir):
        """
        a wrapper for tickgen that yields the parameters for creating a pigpio pulse plus the new absolute position after each pulse.
        
        each yield returns a 5 tuple:
            0: mask of bits to turn on
            1: mask of bits to turn off
            2: microsecond time of this pulse or None if the motor is now stationary
            3: raw position of this motor after this pulse
            4: the name of this motor
            5: action:
                0: normal pulse
                -1:action complete - motor should be stopped
                1: no-op - motor is stationary but may start again later
        """
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        dirvar=self.direction
        setdir={'F':dirvar.getBits('F'),
                'R':dirvar.getBits('R')}
        dirbit=1<<dirvar.pinno
        self.tickgenactive=True
        stepdef.running=True
        newpos=self.rawposn.getValue()
        tickmaker=stepdef.tickgen(command,newpos)
        uslevelv, poschange = stepdef.getmicrosteps()
        stepvar=self.step
        pulselen=stepvar.pulsetime
        stepa=1<<stepvar.pinno if stepvar.vlist[0] == 0 else 0
        stepb=1<<stepvar.pinno if stepa is 0 else 0
        onbits, offbits = stepa, stepb
        dirmask=1<<dirvar.pinno
        usclock=0
        overflow=0.0
        pulseoff=[offbits, onbits, 0, 0, self.name, 0]
        pulseon=[onbits, offbits, 0, 0, self.name, 0]
        startup=True
        while True:
            try:
                tickspec=next(tickmaker)
            except StopIteration:
                yield (0, 0, usclock, newpos, self.name, -1)
                break
            if tickspec is None:
                yield 0, 0, usclock, newpos, self.name, 1
                delay, overflow = (None, 0)
            elif isinstance(tickspec, float):
                delay, overflow=divmod(tickspec*1000000+overflow,1)
                newpos += dirmult*poschange
                pulseon[2]=usclock
                pulseon[3]=newpos
                yield pulseon
                pulseoff[2]=usclock + pulselen
                pulseoff[3]=newpos
                yield pulseoff
                usclock += int(delay)
            else:
                dirmult = 1 if tickspec[0] == 'F' else -1
                print('dirset', tickspec[0],dirmult)
                dirbits=setdir[tickspec[0]]
                if startup:
                    driveenbits=self.drive_enable.getBits('enable')
                    mslevelbits=self.usteppins.pinbits(stepdef.usteplevel.getValue())
                    dirbits = (dirbits[0] | driveenbits[0] | mslevelbits[0], dirbits[1] | driveenbits[1] | mslevelbits[1])
                    startup=False
                yield (dirbits[0], dirbits[1], usclock, newpos, self.name, 0) # setup all the control pins and wait a mo
                delay, overflow=divmod(tickspec[1]*1000000+overflow,1)
                usclock += int(delay)
        holdtime=self.holdstopped.getValue()
        holddelay=100 if holdtime ==0 else int(round(holdtime*1000000))
        disbits=self.drive_enable.getBits('disable')
        if delay is None or delay  >= holddelay:
            usclock += 1
        else:
            dly=int(holddelay-delay)
            assert dly > 0
            usclock += dly
            yield 0, 0, usclock, newpos, self.name, 0
        usclock += 10
        yield disbits + (usclock, newpos, self.name, 0)
        yield (0,0,usclock+10, newpos, self.name, -1)

    def _thcloseIO(self):
        for pinname in self.pinsetuplist:
            getattr(self,pinname).shutdown()