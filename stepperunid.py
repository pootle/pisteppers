#!/usr/bin/python3

"""
A module to drive stepper motors via a basic driver chip like ULN2003.

The driver directly controls the individual windings.
In software (slowstep) mode PWM is used to give fine control of the power to each winding. DMA mode only
switches each winding off or on, so fine grained microstepping is not practical, but much higher speeds
can be reached.
"""
from stepperbase import basestepper
import pootlestuff.watchables as wv
import pigpio


class directstepper(basestepper):
    """
    Drives a single unipolar stepper motor with 4 gpio pins driving a set of simple switches (such as a ULN2003).
    
    It provides the same functionality as the A4988 class - see that for further details.
    
    In slow mode, PWM is used on each output stage to to provide smoother running.
    
    In fast (wave) mode each output stage is simply turned on or off, with on used if the value for the pin is 128 - 255.
    Thus more detailed levels of microstepping are not sensible. Also, particularly if the motor is run at low speed, in this mode
    the motor can get quite hot so it may not be practical for extended continuous running - this depends on the motor, and the speed
    and load.
    
    For now the tables that control the stepping levels are built in.
    """
    def __init__(self, wabledefs, **kwargs):
        self.ustepTables={
                'single'    : {'factor':1, 'table':
                            ((255, 0, 0, 0), (0, 255, 0, 0), (0, 0, 255, 0), (0, 0, 0, 255))},          # energise each coil in turn
                'double'    : {'factor':1, 'table':
                            ((255, 255, 0, 0), (0, 255, 255, 0), (0, 0, 255, 255), (255, 0, 0, 255))},  # energise pairs of coils in turn
                'two'       : {'factor':2, 'table': 
                            ((255, 0, 0, 0), (128,128, 0, 0), (0,255, 0, 0), (0, 128, 128, 0),          # 
                              (0, 0, 255, 0), (0, 0, 128, 128), (0, 0, 0, 255), (128, 0, 0, 128))},
                'four'      : {'factor':4, 'table': ((255, 0, 0, 0),
                                                     (192, 64, 0, 0),
                                                     (128, 128, 0, 0),
                                                     (64, 192,0,0),
                                                     (0,255,0,0),
                                                     (0,192, 64, 0),
                                                     (0, 128, 128, 0),
                                                     (0, 64, 192, 0),
                                                     (0, 0, 255,0),
                                                     (0,0,192,64),
                                                     (0,0,128,128),
                                                     (0,0,64,192),
                                                     (0,0,0,255),
                                                     (64, 0, 0, 192),
                                                     (128,0,0,128),
                                                     (192, 0, 0, 64),
                               )},
            }
        self.usteptypes=list(self.ustepTables.keys())
        wables=wabledefs+[
            ('drive_enable',    wv.enumWatch,       'disable',      False, {'vlist': ('enable', 'disable')}),
            ('direction',       wv.enumWatch,       'F',            False, {'vlist': ('F','R')}),
            ('drive_pins',      wv.textWatch,       None,           False),    # list of the pins in use from json file
            ('drive_hold_power',wv.intWatch,        55,             False, {'minv':0, 'maxv':255}),   # power factor used when stationary
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.pins=[int(p) for p in self.drive_pins.getValue().split()]
        assert len(self.pins) == 4
        for i in self.pins:
            assert isinstance(i, int) and 0<i<32
        self.stepindex=0    # index into active stepping table to show which step we're at
        self.activetable=None
        self.lastpinvals=[None] * len(list(self.ustepTables.values())[0]['table'])
        self.output_enable(None, None, 'disable', None)
        self.drive_enable.addNotify(self.output_enable, wv.myagents.app)

    def output_enable(self,  watched, agent, newValue, oldValue):
        powlevel=self.drive_hold_power.getValue() if newValue=='enable' else 0
        print('--------------------disable outputs with power level', powlevel)
        pinvals=0 if self.activetable is None else self.activetable['table'][self.stepindex if self.stepindex < len(self.activetable) else 0] 
        for pix, p in enumerate(self.pins):
            self.pio.set_PWM_dutycycle(p, 0 if pinvals == 0 else powlevel) 
        self.log(wv.loglvls.DEBUG,' output pins set to dutycycle %d' % powlevel)

    def getmaxusteplevel(self):
        return max([st['factor'] for st in self.ustepTables.values()])

    def getusteplevel(self, usteplevelname):
        return self.ustepTables[usteplevelname]['factor']

    def getustepnames(self):
        return list(self.ustepTables.keys())

    def endstepping(self):
        """
        called when a software step or dma step has completed to reset motor state
        """
        for pix, p in enumerate(self.pins):
            self.pio.set_PWM_dutycycle(p, 0) 

    def crashstop(self):
        """
        immediate stop.
        """
        for p in self.pins:
            self.pio.set_PWM_dutycycle(p,0)
        self.mode.setValue('closed',wv.myagents.user)

    def getstepfunc(self, stepinf):
        """
        called when a new software step run is starting.
        """
        self.activetable=self.ustepTables[stepinf.usteplevel.getValue()]
        for i in range(len(self.lastpinvals)):
            self.lastpinvals[i]=None
        return self.stepmotor

    def stepmotor(self):
        """
        When stepping directly from code, this function is called for each step.
        """
        six=self.stepindex
        valtable=self.activetable['table']
        if self.direction.getValue()=='F':
            six += 1
            if six >= len(valtable):
                six=0
        else:
            six -= 1
            if six < 0:
                six=len(valtable)-1
        self.stepindex=six
        pvals=valtable[six]
        pio=self.app.pio
        for ix, pf in enumerate(pvals):
            if pf != self.lastpinvals[ix]:
                self.lastpinvals[ix] = pf
                pio.set_PWM_dutycycle(self.pins[ix], pf)

    def pulsegen(self, stepdef, command, targetpos, targetdir):
        """
        a wrapper for tickgen that yields the parameters for creating a pigpio pulse plus the new absolute position after each pulse.
        
        each yield returns a 5 tuple:
            0: mask of bits to turn on
            1: mask of bits to turn off
            2: microsecond time of this pulse or None if the motor is now stationary, but may want to move again (tracking targetpos)
            3: raw position of this motor after this pulse
            4: the name of this motor
            5: action:
                0: normal pulse
                -1:action complete - motor should be stopped
                1: no-op - motor is stationary but may start again later
        """
        usteping=stepdef.usteplevel.getValue()
        activetable=self.ustepTables[usteping]
        self.activestepm.setValue(usteping, wv.myagents.app)
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        pio=self.app.pio
        pinbits=[]
        for pinno in self.pins:
            pio.set_mode(pinno, pigpio.OUTPUT)
            pio.write(pinno,0)
            pinbits.append(1<<pinno)
        stepbits=[]
        for pinvals in activetable['table']:
            onb=0
            offb=0
            for ix, onebit in enumerate(pinbits):
                if pinvals[ix] > 127:
                    onb |= onebit
                else:
                    offb |= onebit
            stepbits.append((onb,offb))
        self.tickgenactive=True
        self.stepactive=True
        newpos=self.rawposn.getValue()
        tickmaker=stepdef.tickgen(command,newpos)
        uslevelv, poschange = stepdef.getmicrosteps()
        usclock=0
        overflow=0.0
        startup=True
        while True:
            try:
                tickspec=next(tickmaker)
            except StopIteration:
                yield (0, 0, usclock, newpos, self.name, -1)
                yield (0, 0, usclock+1, newpos, self.name, -1) # flush final position again
                break
            if tickspec is None:
                yield (0,0) + (usclock, newpos, self.name, 1)
                usclock += 100000   # set time to .1 seconds
            elif isinstance(tickspec, float):
                delay, overflow=divmod(tickspec*1000000+overflow,1)
                newpos += tabinc * poschange
                yield stepbits[self.stepindex] + (usclock, newpos, self.name, 0)
                usclock += int(delay)
                if tabinc > 0:
                    self.stepindex += 1
                    if self.stepindex >= len(pinbits):
                        self.stepindex=0
                else:
                    self.stepindex -= 1
                    if self.stepindex < 0:
                        self.stepindex = len(pinbits)-1
            else:
                tabinc=1 if tickspec[0]=='F' else -1
                delay, overflow=divmod(tickspec[1]*1000000+overflow,1)
                yield (0,0) + (usclock, newpos, self.name, 0)
                usclock += int(delay)