#!/usr/bin/python3

"""
A module to drive stepper motors via a basic driver chip like the A4988 or even ULN2003.

For A4988 style drivers, it (optionally) controls pins for drive enable, direction and microstep
level, and sends timed pulses to the step pin.

The step pin can be driven direct from the software (slowstep), which enables pulse by pulse control
of the timing - for example where some sort of feedback / PID is required. The timing of individual
pulses is subject to process / thread scheduling, so pulses can be delayed (typically by up to 
1 or 2 milliseconds on a multicore machine, more on a single core machine), but longer delays can occur.

The step pin can also be driven by pre-prepared memory blocks driving the gpio pins via DMA (faststep).
This provides highly accurate timings (accurate to 1 microsecond). The software prepares these memory blocks
on a JIT basis, but this does mean that feedback control (for example) will have a delay before 'reaching'
the motor. The delay depends on how fast the motor is going and how many motors are being run in parallel.

For direct driven motors via (for example) ULN2003), the driver directly controls the individual windings.
In software (slowstep) mode PWM is used to give fine control of the power to each winding. DMA mode only
switches each winding off or on, so fine grained microstepping is not practical, but much higher speeds
can be reached.

For both drivers fast and slow modes provide functionality to ramp the motor speedup and down, both to allow faster
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
import gpiopins as gpp

class basestepper(wv.watchablepigpio):
    """
    A base class for stepper motors that takes care of the higher level logic.

    It supports:
        direct stepping from software (for slower speeds and with the potential for responsive dynamic feedback control)
        
        fast stepping using dma control blocks to provide very accurate timing with ramp up and down
        capability using the motorset class.

    The motor accepts a small number of commands:
    --------------------------------------------- 
       
    close:  close the motor down - it cannot be used again once closed
    
    stop:   stop the motor - if it is running in will stop in a controlled manner
    
    goto:   the motor will travel to the target position
    
    run:    the motor will run at the target speed until stop is requested

    The motor can be in 1 of a number of (operation) modes. These are set by the motor and show the current state.
    -------------------------------------------------------
    
    closed  : Motor has shut down and cannot be used - the motor can only be run again by re-creating the object
    
    stopped :The motor is not currently active. There may be drive current to hold position.
    
    running : The motor is actioning a command - usually moving but it may be stationary waiting (for example,
                paused when about to change direction, or waiting for the target location to change in goto mode)
    
    stepmodes provide detailed control of step timings by defining the class used to generate tick intervals 
     as well as whether the stepping will by directly from software or using DMA controlled stepping.
    
    The class runs some aspects of the motor control in its own thread, commands are passed to that thread to process
    
    rawposn is an integer representation of the position in microsteps at the highest microstep level.
    """
    
    opmodes= ('closed', 'stopped', 'softrun', 'dmarun')

    commands=('none', 'close', 'stop', 'goto', 'run')

    def __init__(self, name, app, value, wabledefs=[], **kwargs):
        """
        sets up a generic stepper motor driver.
        
        name    : used in log messages, and to identify motor in wave processing
        
        app     : multi motor controller
        
        value   : dict with saved values for watchables
        
        wabledefs: extra watchables to be included in the object 
        
        settings must be included in kwargs to define pins and fixed values and limits for the motor. These vary
        with the inheriting class.

        see the example file 'motorset.json'

        kwargs  : other args passed to watchable.
        """
        self.name=name
        self.mthread=None
        self.tlogs={}
        modewables=[]
        rmodes=[]
        for smode, svals in value['stepmodes'].items():         # find the stepmodes declared in values (originally from json file) and make fields defs
            modewables.append((smode,   app.classdefs[svals['stepclass']], None, False, {'name': smode, 'motor': self}))
            rmodes.append(smode)
        self.stepmodenames=rmodes
        wables=wabledefs+[
            ('userstepm',   wv.enumWatch,   rmodes[0],          False,  {'vlist': rmodes}),         # available stepping modes - belongs in web part, but needs rmodes...
            ('targetrawpos',wv.intWatch,    0,                  False),                             # target position for goto
            ('target_dir',  wv.intWatch,    1,                  False),                             # target direction for run - +ve for fwd, -ve for reverse
            ('opmode',      wv.enumWatch,   self.opmodes[1],    False,  {'vlist': self.opmodes}),   # what mode is current - set by the motor
            ('holdstopped', wv.floatWatch,  .5,                 False),     # when motor stops, drive_enable goes false after this time (in seconds), 0 means drive always enabled
            ('rawposn',     wv.intWatch,    0,                  False),     # current position in microsteps (not totally up to date while fast stepping)
            ('ticktime',    wv.floatWatch,  0,                  False),     # minimum step interval (clamped to minslow for slow stepping)
            ('stepmodes',   wv.watchablesmart,None,             False,  {'wabledefs': modewables}), # the available stepper control modes
            ('activestepm', wv.textWatch,   '-',                False),     # when running shows the active step mode
        ]
        super().__init__(wabledefs=wables, app=app, value=value, **kwargs)
        self.log(loglvls.INFO,'starting motor %s thread stepper using class %s' % (self.name, type(self).__name__))
        self.maxstepfactor = self.getmaxusteplevel()

    def waitstop(self):
        if not self.mthread is None:
            self.mthread.join()
        self.drive_enable.setValue('disable', wv.myagents.app)
        self.opmode.setValue('closed', wv.myagents.app)

    def dothis(self, command, targetpos=None, targetdir=None, stepmode=None):
        curmode=self.opmode.getValue()
        if curmode=='closed':
            return None
        assert command in self.commands
        if command in ('goto', 'run'):
            if curmode == 'stopped':
                assert stepmode in self.userstepm.vlist
                if command == 'run':
                    assert targetdir in ('fwd','rev')
                if command == 'goto':
                    assert isinstance(targetpos, (int,float))
                stepdef=getattr(self.stepmodes, stepmode)
                if stepdef.mode=='software':
                    self.stepactive=True
                    self.stepinf=stepdef
                    self.opmode.setValue('softrun', wv.myagents.app)   # opmode returns to stopped when the thread is about to exit
                    self.mthread= threading.Thread(name=self.name+'_softrun', target=self._softrun, kwargs={
                            'stepinf': stepdef,
                            'command': command, 
                            'targetpos': targetpos, 
                            'targetdir': targetdir})
                    self.mthread.start()
                elif stepdef.mode=='wave':
                    return 'wave'
                else:
                    raise NotImplementedError('oopsy')
            else:
                if not targetpos is None:
                    self.targetrawpos.setValue(targetpos, wv.myagents.app)                  # these 2 are monitored by the stepgenerator
                if not targetdir is None:
                    self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
                
        elif command=='close' or command=='stop':
            curmode=self.opmode.getValue()
            if curmode=='stopped' or curmode=='closed':
                self.drive_enable.setValue('disable', wv.myagents.app)
            elif curmode=='softrun' or curmode=='dmarun':
                print('tell ticker to stop')
                self.stepactive=False
        elif command != 'none':
            print('putting', command)
            self.cmndq.put_nowait((self._command, {'command': command, 'targetpos': targetpos, 'targetdir': targetdir, 'stepmode': stepmode}))
            if command in ('goto', 'run'):
                mo = getattr(self.stepmodes,stepmode).mode
                return mo        
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
            print('setup pulse', stepgen.mode, type(stepgen).__name__)
            self.stepactive=True
            return self.pulsegen(stepgen, **kwargs)
        return None

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

    def _softrun(self, stepinf, command, targetpos, targetdir):
        """
        drives the motor stepping from software.
        
        It gets the step interval from the tick generator, which monitors both the target mode of the motor and
        the target position to manage ramping.
        """
        self.targetrawpos.setValue(targetpos, wv.myagents.app)                  # these 2 are monitored by the stepgenerator
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app) # ditto
        self.activestepm.setValue(stepinf.usteplevel.getValue(), wv.myagents.app)
        self.drive_enable.setValue('enable', wv.myagents.app)
        self.log(loglvls.INFO, '%s _softrun starts' % self.name)
        self.overrunctr=0
        self.overruntime=0.0
        tickmaker=stepinf.tickgen(command, self.rawposn.getValue())
        steptrig=self.getstepfunc(stepinf)
        directionset=self.direction.setValue
        posnset = self.rawposn.setValue
        self.starttimelog('softrun')
        nextsteptime=time.time()
        posupdatetime=nextsteptime+.8
        tickctr=0
        newpos=self.rawposn.getValue()
        stoppedtimer=None
#        tlf=open('tlog.txt','w')
        tstart=time.time()
        while True:
            try:
                dirchange, ticktime, newpos = next(tickmaker)
            except StopIteration:
                self.log(loglvls.INFO,'StopIteration!!!!!!!')
                break
            if dirchange:
                print('setting direction to', dirchange)
                directionset(dirchange, wv.myagents.app)
            if ticktime is None:
#                    tlf.write('skip at %7.5f\n' % (time.time()-tstart))
                mode=self.opmode.getIndex()
                if mode==2:  # its a goto - exit
                    self.log(loglvls.INFO,'null tick with goto - completed goto')
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
                        self.log(loglvls.INFO, "{} has turned off drive current.".format(self.name))
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
            delay=nextsteptime - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                self.overrunctr+=1
                self.overruntime+=-delay
#        tlf.close()
        self.log(loglvls.INFO, "%s _slowrun complete, now at %s, %d overruns of %d ticks, total overrun: %7.3f." % (self.name, newpos, self.overrunctr, tickctr, self.overruntime))
        if not newpos is None:
             posnset(newpos, wv.myagents.app)
        self.endstepping()
        self.log(loglvls.INFO, self.reporttimelog('softrun'))

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
            ('drive_pins',      wv.textWatch,       '17 23 22 27',  False),    # list of the pins in use
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
        self.opmode.setValue('stopped', wv.myagents.app)
        self.stepactive=False

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
            2: microsecond time of this pulse or None if the motor is now stationary
            3: raw position of this motor after this pulse
            4: the name of this motor
        """
        usteping=stepdef.usteplevel.getValue()
        activetable=self.ustepTables[usteping]
        self.activestepm.setValue(usteping, wv.myagents.app)
        self.opmode.setValue('dmarun', wv.myagents.app)
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
        tickmaker=stepdef.tickgen(command,self.rawposn.getValue())
        usclock=0
        overflow=0.0
        startup=True
        while True:
            try:
                dirchange, ticktime, newpos = next(tickmaker)
            except StopIteration:
                break
            if ticktime is None:
                self.log(loglvls.INFO,'Null pulse - we are done')
                break
#                delay, overflow = (None, 0)
            else:
                delay, overflow=divmod(ticktime*1000000+overflow,1)
            if not dirchange is None:
                tabinc=1 if dirchange=='F' else -1
            yield stepbits[self.stepindex] + (usclock, newpos, self.name)
            if delay is None:
                break
            else:
                usclock += int(delay)
            try:
                if tabinc > 0:
                    self.stepindex += 1
                    if self.stepindex >= len(pinbits):
                        self.stepindex=0
                else:
                    self.stepindex -= 1
                    if self.stepindex < 0:
                        self.stepindex = len(pinbits)-1
            except:
                print('oooopsy ---------------', dirchange, ticktime, newpos)

class A4988stepper(basestepper):
    """
    Extends basestepper for A4988 / DRV8825 style stepper driver chips with signals controlled directly by gpio pins.
    """
    def __init__(self, wabledefs=[], **kwargs):
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
        self.opmode.setValue('stopped', wv.myagents.app)
        self.stepactive=False
        
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
        """
        self.opmode.setValue('dmarun', wv.myagents.app)
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        dirvar=self.direction
        setdir={'F':dirvar.getBits('F'),
                'R':dirvar.getBits('R')}
        dirbit=1<<dirvar.pinno
        self.tickgenactive=True
        stepdef.running=True
        tickmaker=stepdef.tickgen(command, self.rawposn.getValue())
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
        startup=True
        while True:
            try:
                dirchange, ticktime, newpos = next(tickmaker)
            except StopIteration:
                delay, overflow = (None, 0)
                break
            if ticktime is None:
                delay, overflow = (None, 0)
            else:
                delay, overflow=divmod(ticktime*1000000+overflow,1)
            if dirchange:
                dirbits=setdir[dirchange]
                if startup:
                    driveenbits=self.drive_enable.getBits('enable')
                    mslevelbits=self.usteppins.pinbits(stepdef.usteplevel.getValue())
                    dirbits = (dirbits[0] | driveenbits[0] | mslevelbits[0], dirbits[1] | driveenbits[1] | mslevelbits[1])
                    startup=False
                yield (dirbits[0], dirbits[1], usclock, newpos, self.name) # setup all the control pins and wait a mo
                pulseon[2]=usclock+1
                pulseon[3]=newpos
                yield pulseon
                pulseoff[2]=usclock + 1 + pulselen
                pulseoff[3]=newpos
            else:
                pulseon[2]=usclock
                pulseon[3]=newpos
                yield pulseon
                pulseoff[2]=usclock + pulselen
                pulseoff[3]=newpos
#            print('d', pulseoff[2])
            yield pulseoff
            if delay is None:
                usclock += pulselen
                break
            else:
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
#            print('e', usclock)
            yield 0, 0, usclock, newpos, self.name
#        print('--->', usclock)
        yield disbits + (usclock, newpos, self.name)

    def _thcloseIO(self):
        for pinname in self.pinsetuplist:
            getattr(self,pinname).shutdown()

def pinnos(vv):
    return ' '.join(['%2d' % i for i in range(32) if (1<<i & vv) !=0])

controllermodes=('closed', 'off', 'faststep')

class multimotor(wv.watchablepigpio):
    def __init__(self, gpiolog=False, loglvl=loglvls.INFO, **kwargs):
        wables=[
            ('pigpmspw',    wv.intWatch,    0,                  False),
            ('pigpppw',     wv.intWatch,    0,                  False),
            ('pigpbpw',     wv.intWatch,    0,                  False),
            ('mode',        wv.enumWatch,   controllermodes[1], False,  {'vlist': controllermodes}),
            ('gotonow',     wv.btnWatch,    'Action',           False),
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
            motor.dothis(command='close')
        for motor in self.motors.values():
            motor.waitstop()
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
        motorlist: a dict with a key for each motor to use, and the value is the params for the motor
        """
        self.log(loglvls.INFO, "starting mode 'fast")
        timestart=time.time()
        mgens=[]
        motors={}
        for motor, moveparams in motorlist:
            tgen=motor.fastgoto(**moveparams)
            if not tgen is None:
                mgens.append((motor.name, tgen))
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
            maxpulses=self.wavepulses.getValue()
#            logf=open('pulselog.csv','w')
            logf=None
            while moredata:
                while len(pendingbufs) < 3 and moredata:
                    nextbuff=[]
                    mposns={}
                    bufftime=1000000            # count the time and stop adding pulses if we get to 1 second
                    while len(nextbuff) < maxpulses:
                        try:
                            nextp=next(mergegen)
                        except StopIteration:
                            nextp=None
                        if nextp is None:
                            nextbuff.append(pigpio.pulse(thisp[0], thisp[1], 1))
                            moredata=False
                            thisp=nextp
                            break
                        else:
                            dtime=nextp[2]-thisp[2]
                            assert dtime >= 0
                            nextbuff.append(pigpio.pulse(thisp[0], thisp[1], dtime))
                            thisp=nextp
                            bufftime -= dtime
                            if bufftime <= 0:
                                break
                        mposns[thisp[4]]=thisp[3]
                    if len(nextbuff) > 0:
                        if not logf is None:
                            for pp in nextbuff:
                                if pp.delay != 2:
                                    logf.write('%8x, %8x, %5d\n' % (pp.gpio_on, pp.gpio_off, pp.delay))
                        try:
                            pcount=self.pio.wave_add_generic(nextbuff)
                        except Exception as ex:
                            '''oh dear we screwed up - let's print the the data we sent'''
                            print('FAIL in wave_add_generic' + str(ex))
                            for i, p in enumerate(nextbuff):
                                print('%4d: on: %8x, off: %8x, delay: %8d' % (i, p.gpio_on, p.gpio_off, p.delay ))
                            raise
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
            if not logf is None:
                logf.close()
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
        self.log(loglvls.INFO, "motoset leaving mode fast")
        for motor in motors.values():
            motor.endstepping()
#            motor.dothis(command =   'stop')
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
            gen=newgens[0][1]
            yield mpulses[0]
            while True:
                try:
                    yield next(gen)
                except StopIteration:
                    self.log(loglvls.INFO,'pulsemerge single pulse mode - complete')
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
