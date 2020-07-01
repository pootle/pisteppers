#!/usr/bin/python3

"""
A module to drive a stepper motor via a basic driver chip (such as a pololu A4988) or drive 
unipolar steppers via chips such as ULN2003..

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
    
    The class runs some aspects of the motor control in its own thread, commands are passed to that thread to process
    
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
        
        settings must be included in kwargs to define pins and fixed values and limits for the motor. These vary
        with the inheriting class.

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
        wables=wabledefs+[
            ('userstepm',   wv.enumWatch,   rmodes[0],          False,  {'vlist': rmodes}),         # available stepping modes
            ('targetrawpos',wv.intWatch,    0,                  False),                             # target position when in goto mode
            ('target_dir',  wv.intWatch,    1,                  False),                             # target direction when in run mode - +ve for fwd, -ve for reverse
            ('opmode',      wv.enumWatch,   self.opmodes[1],    False,  {'vlist': self.opmodes}),   # what mode is current - set by the motor
            ('stepstyle',   wv.enumWatch,   'off',              False,  {'vlist': ('off', 'soft', 'dma')}), # shows if motor is stepping and if so which type
            ('holdstopped', wv.floatWatch,  .5,                 False),     # when motor stops, drive_enable goes false after this time (in seconds), 0 means drive always enabled
            ('rawposn',     wv.intWatch,    0,                  False),     # current position in microsteps (not totally up to date while fast stepping)
            ('ticktime',    wv.floatWatch,  0,                  False),     # minimum step interval (clamped to minslow for slow stepping)
            ('stepmodes',   wv.watchablesmart,None,             False,  {'wabledefs': modewables}), # the available stepper control modes
        ]
        super().__init__(wabledefs=wables, app=app, value=value, **kwargs)
        self.pending=None
        self.dothis('stop')
        self.mthread=threading.Thread(name='stepper'+self.name, target=self._thrunner)
        self.log(loglvls.INFO,'starting motor %s thread stepper using class %s' % (self.name, type(self).__name__))
        self.maxstepfactor = self.usteplevel.maxusteps()
        self.mthread.start()

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
            return self.pulsegen(stepgen, **kwargs)
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
        self.log(loglvls.INFO, "%s entering mode 'stopped with timeout %4.2f" % (self.name, holdtimeout))
        while self.pending==None:
            if holdtimeout:
                self._thwaitq(waittill=holdtimeout)
            else:
                self._thwaitq(delay=1)
            if holdtimeout and holdtimeout < time.time():
                print('turn off', self.drive_enable.setValue('disable',wv.myagents.app))
                holdtimeout=None
        self.log(loglvls.INFO, "%s leaving mode 'stopped'" % self.name)

    def _thrun_faststep(self):
        """
        with runfast, the real work is done in the motorset class, so this just idles until the mode changes again
        """
        self.log(loglvls.INFO, "%s entering faststep" % self.name)
        while self.pending==None:
            self._thwaitq(delay=1)
        self.log(loglvls.INFO, "%s leaving faststep" % self.name)

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

    def _slowrun(self, stepinf, command, targetpos, targetdir):
        """
        drives the motor stepping from software.
        
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
        steptrig=self.getstepfunc()
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
        self.log(loglvls.INFO, "%s _slowrun complete, now at %s, %d overruns of %d ticks, total overrun: %7.3f." % (self.name, newpos, self.overrunctr, tickctr, self.overruntime))
        if not newpos is None:
             posnset(newpos, wv.myagents.app)
        self.log(loglvls.INFO, self.reporttimelog('slowrun'))

class ustepdefuni(wv.enumWatch):
    def __init__(self, motor, **kwargs):
        self.motor=motor
        super().__init__(**kwargs)

    def maxusteps(self):
        return max([st['factor'] for st in self.motor.stepTables.values()])

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
        self.stepTables={
                'single'    : {'factor':4, 'table':
                            ((255, 0, 0, 0), (0, 255, 0, 0), (0, 0, 255, 0), (0, 0, 0, 255))},          # energise each coil in turn
                'double'    : {'factor':4, 'table':
                            ((255, 255, 0, 0), (0, 255, 255, 0), (0, 0, 255, 255), (255, 0, 0, 255))},  # energise pairs of coils in turn
                'two'       : {'factor':2, 'table': 
                            ((255, 0, 0, 0), (128,128, 0, 0), (0,255, 0, 0), (0, 128, 128, 0),          # 
                              (0, 0, 255, 0), (0, 0, 128, 128), (0, 0, 0, 255), (128, 0, 0, 128))},
                'four'      : {'factor':1, 'table': ((255, 0, 0, 0),
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
        steptypes=list(self.stepTables.keys())
        wables=wabledefs+[
            ('drive_enable',    wv.enumWatch,       'disable',      False, {'vlist': ('enable', 'disable')}),
            ('direction',       wv.enumWatch,       'F',            False, {'vlist': ('F','R')}),
            ('drive_pins',      wv.textWatch,       '17 23 22 27',  False),    # list of the pins in use
            ('drive_hold_power',wv.intWatch,        55,             False, {'minv':0, 'maxv':255}),   # power factor used when stationary
            ('usteplevel',      ustepdefuni,        steptypes[0],   False, {'vlist': steptypes, 'motor': self}), # sets up the available step modes
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.pins=[int(p) for p in self.drive_pins.getValue().split()]
        assert len(self.pins) == 4
        for i in self.pins:
            assert isinstance(i, int) and 0<i<32
        self.stepindex=0    # index into self.stepTables to show where we are
        self.activetable=self.stepTables[self.usteplevel.getValue()]['table']
        self.lastpinvals=[None, None, None, None]
        self.output_enable(None, None, 'disable', None)
        self.drive_enable.addNotify(self.output_enable, wv.myagents.app)
        self.usteplevel.addNotify(self.setstepinfo, wv.myagents.app)
        self.usteplevel.addNotify(self.setstepinfo, wv.myagents.user)

    def output_enable(self,  watched, agent, newValue, oldValue):
        powlevel=self.drive_hold_power.getValue() if newValue=='enable' else 0
        print('--------------------disable outputs with power level', powlevel)
        pinvals=self.activetable[self.stepindex if self.stepindex < len(self.activetable) else 0] 
        for pix, p in enumerate(self.pins):
            self.pio.set_PWM_dutycycle(p, 0 if pinvals == 0 else powlevel) 
        self.log(wv.loglvls.DEBUG,' output pins set to dutycycle %d' % powlevel)

    def setstepinfo(self, watched, agent, newValue, oldValue):
        self.activetable=self.stepTables[newValue]['table']
        self.stepindex=0

    def getusteplevelint(self):
        return self.stepTables[self.usteplevel.getValue()]['factor']

    def crashstop(self):
        """
        immediate stop.
        """
        for p in self.pins:
            self.pio.set_PWM_dutycycle(p,0)
        self.mode.setValue('closed',wv.myagents.user)

    def getstepfunc(self):
        return self.stepmotor

    def stepmotor(self):
        """
        When stepping directly from code, this function is called for each step.
        """
        six=self.stepindex
        if self.direction.getValue()=='F':
            six += 1
            if six >= len(self.activetable):
                six=0
        else:
            six -= 1
            if six < 0:
                six=len(self.activetable)-1
        self.stepindex=six
        pvals=self.activetable[six]
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
        self.opmode.setValue({'goto': 'going', 'run': 'moving'}[command], wv.myagents.app)
        self.targetrawpos.setValue(targetpos, wv.myagents.app)
        self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app)
        pio=self.app.pio
        pinbits=[]
        for pinno in self.pins:
            pio.set_mode(pinno, pigpio.OUTPUT)
            pio.write(pinno,0)
            pinbits.append(1<<pinno)
        stepbits=[]
        for pinvals in self.activetable:
            onb=0
            offb=0
            for ix, onebit in enumerate(pinbits):
                if pinvals[ix] > 127:
                    onb |= onebit
                else:
                    offb |= onebit
            stepbits.append((onb,offb))
        self.usteplevel.setValue(stepdef.usteps.getValue(), wv.myagents.app)
        self.tickgenactive=True
        tickmaker=stepdef.tickgen()
        usclock=0
        overflow=0.0
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
                tabinc=1 if dirchange=='F' else -1
            yield stepbits[self.stepindex] + (usclock, newpos, self.name)
            if delay is None:
                break
            else:
                usclock += int(delay)
            if tabinc > 0:
                self.stepindex += 1
                if self.stepindex >= len(pinbits):
                    self.stepindex=0
            else:
                self.stepindex -= 1
                if self.stepindex < 0:
                    self.stepindex = len(pinbits)-1
        self.drive_enable.setValue('disable', wv.myagents.app)


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
            ('usteplevel',  gpp.usteplevel_pinset,None,         False),     # sets up the pins used to control microstep level
        ]
        self.pinsetuplist=('drive_enable','direction','step','usteplevel')
        super().__init__(wabledefs=wables, **kwargs)

    def crashstop(self):
        """
        immediate stop.
        """
        self.drive_enable.setValue('disable', wv.myagents.user)
        self.mode.setValue('closed',wv.myagents.user)

    def getstepfunc(self):
        return self.step.trigger

    def getusteplevelint():
        return self.usteplevel.getValue()

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
        
        each yield returns a 5 tuple:
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
        self.usteplevel.setValue(stepdef.usteps.getValue(), wv.myagents.app)
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

    def _thcloseIO(self):
        for pinname in self.pinsetuplist:
            getattr(self,pinname).shutdown()

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
        self.log(loglvls.INFO, "motoset leaving mode fast")
        for motor in motors.values():
            motor.dothis(command =   'stop')
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
