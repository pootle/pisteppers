#!/usr/bin/python3

"""
This module provides the base class For driving unipolar and bipolar stepper motors via a number of different chips.

"""
import pootlestuff.watchables as wv
from pootlestuff.watchables import loglvls
import threading,  time

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
    
    stop:   stop the motor - if it is running in will stop in a controlled manner and the op mode will revert to stopped
    
    goto:   the motor will travel to the target position and then monitor target position for change until stop is requested.
    
    onegoto:the motor will travel to the target position and then revert to stopped.
    
    run:    the motor will run at the target speed and in the target direction until stop is requested

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

    commands=('none', 'close', 'stop', 'goto', 'run', 'onegoto', 'setpos')

    def __init__(self, name, app, value, wabledefs=[], **kwargs):
        """
        sets up a generic stepper motor driver.
        
        name    : used in log messages, and to identify motor in wave processing
        
        app     : multi motor controller
        
        value   : dict with saved values for watchables
        
        wabledefs: extra watchables to be included in the object 
        
        settings must be included in kwargs to define pins and fixed values for the motor. These vary
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
            ('userstepm',   wv.enumWatch,   rmodes[0],          False,  {'vlist': rmodes}),         # available stepping modes - used by the gui, but needs rmodes...
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
        """
        request motor to apply a command.
        
        command: 
            'none':     noop commmand 
            'close':    shuts down motor - no further actions can be taken on this class instance
            'stop':     requests motor to stop in a controlled way. For example if the motor is running with ramp=up / down. the maotor will ramp down and stop.
            'goto':     requests motor moves to 'targetpos' using 'stepmode'. 'targetpos' (and various other params) are monitored and action changes on the fly.
                        once the target position 'targetpos' continues to be monitored and the motor will respond if it changes. This continues until a 'stop'
                        is received.
            'run':      requests the motor moves in 'targetdir' using stepmode until a stop is issued. 'targetdir' (and various other params) are monitored
                        and action changes on the fly. This continues until a 'stop'
                        is received.
            'onegoto':  requests motor moves to 'targetpos' using 'stepmode'. Once the target is reached the motor stops.
            'setpos' :  if the motor is currently stopped the current position is changed to 'targetpos' without moving the motor.
        """
        if command=='None':
            return
        curmode=self.opmode.getValue()
        if curmode=='closed':
            raise ValueError('motor %s is closed - cannot respond.' % self.name)
        assert command in self.commands
        if command in ('goto', 'onegoto', 'run'):
            if curmode == 'stopped':
                if not stepmode in self.userstepm.vlist:
                    raise ValueError('stepmode error: stepmode %s not found for motor %s' % (stepmode, self.name))
                if command == 'run':
                    assert targetdir in ('fwd','rev')
                if command == 'goto' or command == 'onegoto':
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
                if not targetdir is None:                                                   # other changed values in the step generator settings
                    self.target_dir.setValue(1 if targetdir=='fwd' else -1,wv.myagents.app) # are picked up directly in the generator 
                self.updatetickerparams=True
        elif command=='close' or command=='stop':
            curmode=self.opmode.getValue()
            if curmode=='stopped' or curmode=='closed':
                self.drive_enable.setValue('disable', wv.myagents.app)
            elif curmode=='softrun' or curmode=='dmarun':
                self.stepactive=False
        elif command=='setpos':
            if curmode=='stopped':
                self.rawposn.setValue(int(targetpos), wv.myagents.app)
            else:
                raise ValueError('cannot set position in mode %s' % curmode)
        return None

    def dmarun(self, stepmode, **kwargs):
        """
        prepare to move or goto position in dma mode
        
        Check if this motor mode uses waves and if so return a generator
        The main work is done in the controller.   
        """
        stepgen=getattr(self.stepmodes,stepmode)
        mtype=stepgen.mode
        if mtype=='wave':
            self.stepactive=True
            self.opmode.setValue('dmarun', wv.myagents.app)
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
        uslevelv, poschange = stepinf.getmicrosteps()
#        tlf=open('tlog.txt','w')
        tstart=time.time()
        while True:
            try:
                ntick=next(tickmaker)
            except StopIteration:
                self.log(loglvls.INFO,'StopIteration!!!!!!!')
                break
            if isinstance(ntick, float):
                if stoppedtimer is None:
                    steptrig()
                    newpos += poschange*dirsign
                else:
                    self.drive_enable.setValue('enable', wv.myagents.app)
                    stoppedtimer=None
                tickctr+=1
                nextsteptime += ntick
                if time.time() > posupdatetime:
                    posnset(newpos, wv.myagents.app)
                    posupdatetime += .8
            elif ntick is None:
                nextsteptime += .05  #  If nothing to do just wait for a bit and go round again
                if time.time() > posupdatetime:
                    posnset(newpos, wv.myagents.app)
                    posupdatetime += .8
            else:
                dirsign=1 if ntick[0]=='F' else -1
                directionset(ntick[0], wv.myagents.app)
                nextsteptime += ntick[1]
            delay=nextsteptime - time.time()
            if delay > 0:
                time.sleep(delay)
            else:
                self.overrunctr+=1
                self.overruntime+=-delay
#        tlf.close()
        self.log(loglvls.INFO, "%s _slowrun complete, now at %s, %d overruns of %d ticks, total overrun: %7.3f." % (self.name, newpos, self.overrunctr, tickctr, self.overruntime))
        posnset(newpos, wv.myagents.app)
        self.endstepping()
        self.opmode.setValue('stopped', wv.myagents.app)
        self.stepactive=False
        self.log(loglvls.INFO, self.reporttimelog('softrun'))
