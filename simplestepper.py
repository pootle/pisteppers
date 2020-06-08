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

class stepcontrols(wv.watchablesmart):
    """
    This class implements the details of step timing with parameters to control the various speeds and
    ramping up and down 
    """
    def __init__(self, name, **kwargs):
        wables=[
            ('minstep',         wv.floatWatch,      .0017,      False),     # minimum step interval (-> fastest speed)
            ('startstep',       wv.floatWatch,      .006,       False),     # initial interval when starting
            ('rampfact',        wv.floatWatch,      1.1,        False),     # change in interval applied each ramptick when ramping
            ('rampintvl',       wv.floatWatch,      .05,        False),     # interval between speed changes in ramp mode
            ('rampticks',       wv.intWatch,        7700,       False),     # number of ticks to stop from max slow speed
            ('ramputicks',      wv.intWatch,        0,          False),     # actual ticks used to ramp up / down
        ]
        self.name=name
        super().__init__(wabledefs=wables, **kwargs)

#modevals=('closed', 'off', 'stopped', 'slowstep', 'faststep')
modevals=('closed', 'stopped', 'slowgoto', 'slowtrack', 'fastgoto')
#            0          1          2            3           4
class stepper(wv.watchablepigpio):
    """
    Drives a single stepper motor with 2 different styles of driving. Uses an A4988 style stepper controller.

    It supports:
        direct stepping (for slower speeds and with the potential for dynamic feedback control)
        
        fast stepping using dma control blocks to provide very accurate timing with ramp up and down
        capability
        
    The class runs the motor control in its own thread, some commands are passed to that thread to process
    
    rawposn is an integer representation of the position in microsteps at the highest microstep level
    """
    def __init__(self, stepcontroller, name, **kwargs):
        """
        sets up a stepper motor driver for an A4988 style chip.
        
        name    : used in log messages.
        
        settings must be included in kwargs to define pins and fixed values and limits for the motor.
        see the example file 'motor.cfg'

        kwargs  : other args passed to watchable.
        """
        self.name=name
        self.mthread=None
        self.cmndq=queue.Queue()
        self.overrunctr=0
        self.overruntime=0.0
        self.tlogs={}
        wables=[
            ('mode',        wv.enumWatch,   modevals[1],        False,  {'vlist': modevals}),   # what mode is current
            ('start_mode',  wv.enumWatch,   modevals[1],        True,   {'vlist': modevals}),   # mode to start in
            ('drive_enable',gpio_out_pin,   None,               False,  {'name': 'drive enable', 'loglevel': wv.loglvls.DEBUG}),  # sets up drive enable pin
            ('direction',   gpio_out_pin,   None,               False,  {'name': 'direction'}),     # sets up direction pin
            ('step',        gpio_trigger_pin,None,              False,  {'name': 'step'}),          # sets up step pin
            ('holdstopped', wv.floatWatch,  .5,                 False),     # when motor stops, drive_enable goes false after this time (in seconds), 0 means drive always enabled
            ('usteplevel',  usteplevel_pinset,None,             False),     # sets up the pins used to control microstep level
            ('rawposn',     wv.intWatch,    0,                  False),     # current position in microsteps (not totally up to date while fast stepping)
            ('ticktime',    wv.floatWatch,  0,                  False),     # minimum step interval (clamped to minslow for slow stepping)
            ('targetrawpos',wv.intWatch,    0,                  False),     # target position when in goto mode
            ('runmode',     wv.enumWatch,   'none',             False,  {'vlist': ('none', 'slowgoto', 'slowtrack', 'fastgoto')}), # mode to use for next movement
            ('fast',        stepcontroller, None,               False,  {'name': 'fast'}),
            ('slow',        stepcontroller, None,               False,  {'name': 'slow'}),
        ]
        self.pinsetuplist=('drive_enable','direction','step','usteplevel')
        super().__init__(wabledefs=wables, **kwargs)
        self.mode.addNotify(self.tracker, wv.myagents.user)
        self.mode.addNotify(self.tracker, wv.myagents.app)
        self.mthread=threading.Thread(name='stepper'+self.name, target=self._thrunner)
        self.log(loglvls.INFO,'starting motor %s thread stepper' % self.name)
        self.maxstepfactor = self.usteplevel.maxusteps()
        self.mthread.start()

    def tracker(self, oldValue, newValue, agent, watched):
        print('oldvalue: %s, newvalue: %s, by agent %s' %(oldValue, newValue, agent))

    def crashstop(self):
        """
        immediate stop eventually
        """
        self.drive_enable.setValue('disable',wv.myagents.app)
        self.mode.setValue('closed',wv.myagents.user)

    def cleanstop(self):
        """
        requests the driver thread to come to a clean stop and end the thread
        
        returns when thread has finished
        """
        self.mode.setValue('closed',wv.myagents.user)
        while self.mthread.is_alive():
            time.sleep(.3)
        self.log(loglvls.INFO,'motor %s thread stepper finished' % self.name)

    def setValue(self, **kwargs):
        """
        redefines the standard setValue to instead pass the request to the motor thread
        and make the changes in thread. This also means the motor responds quickly to mode changes.
        
        This expects:
            value: a dict of watchable name -> value
            agent: the agent making the change
        """
        self.cmndq.put_nowait((self._thsetValue, kwargs))

    def _thrunner(self):
        self.log(loglvls.INFO,'%s motor thread running', self.name)
        curmode=self.mode.getIndex()
        abs_start=time.time()
        abs_thread=time.thread_time()
        abs_process=time.process_time()
        while curmode != 0:  # mode zero means time to exit thread
            if curmode==1:      # motor is stopped - drop power after 'holdstopped'
                self._thrun_stopped()
            elif curmode==2 or curmode==3:    # slowgoto - software stepping until target reached, then stop
                self._slowrun()
            elif curmode==4:                  # fast goto mode - ticks generated here, control is in motorset controller
                self._thrun_faststep()
            else:
                self.log(loglvls.ERROR,'%s unknown motor mode %s (%d)' % (self.name, self.mode.getValue(), self.mode.getIndex()))
            curmode=self.mode.getIndex()
        self._thcloseIO()
        self.log(loglvls.INFO,'%s exiting motor thread. Execution summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (self.name,
                    time.time()-abs_start, time.process_time()-abs_process, time.thread_time()-abs_thread))
        return

    def _thwaitq(self, waittill):
        """
        waits on the command q till at most waittill.
        
        if nothing arrived on the q it returns True
        
        if anything returned on the q and has been processed it returns False (to signify the target time has not been reached)
        this allows loops to react quickly to change settings
        """
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
        method(**kwargs)
        return delay <= 0

    def _thsetValue(self, value, agent):
        if agent in self.app.agentclass:
            for n, v in value.items():
                getattr(self, n).setValue(v, agent)
        else:
            self.log(loglvls.FATAL, 'agent {} not known in setting var {} for {}'.format(agent, n, self.name)) 

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
        while self.mode.getIndex() == 1:
            self._thwaitq(time.time()+1 if holdtimeout is None else holdtimeout)
            if holdtimeout and holdtimeout < time.time():
                self.drive_enable.setValue('disable',wv.myagents.app)
        self.log(loglvls.INFO, "%s leaving mode 'stopped'" % self.name)

    def _thrun_faststep(self):
        """
        with runfast, the real work is done in the motorset class, so this just idles until the mode changes again
        """
        self.log(loglvls.INFO, "%s entering mode 'faststep'" % self.name)
        while self.mode.getIndex() == 4:
            self._thwaitq(time.time()+1)
        self.log(loglvls.INFO, "%s leaving mode 'faststep'" % self.name)

    def tickgen(self):
        """
        generator function for step times
        
        yields a sequence of 3-tuples until target is reached, at which point it returns None for the time interval
        
        each tuple is:
            0 : None if direction pin is unchanged, else the new value for the direction pin
            1 : time in seconds to the next step pin trigger - None if no step required (e.g. at target pos)
            2 : raw position after this step is issued
        """
        targetposget= self.targetrawpos.getValue            # set up functions to directly access the various info needed - dynamic fetch allows update on the fly
        uslevelget  = self.usteplevel.getValue              # current microstep level
        mode=self.mode.getIndex()
        if mode==2 or mode ==3:
            params=self.slow
        else:
            params=self.fast
        starttickget= params.startstep.getValue               # minimum time for a first tick
        mintickget=params.minstep.getValue                    # minimum tick time once acceleration complete - fastest we can go
        tickget=self.ticktime.getValue                      # current target tick time - 0 gives max speed in this mode (clamped to 'minslow')
        ramptget=params.rampticks.getValue                # number of ticks needed to slow down from max speed (takes too long to calculate on the fly)
        rampintvlget=params.rampintvl.getValue
        rampfactget=params.rampfact.getValue
        currentposv=self.rawposn.getValue()                 # get the current position - we assume we are in control of this so don't re-read it
        utickctr=0
        currenttick=None                                    # set stationary
        elapsedtime=0.0
        self.log(loglvls.DEBUG, '%s tickgen setup complete %d usteps per step' % (self.name, self.maxstepfactor // uslevelget()))
        while self.tickgenactive:
            dirset=None
            movesteps=targetposget()-currentposv
            uslevelv=uslevelget()
            usteps=self.maxstepfactor // uslevelv
            if abs(movesteps) < self.maxstepfactor // uslevelv:  # if less than the current step size stop now before we start
                yield (None, None, currentposv)
                currenttick=None 
            else:
#                self.log(loglvls.DEBUG, 'moving %d' % movesteps)
                if currenttick is None:
                    forwards=movesteps > 0
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
                    self.log(loglvls.DEBUG, '% starts currenttick %5.4f, scaled to: %6.5f, ramptick %s' % (
                                self.name, currenttick, currenttick/uslevelv, ('none' if nextramptick is None else '%5.4f' %rintvl)))
                else:   # we're moving...
                    if forwards == (movesteps > 0): # and we're going the right way
                        if abs(movesteps) < ramptget():                 # check if we need to slow down
                            targettickv=starttickget()                  # target is initial speed
                            if currenttick < targettickv:               # are we going too fast?
                                if nextramptick is None or elapsedtime > nextramptick:
                                    currenttick = currenttick*rampfactget() # slow down a bit
                                    if currenttick > targettickv:       # have we passed slowest speed?
                                        currenttick = targettickv
                                        nextramptick=None
#                                        print('reached min %5.4f' % currenttick)
                                    else:
                                        if nextramptick is None:
                                            nextramptick = elapsedtime+rampintvlget()
                                        else:
                                            nextramptick += rampintvlget()  # set time for next speed change
#                                        print('speed to %5.4f' % currenttick)
                            else:
                                currenttick = targettickv               # make sure we're at target and now we go for constant speed
                                nextramptick=None
                        else:
                            targettickv=max(tickget(), mintickget())     # how fast do we want to go?
                            if targettickv < currenttick:               # -- faster
                                if nextramptick is None or elapsedtime > nextramptick:  # time to go faster?
                                    currenttick = currenttick / rampfactget() # speed up a bit
                                    if currenttick < targettickv:       # too fast?
                                        currenttick = targettickv
                                        nextramptick=None
                                        print('%s reached max %5.4f at tick %d' % (self.name, currenttick, utickctr))
                                    else:
                                        if nextramptick is None:
                                            nextramptick = elapsedtime+rampintvlget()
                                        else:
                                            nextramptick += rampintvlget()
#                                        print('speed to %5.4f' % currenttick)
                            else:
                                currenttick = targettickv               # make sure we're at target speed and now we go for constant speed
                                nextramptick=None
   
                    else:   # direction changed - start slowing down
                        if currenttick >= starttickget(): # now go to stopped
                            self.log(loglvls.DEBUG,'change dir - stopped')
                            currenttick=None
                        elif nextramptick is None:
                            self.log(loglvls.DEBUG, 'set nextramptick')
                            nextramptick = elapsedtime+rampintvlget()
                        elif elapsedtime < nextramptick:
                            pass
                        else:
                            currenttick *= rampfactget()
                            self.log(loglvls.DEBUG, 'slowing.... %5.4f' % currenttick)
                            nextramptick += rampintvlget()
                currentposv += usteps if forwards else -usteps
                utickctr += usteps
                yv=.3 if currenttick is None else currenttick/uslevelv
                elapsedtime += yv
                yield (dirset, yv, currentposv)

    def pulsegen(self):
        """
        a wrapper for tickgen that yields the parameters for creating a pigpio pulse plus the new absolute position after each pulse.
        
        thus each yield returns a 5 tuple:
            0: mask of bits to turn on
            1: mask of bits to turn off
            2: microsecond time of this pulse or None if the motor is now stationary
            3: raw position of this motor after this pulse
            4: the name of this motor
        """
        tickmaker=self.tickgen()
        dirvar=self.direction
        dirbit=1<<dirvar.pinno
        if dirvar.vlist.index('F') == 1:
            setdir={'F': (dirbit,0),
                    'R': (0, dirbit)}
        else:
            setdir={'F': (0, dirbit),
                    'R': (dirbit,0)}
        stepvar=self.step
        pulselen=stepvar.pulsetime
        stepa=1<<stepvar.pinno if stepvar.vlist[0] == 0 else 0
        stepb=1<<stepvar.pinno if stepa is 0 else 0
        stepon=(stepa, stepb)
        stepoff=(stepb,stepa)
        dirmask=1<<dirvar.pinno
        usclock=0
        overflow=0.0
        while True:
            try:
                dirchange, ticktime, newpos = next(tickmaker)
            except StopIteration:
                break
            onbits, offbits = stepon
            if ticktime is None:
                delay, overflow = None, 0
            else:
                delay, overflow=divmod(ticktime*1000000+overflow,1)
            if dirchange:
                dirbits=setdir[dirchange]
                yield (onbits | dirbits[0], offbits | dirbits[1], usclock, newpos, self.name)
            else:
                yield (onbits, offbits, usclock, newpos, self.name)
            usclock += pulselen
            yield (offbits, onbits, usclock, newpos, self.name)
            if delay is None:
                break
            else:
                usclock += int(delay)-pulselen

    def _slowrun(self):
        """
        runs a simple loop sending trigger pulses to the chip's step pin from code.
        
        It gets the step interval from the tick generator, which monitors both the target mode of the motor and
        the target position to manage ramping, it uses _thwaitq to delay for the required time, _thwaitq also
        picks up incoming commands and processes them.
        """
        self.drive_enable.setValue('enable', wv.myagents.app)
        self.log(loglvls.INFO, '%s _slowrun starts' % self.name)
        self.tickgenactive=True
        realstep=True
        self.overrunctr=0
        self.overruntime=0.0
        tickmaker=self.tickgen()
        steptrig=self.step.trigger
        directionset=self.direction.setValue
        posnset = self.rawposn.setValue
        self.starttimelog('slowrun')
        nextsteptime=time.time()
        posupdatetime=nextsteptime+.8
        tickctr=0
        newpos=self.rawposn.getValue()
        stoppedtimer=None
        while True:
            if realstep:
                try:
                    dirchange, ticktime, newpos = next(tickmaker)
                except StopIteration:
                    break
                if dirchange:
                    directionset(dirchange, wv.myagents.app)
                if ticktime is None:
                    mode=self.mode.getIndex()
                    if mode==2:  # its a goto - exit to stopped mode
                        self.mode.setIndex(1,wv.myagents.app)
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
                else:
                    if not stoppedtimer is None:
                        self.drive_enable.setValue('enable', wv.myagents.app)
                    steptrig()
                    stoppedtimer=None
                    tickctr+=1
                    nextsteptime += ticktime
                if time.time() > posupdatetime:
                    posnset(newpos, wv.myagents.app)
                    posupdatetime += .8
            if not self.mode.getIndex() in (2,3):
                self.tickgenactive=False
            realstep=self._thwaitq(nextsteptime)
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

    def goto(self, steppos):
        """
        a handy function useful for testing a single freestanding motor
        """
        tpos=round(steppos * self.maxstepfactor)
        self.targetrawpos.setValue(tpos, wv.myagents.app)
        self.mode.setValue('slowstep', wv.myagents.app)
        self.log(loglvls.INFO,'%s new target raw pos %d' % (self.name, tpos))

    def fastgoto(self):
        """
        prepare to goto position in targetrawpos    
        """
        self.drive_enable.setValue('enable', wv.myagents.app)
        self.tickgenactive=True
        return self.pulsegen()

controllermodes=('closed', 'off', 'faststep')

class multimotor(wv.watchablepigpio):
    def __init__(self, motors, **kwargs):
        wables=[
            ('pigpmspw',    wv.intWatch,    0,                  False),
            ('pigpppw',     wv.intWatch,    0,                  False),
            ('pigpbpw',     wv.intWatch,    0,                  False),
            ('mode',        wv.enumWatch,   controllermodes[1], False,  {'vlist': controllermodes}),
            ('gotonow',     wv.btnWatch,    'goto now',         False),
            ('wavepulses',  wv.intWatch,    1500,               True,   {'minv':100}),
        ]
        super().__init__(wabledefs=wables, loglevel=loglvls.DEBUG, **kwargs)
        self.pigpmspw.setValue(self.pio.wave_get_max_micros(), wv.myagents.app)
        self.pigpppw.setValue(self.pio.wave_get_max_pulses(), wv.myagents.app)
        self.pigpbpw.setValue(self.pio.wave_get_max_cbs(), wv.myagents.app)
        self.motors={}
        print(self.startsettings.keys())
        print(motors)
        for motname, motclass in motors:
            newmotor=self.makeChild(defn=(motname, motclass, None, False, {'name': motname}), value=self.startsettings.get(motname,{}))
            if newmotor is None:
                raise ValueError('motor construction failed for %s' % motname)
            self.motors[motname]=newmotor
        self.cmndq=queue.Queue()
        self.cthread=threading.Thread(name='controller(%s)' % type(self).__name__, target=self._thrunner)
        self.cthread.start()

    def close(self):
        self.cleanstop()

    def cleanstop(self):
        for motor in self.motors.values():
            motor.cleanstop()
        self.mode.setValue('closed', wv.myagents.app)

    def _thrunner(self):
        self.log(loglvls.INFO,'multimotor control thread running' )
        abs_start=time.time()
        abs_thread=time.thread_time()
        abs_process=time.process_time()
        curmode=self.mode.getValue()
        while curmode != 'closed':
            modefunc=getattr(self, '_thrun_'+curmode, None)
            if callable(modefunc):
                modefunc()
            else:
                self.log(loglvls.FATAL,'unable to find mode method for %s (%s)' % (curmode, '_thrun_'+curmode))
                break
            curmode=self.mode.getValue()
        self.log(loglvls.INFO,'exiting control thread. Execution summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (
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

    def _thrun_off(self):
        """
        this function runs in the control thread while the mode is 'off'. It just waits for the mode to change.
        
        It is called when the thread first runs (because the mode will be 'off') and called subsequently
        whenever the mode changes to 'off'
        """
        self.log(loglvls.INFO, "entering mode 'off'")
        while self.mode.getValue() == 'off':
            self._thwaitq(time.time()+4.5)
        self.log(loglvls.INFO, "leaving mode 'off'")

    def _thrun_faststep(self):
        """
        fast step request - get the motors involved and .....
        motorposns: a dict with a key for each motor to use, and the value is the target position for that motor
        """
        self.log(loglvls.INFO, "starting mode 'fast")
        timestart=time.time()
        mgens=[]
        motors=[]
        for mname, motor in self.motors.items():
            if motor.runmode.getIndex()==3:
                tgen=motor.fastgoto()
                if not tgen is None:
                    mgens.append((mname, tgen))
                    motor.mode.setIndex(4, wv.myagents.app)
                    motor.drive_enable.setValue('enable', wv.myagents.app)
                    motors.append(motor)
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
            logf=open('tlog.txt','w')
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
#                            self[mn+'/rawposn'].setValue(mp,'driver')
                            self.motors[mn].rawposn.setValue(mp, wv.myagents.app)
                    elif current == pendingbufs[0]:
                        pass
#                        self.log(loglvls.DEBUG,'wave %d running' % current)
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
                        print('--------pos updated')
                elif current ==pendingbufs[0]:
                    self.log(loglvls.DEBUG,'wave %d running' % current)
                else:
                    self.log(loglvls.DEBUG,'AAAAAAAAAAAAAAAAAAAAAAAAAAAAArg')
#            self.pigp.wave_clear()
            if not logf is None:
                logf.close()
        self.log(loglvls.INFO, "leaving mode 'fast")
        for motor in motors:
            motor.mode.setIndex(1, wv.myagents.app)
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
        while True:
            if len(mpulses) == 0:
                break
            elif len(mpulses) == 1:
                yield mpulses[0]
                try:
                    mpulses[0]=next(newgens[0][1])
                except StopIteration:
                    mpulses=[]
            else:
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

pinsm1={

    'ticksperunit'  : 144*120*48/360,  # 144 is worm drive ratio, 128 is gearbox ratio, 48 steps per rev on stepper motor - unit is 1 degree
}

pinsm2={

    'ticksperunit'  : 144*120*48/360,  # 144 is worm drive ratio, 128 is gearbox ratio, 48 steps per rev on stepper motor - unit is 1 degree
}

m1params={
    '12V':    {'minslow': .0027, 'usteplevel':2},
    'ra24V':  {'minslow': .0020, 'usteplevel':2, 'startslow': .003,'rampintvl':.001, 'rampfact': 1.001, 'ticktime': 0},
    'dec24V': {'minslow': .0020, 'usteplevel':2, 'startslow': .003,'rampintvl':.001, 'rampfact': 1.001, 'ticktime': 0},
    'ra24vfast': {'minslow': .0010, 'usteplevel':2, 'startslow': .003,'rampintvl':.001, 'rampfact': 1.0005, 'ticktime': 0},
    'dec24Vfast': {'minslow': .0011, 'usteplevel':2, 'startslow': .003,'rampintvl':.001, 'rampfact': 1.0005, 'ticktime': 0},
}
# note 24v values work with both motors running in slow mode
