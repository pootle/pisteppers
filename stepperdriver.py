#!/usr/bin/python3
#these imports are for the low level code
import pigpio
from pigpio import pulse as pgpulse
import time
from collections import deque
import heapq
import socket
import messageMan_sel as mms

class motor():
    """
    knows about and drives a single stepper motor via a pololu A4988 driver chip, with help from the parent
    which has the pigpio interface.
    
    It has a few modes which limit possible actions:
        idle     : nothing happening and drive chip is disabled
        fastwave : a fast move is in progress, passing steps to the parent through the pulser generator
        slowstep : slow stepping in progress, potentially with feedback correction
    """
    
    # default mapping of microstep levels to enable / disable on the mspins
    microstepset={
        1: (0,0,0),
        2: (1,0,0),
        4: (0,1,0),
        8: (1,1,0),
        16:(1,1,1)
        }

    def __init__(self,parent, **kwargs):
        """
        sets up a motor instance (for steppers in this case)
        
        name     : the name of this motor - used in notifications etc.
        parent   : the thing that owns this motor - must support method, setpin
        loglevel : bitwise notification control:
                    LOGLVLLIFE  : log lifecycle actions
                    LOGLVLSTATE : log state changes (direction, mode, enable, start / end sequences / phases
                    LOGLVLFAIL  : fails / errors
                    LOGLVLDETAIL: loads of stuff
        pins     : dict identifying (intergers) pins used to control this motor:
                     'enable':      the pin that enables the drive chip
                     'direction':   the pin that controls the step direction
                     'step':        the pin that tells the driver chip to step
                     'steplevels':  a tuple of pins that set the step level (3 pins - 5 settings for the A4988)
                                    works in conjunction with microstepset above which defines valid values
        maxval   : Currently this driver assumes the ultimate output is rotation, and maxval is the number of 
                    steps for 1 revolution, after which values wrap around
        fastdefaults: a bunch of values used for fast moving with ramp up / down defining starting speed,
                    maximum speed, ramp rate etc.
        """
        self.motmode='idle' # or 'fastwave', 'slowstep'
        self.parent=parent
        self.currentposition = 0 # current position of motor in finest resolution microsteps
        self.stepfactor = 0
        self.settings={}
        self.updatesettings(**kwargs)
        self.logmsg(level=mms.LOGLVLLIFE, action='motor control', status='started', message='')

    def updatesettings(self, name=None, loglevel=None, pins=None, maxval=None, fastdefaults=None, flipdir=None):
        changelog=[]
        if not name is None and self.settings.get('name',None) != name:
            changelog.append("motor name updated from %s to %s" % (self.settings.get('name','<none>'), name))
            self.mname = name
            self.settings['name'] = name
        if not loglevel is None and self.settings.get('loglevel',None) != loglevel:
            changelog.append("log level updated from %d to %d" % (self.settings.get('loglevel',0), loglevel))
            self.loglevel = loglevel
            self.settings['loglevel'] = loglevel
        if self.motmode == 'idle':
            if not maxval is None and self.settings.get('maxval',None) != maxval:
                changelog.append("maxval updated from %d to %d" % (self.settings.get('maxval',0), maxval))
                self.maxval = maxval
                self.settings['maxval'] = maxval
                self.currentposition = self.currentposition % self.maxval
            if not flipdir is None and self.settings.get('flipdir', None) != flipdir:
                self.settings['flipdir'] = flipdir
                changelog.append("motor direction is %s" % ('reversed' if flipdir else 'normal'))
            if not pins is None:
                self.settings['pins'] = pins
                self.steppin=pins['step']
                self.steppinmask=1<<self.steppin
                self.steplevelpins = tuple(int(x) for x in pins['steplevels'])
                self.allpins = self.steplevelpins + (pins['enable'], pins['direction'], pins['step'])
                self._setpinmodes()
                self._enabledrive(False)
            if not fastdefaults is None:
                self.fastdefaults=fastdefaults
        else:
            self.notifyFail(action='updatesettings', status='failed', reason='wrong mode'
                , message="cannot update some settings in mode  >%s<." % str(self.motmode))
        print(changelog)

    def notifyFail(self, **kwargs):
        self.logmsg(level=mms.LOGLVLFAIL, **kwargs)

    def incrementpos(self, change):
        self.currentposition = (self.currentposition + change) % self.maxval

    def getpos(self):
        """
        returns the position in microsteps (an integer)
        """
        if self.motmode == 'fastwave':
            return (self.currentposition + self.lastpulseoffset) % self.maxval
        else:
            return self.currentposition

    def setpos(self, degrees):
        if self.motmode == 'fastwave':
            self.notifyFail(action='setpos', status='failed', reason='wrong mode', message="cannot setposition while in fastwave")
            return
        elif self.motmode in ('slowstep', 'idle'):
            pass
        else:
            self.notifyFail(action='setpos', status='failed', reason='wrong mode'
                , message="cannot setpos - from unknown mode >%s<." % str(self.motmode))
            return
        self.currentposition = self.degtosteps(degrees)
        self.logmsg(level=mms.LOGLVLSTATE, action='setpos', status='complete', message='%3.2f' % self.degpos())

    def stepstodeg(self,steppos):
        """
        converts microsteps to degrees
        """
        return steppos/self.maxval*360

    def degtosteps(self, degpos):
        """
        converts degrees to microsteps
        """
        return round(degpos/360*self.maxval)

    def degpos(self):
        """
        returns the current motor position in degrees
        """
        return self.stepstodeg(self.getpos())

    def getState(self):
        """
        returns a dict with the current state
        """
        tpos = self.getpos()
        return {'steppos':tpos, 'degpos': self.stepstodeg(tpos), 'action':self.motmode
            , 'forward':(self.stepfactor > 0), 'microstep':abs(self.stepfactor)}

###################################################################################################
# These functions handle slow stepping

    def slowmove(self, steprate, totalsteps, pulseontime, forward, warp):
        """
        Initiates steady movement at steprate steps per second until totalsteps is reached (or forever if totalsteps is -1).
        
        steprate:    number of microsteps per second to issue, the actual ticks per second issued will be steprate*warp/16.
        totalsteps:  number of smallest sized steps to move, -1 means run forever. Actual steps issued will be int(totalsteps*warp/16)
        pulseontime: microseconds the on pulse will last
        forward:     boolean to set forward / reverse motor direction.
        warp:        microstep setting to use, must match one of the keys in microstepset
        
        In this mode the step rate can be fine tuned using the corrections interface (TBD)
        """
        if self.motmode == 'fastwave':
            self.notifyFail(action='slowmove', status='failed', reason='wrong mode', message="cannot slow move - already in fastwave")
            return
        elif self.motmode == 'slowstep':
            self.notifyFail(action='slowmove', status='failed', reason='wrong mode', message="cannot start slow move - already active")
            return
        elif self.motmode == 'idle':
            pass
        else:
            self.notifyFail(action='slowmove', status='failed', reason='wrong mode'
                , message="cannot slowmove - from unknown mode >%s<." % str(self.motmode))
            return
        if not self._set_ms_mode_dir(warp, forward):
            self.notifyFail(action='slowmove', status='failed', reason='bad parameter', message="invalid warp factor provided %s" % str(warp))
            return
        if totalsteps > 0:
            self.slowstepsleft = int(totalsteps * warp / (max(self.microstepset.keys())))
        else:
            self.slowstepsleft = -1
        self.slowstepfullrate = steprate
        self.slowticktime = 1 / (self.slowstepfullrate * warp / 16)
        print("tick tim is ", self.slowticktime)
        self.slowtickdue = time.time()
        self.slowpulsetime = pulseontime
        self._enabledrive(True)
        self.motmode = 'slowstep'
        self.logmsg(level=mms.LOGLVLSTATE, action='slowmove', status='started'
                    , message="")
        self.slowmovetick()

    def slowmovetick(self):
        if self.motmode != 'slowstep':
            return
        self.parent.pio.gpio_trigger(self.steppin,self.slowpulsetime,0)
        self.incrementpos(self.stepfactor)
        if self.slowstepsleft > 0:
            self.slowstepsleft -= 1
            if self.slowstepsleft == 0:
                self.slowmovestop('slowmove tick count reached')
                return
        self.slowtickdue = self.slowtickdue + self.slowticktime
        self.parent.setpollat(self.slowtickdue,self.slowmovetick)

    def slowmovestop(self, msg):
        self.motmode = 'idle'
        self._enabledrive(False)
        self.logmsg(level=mms.LOGLVLSTATE, action='slowmove', status='complete'
            , message=msg)

###################################################################################################
# These functions handle fast (wave based) stepping

    def pulser(self, startsr, maxsr, overreach, ramp, totalsteps, pulseontime, forward, warp):
        """
        A generator that returns pulse info to ramp up from initial to final step rate, then maintain
        constant speed, and finally ramp back down.
        
        It returns a tuple with the bits to waggle on now, bits to waggle off now and the delay until
        the next pulse is due.
    
        The code will abort the ramp up and switch directly to ramp down if it finds it has reached the 1/2 way
        point during ramp up.
        
        startsr     : initial step rate in steps per second 
                        (this and maxsr are the effective full step rate - for warp other than 1)
        maxsr       : maximum step rate in steps per second
        overreach   : asymptotic approach to beyond final speed by this amount so we arrive at 
                      the maximum step rate in finite time (1.1 is good)
        ramp        : factor used to control the rate of ramp
        totalsteps  : total number of steps to take (in smallest microsteps - steps to issue adjust by warp factor)
        pulseontime : the number of microseconds to hold the pulse on
        pulsebits   : the bits which will be turned on and off for each pulse
        forward     : True for motor to go forwards
        warp        : stepping mode - 1,2,4,8 or 16 ( see microstepset above)
        """
        if self.motmode == 'fastwave':
            self.notifyFail(action='pulsegen', status='failed', message="cannot do fast pulses - already active")
            return None
        elif self.motmode == 'slowstep':
            self.slowmovestop('slowstep changing to fastwave')
        elif self.motmode == 'idle':
            pass
        else:
            self.notifyFail(action='pulsegen', status='failed', message="cannot do fast pulses - from unknown mode >%s<." % str(self.motmode))
            return None
        assert maxsr > startsr
        if not self._set_ms_mode_dir(warp, forward):
            self.notifyFail(action='pulsegen', status='failed', reason='bad parameter', message="invalid warp factor provided %s" % str(warp))
            return None
        dividestepsby = max(self.microstepset.keys())/warp
        totalsteps = int(totalsteps / dividestepsby)
        startsr = startsr / dividestepsby
        maxsr = maxsr / dividestepsby
        self.lastpulseoffset = 0
        self.motmode ='fastwave'
        self._enabledrive(True)
        self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='started'
                , message="mode now %s. %d microticks, microstepping 1/%d" % (self.motmode, totalsteps, warp))
        tickunit = 1000000                  # the underlying time is microseconds - this converts seconds to microseconds
        target   = maxsr*overreach          # asymptotically approach this step rate so we actually intercept the maximum rate
        trange   = (target-startsr)*ramp    # pre-calculate the range of rates we will use
        elapsed  = ramp                     # start at ramp for easy calculation
        ramping  = True
        flipover = totalsteps/2             # check if we reach this step count during ramp up and switch direct to ramp down
        self.pulseAbort = None
        yield self.steppinmask, 0, 0
        yield 0, self.steppinmask, pulseontime
        self.gensteps = 1                   # maintain count of steps issued
        while ramping and self.pulseAbort is None:
            tps=target + trange / -elapsed
            if tps > maxsr:
                ramping = False
                tps = maxsr
            ticksec = 1 / tps
            elapsed += ticksec
            yield self.steppinmask, 0, int(ticksec*tickunit)-pulseontime
            yield 0, self.steppinmask, pulseontime
            self.gensteps += 1
            if self.gensteps >= flipover:
                ramping = False

        if self.pulseAbort is None:
            maxcount = totalsteps-(self.gensteps*2)     # ticks to do at full speed (-ve if we are already half way)
            self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='progress'
                    , message="ramp up complete at %5.2f in %4.2f ticks with %5.2f to do at full speed" % (
                        elapsed-ramp, self.gensteps/warp, (0 if maxcount < 0 else maxcount/warp)))
            tick = int(tickunit / maxsr)-pulseontime
            ticksec = 1/maxsr 
        else:
            if self.pulseAbort == 'softstop':
                maxcount = -1     # skip straight to ramp down
                self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='progress', message="ramp up aborted at %5.2f in %4.2f ticks." % (
                        elapsed-ramp, self.gensteps/warp))
                totalsteps = self.gensteps *2 # update total ticks to twice ticks spent ramping up so far
            else:
                self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='complete', message="ramp up crash stop at %5.2f in %d ticks." % (
                        elapsed-ramp, self.gensteps/warp))
                return
        ramptime = elapsed
        rampticks = self.gensteps
        while maxcount > 0 and self.pulseAbort is None:
            maxcount -= 1
            yield self.steppinmask, 0, tick
            yield 0, self.steppinmask, pulseontime
            elapsed += ticksec
            self.gensteps += 1
        if not self.pulseAbort is None:
            if self.pulseAbort == 'softstop':
                self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='progress', message="full speed aborted at %5.2f in %5.2f ticks." % (
                        elapsed-ramp, self.gensteps/warp ))
                totalsteps = self.gensteps+rampticks
            else:
                self.motmode='idle'
                self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='complete', message="ramp up crash stop at %5.2f in %5.2f ticks." % (
                    elapsed-ramp, self.gensteps/warp ))
                return

        timeleft = ramptime
        self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='progress', message="starting ramp down")
        while self.gensteps < totalsteps and (self.pulseAbort is None or self.pulseAbort == 'softstop'):
            tps = target + trange / -timeleft
            if tps < startsr:
                tps = startsr
            if tps > maxsr:
                tps = maxsr
            ticksec = 1 / tps
            timeleft -= ticksec
            yield self.steppinmask, 0, int(ticksec*tickunit)-pulseontime
            yield 0, self.steppinmask, pulseontime
            self.gensteps += 1

        if self.pulseAbort == 'softstop':
            self.logmsg(level=mms.LOGLVLSTATE, action='softstop', status='complete', message="")
        self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='complete'
            , message="pulse train complete in %5.2f ticks" % (self.gensteps/warp))

    def pulsereached(self,faststeps):
        """
        called from parent as it detects each wave finishing, offset is updated to the position at the end of wavew
        """
        self.lastpulseoffset = faststeps * self.stepfactor

    def pulsedone(self, faststeps):
        """
        called from parent when a motor's last step from a series of waves is reached. 
        Note this can happen at totally different times for different motors.
        """
        self.incrementpos(faststeps * self.stepfactor)
        self.lastpulseoffset = 0
        self.motmode='idle'
        self.logmsg(level=mms.LOGLVLSTATE, action='pulsegen', status='complete'
            , message="now at %d" % self.stepstodeg(self.getpos()))

    def rampPulseCalc(self, startsr, maxsr, overreach, ramp, totalsteps, pulseontime, forward, warp):
        """
        derived from pulser, this calculates the time and tick count needed to ramp up
        """
        assert maxsr > startsr
        if not self._set_ms_mode_dir(warp, forward):
            self.notifyFail(action='pulsegen', status='failed', reason='bad parameter', message="invalid warp factor provided %s" % str(warp))
            return None
        dividestepsby = max(self.microstepset.keys())/warp
        ticksteps = int(totalsteps / dividestepsby)
        startsr = startsr / dividestepsby
        maxsr = maxsr / dividestepsby
        tickunit = 1000000                  # the underlying time is microseconds - this converts seconds to microseconds
        target   = maxsr*overreach          # asymptotically approach this step rate so we actually intercept the maximum rate
        trange   = (target-startsr)*ramp    # pre-calculate the range of rates we will use
        elapsed  = ramp                     # start at ramp for easy calculation
        ramping  = True
        flipover = ticksteps/2             # check if we reach this step count during ramp up and switch direct to ramp down
        ramptime = pulseontime
        stepcount = 1                       # maintain count of steps issued
        while ramping:
            tps=target + trange / -elapsed
            if tps > maxsr:
                ramping = False
                tps = maxsr
            ticksec = 1 / tps
            elapsed += ticksec
            ramptime += int(ticksec*tickunit)
            stepcount += 1
            if stepcount >= flipover:
                ramping = False
        return stepcount, ramptime/1000000, (stepcount >= flipover)

    def constPulseCalc(self, totalsteps, maxsr):
        tickunit = 1000000
        tick = int(tickunit / maxsr)
        return (tick * totalsteps)/1000000

    def timetomove(self, target=None, angle=None):
        """
        calculates the 2 times to move (cw, ccw) either from current position to target, or for the angle given
        if target is None.

        Returns a pair of settings, first is faster, second is slower.
        Settings is a tuple of:
            move time
            signed move in degrees
            (unsigned) step count
            
        """
        if target is None:
            fwang=angle % 360
        else:
            fwang=(target-self.degpos()) % 360
        revang = 360-fwang
        msets = self.fastdefaults.copy()
        msets['forward'] = True
        fwdsteps = self.degtosteps(fwang)
        msets['totalsteps'] = fwdsteps
        fcount, elapsed, nomiddle = self.rampPulseCalc(**msets)
        if nomiddle:
            ftime=elapsed*2
        else:
            ftime=elapsed*2 + self.constPulseCalc(totalsteps=fwdsteps-2*fcount, maxsr=msets['maxsr'])
        msets['forward'] = False
        revsteps = self.degtosteps(revang)
        msets['totalsteps'] = revsteps
        rcount, elapsed, nomiddle = self.rampPulseCalc(**msets)
        if nomiddle:
            rtime=elapsed*2
        else:
            rtime=elapsed*2 + self.constPulseCalc(totalsteps=revsteps-2*rcount, maxsr=msets['maxsr'])
        if ftime < rtime:
            return (ftime, fwang, fwdsteps), (rtime,  -revang, revsteps)
        else:
            return (rtime, -revang, revsteps), (ftime, fwang, fwdsteps)

###################################################################################################
# These functions handle the various stop / abort functions

    def softstop(self):
        """
        smoothly shuts down from any mode
        
        returns True if motor is stopped on return, False means there will be a run-down delay
        """
        if self.motmode == 'fastwave':
            self.pulseAbort = 'softstop' # set this so the pulser will cleanly close down
            self.logmsg(level=mms.LOGLVLSTATE, action='softstop', status='progress', message="pulse generator set to ramp down")
            return False
        elif self.motmode == 'slowstep':
            self.slowmovestop('softstop interrupts slowstep')
        elif self.motmode == 'idle':
            self._enabledrive(False) # just in case....
            self.logmsg(level=mms.LOGLVLSTATE, action='softstop', status='complete', message="from idle - nothing to stop")
        else:
            self.logmsg(level=mms.LOGLVLSTATE, action='softstop', status='complete', message="from unknown mode >%s<." % str(self.motmode))
        return True

    def hardstop(self):
        """
        stops quickly in a controlled manner - if in fastwave mode the currently issued waves are completed
        """
        if self.motmode == 'fastwave':
            self.pulseAbort = 'hardstop'
            self.logmsg(level=mms.LOGLVLSTATE, action='hardstop', status='complete', message="pulse generator crashout")
        elif self.motmode == 'slowstep':
            self.slowmovestop('hardstop interrupts slowstep')
        elif self.motmode == 'idle':
            self._enabledrive(False) # just in case....
            self.logmsg(level=mms.LOGLVLSTATE, action='hardstop', status='complete', message="from idle - nothing to stop")
        else:
            self.logmsg(level=mms.LOGLVLSTATE, action='hardstop', status='complete', message="from unknown mode >%s<." % str(self.motmode))
        return True

    def crashstop(self):
        """
        stops immediately - any wave in progress will be canceled before this is called.
        """
        if self.motmode == 'fastwave':
            self.pulseAbort = 'crash'
            self.logmsg(level=mms.LOGLVLSTATE, action='crashstop', status='complete', message="pulse generator crashout")
        elif self.motmode == 'slowstep':
            self.slowmovestop('hardstop interrupts slowstep')
        elif self.motmode == 'idle':
            self._enabledrive(False) # just in case....
            self.logmsg(level=mms.LOGLVLSTATE, action='crashstop', status='complete', message="from idle - nothing to stop")
        else:
            self.logmsg(level=mms.LOGLVLSTATE, action='crashstop', status='complete', message="from unknown mode >%s<." % str(self.motmode))
        return True

###################################################################################################
# These functions handle the various functions to set pins for enable, direction and microstep mode.

    def _setpinmodes(self):
        for p in self.allpins:
            self.parent.pio.set_mode(p,pigpio.OUTPUT)
        self.logmsg(level=mms.LOGLVLSTATE, action='pin modes set', status='complete', message='output')

    def _enabledrive(self, enable):
        self.parent.setpin(self.settings['pins']['enable'],0 if enable else 1)
        self.logmsg(level=mms.LOGLVLSTATE, action='enable', status='complete', message='enable' if enable else 'disable')
        return True

    def _set_ms_mode_dir(self, microsteps, forward):
        if microsteps in self.microstepset:
            mslevels = self.microstepset[microsteps]
            for slentry in range(len(mslevels)):
                self.parent.setpin(self.steplevelpins[slentry], mslevels[slentry])
            if self.settings['flipdir']:
                pinval = 1 if forward else 0
            else:
                pinval = 0 if forward else 1
            self.parent.setpin(self.settings['pins']['direction'],pinval)
            self.logmsg(level=mms.LOGLVLSTATE, action='micro step level/direction ', status='complete'
                , message="warp 1/%d %s" % (microsteps, 'forward' if forward else 'reverse'))
            self.stepfactor = int(max(self.microstepset.keys())/microsteps)
            if not forward:
                self.stepfactor = -self.stepfactor
            return True
        else:
            self.logmsg(level=mms.LOGLVLFAIL, action='micro step level', status='failed'
                    , message="cannot do warp factor %d for motor %d" % microsteps)
            return False

    def logmsg(self, **kwargs):
        self.parent.logmsg(agent=(self.mname, ), **kwargs)

class onewave(mms.timerSelector):
    """
    class to run stepper motors using pigpio waves. This runs as a process, controlled by messages from a primary agent
    over a socket connection.
    """
    def __init__(self, connectTo,  wavepollinterval, motors, **kwargs):
        """
        see default mail code at end of module
        """
        self.mode='startup' # or 'idle' or 'waving' - note that even if single motors are busy with other than a wave, this will still be 'idle'.
        super().__init__(**kwargs)
        self.funclist=('goto',)
        self.starttime = time.time()
        self.pio = pigpio.pi()
        self.motors={}
        self.hostConn = mms.autoSocket(framework=self, connectTo=connectTo, targetob=self
                    , name='mount driver', reportState=self.hostlinkChanged)
        self.motors={mdef['name']: motor(parent=self, **mdef) for mdef in motors}
        self.wavepollinterval = wavepollinterval
        self.maxpulsesperwave = self.pio.wave_get_max_pulses()
        self.maxmsperwave = self.pio.wave_get_max_micros()
        self.maxdblocksperwave = self.pio.wave_get_max_cbs()
        self.pwpulselim=self.maxpulsesperwave/3
        self.pwmslim = self.maxmsperwave/3
        self.pwcblim = self.maxdblocksperwave/8.3
        startmsg = "wave agent: wave limits, pulses per wave: %d, seconds per wave: %5f, control blocks per wave: %d" % (self.maxpulsesperwave
            , self.maxmsperwave/1000000, self.maxdblocksperwave)
        self.mode='idle'
        self.logmsg(action='run agent', status='started', level=mms.LOGLVLLIFE, message=startmsg)

    def hostlinkChanged(self,cname, cstate):
        self.logmsg(level=mms.LOGLVLLIFE,message="onewave.hostlinkChanged now reports " + cstate)

    def sendLogmsg(self, **kwargs):
        try:
            self.hostConn.runFunc('statusUpdate', **kwargs)
        except AttributeError: # at startup, this can be called before hostConn is setup....
            pass

    def logmsg(self, mode=None, **kwargs):
        super().logmsg(mode=self.mode if mode is None else mode, **kwargs)

    def report(self):
        mstrs = ('%s motor: %s at %3.2f' % (m.mname, m.motmode, m.degpos()) for m in self.motors.values())
        self.logmsg(action='report', status='complete', level=mms.LOGLVLINFO, message = ', '.join(mstrs))

    def _stopfromfast(self, stopfunc, stoptext):
        alldone = True
        for m in self.motors.values():
            mdone = stopfunc()
            alldone = mdone and alldone # beware the side effects and lazy evaluation"
        self.logmsg(action=stoptext, status='complete' if alldone else 'started', level=mms.LOGLVLSTATE, message='')
        if alldone:
            self.mode='idle'

    def softstop(self):
        alldone = True
        for m in self.motors.values():
            mdone = m.softstop()
            alldone = mdone and alldone # beware the side effects and lazy evaluation"
        self.logmsg(action='softstop', status='complete' if alldone else 'started', level=mms.LOGLVLSTATE, message='')
        if alldone:
            self.mode='idle'

    def hardstop(self):
        alldone = True
        for m in self.motors.values():
            mdone = m.hardstop()
            alldone = mdone and alldone # beware the side effects and lazy evaluation"
        self.logmsg(action='hardstop', status='complete' if alldone else 'started', level=mms.LOGLVLSTATE, message='')
        if alldone:
            self.mode='idle'

    def crashstop(self):
        self._stopfromfast(m.crashstop, 'crashstop')
        
    def slowRun(self, slowparams):
        """
        Give a tuple of slowparams, passes each set of params to the relevant motor.
        """
        for pset in slowparams:
            paramset = pset.copy()
            assert paramset['motor'] in self.motors
            mkey = paramset.pop('motor')
            self.motors[mkey].slowmove(**paramset)

    def wavepulsemaker(self, pulseparams):
        """
        Given tuple of pulseparams, creates a pulser for each set of params and then prepares to dynamically
        generate a succession of pigpio waves.
        
        By using generators and creating new waves on the fly, arbitrarily long and complex waves can be driven through pigpio, avoiding 
        the various limits on pulse counts and dma blocks imposed by both hardware and software.
        """
        if self.mode == 'waving':
            self.logmsg(action='waving', status='failed', level=mms.LOGLVLFAIL, message="wave pulses already active")
            return
        wavegens = []
        for pset in pulseparams:
            paramset = pset.copy()
            assert paramset['motor'] in self.motors
            mkey = paramset.pop('motor')
            gen = self.motors[mkey].pulser(**paramset)
            if gen == None:
                self.logmsg(action='waving', status='failed', level=mms.LOGLVLFAIL, message="failed to create step generator for motor %s." % mkey)
                return
            wavegens.append((gen, mkey))
        self.wpgen = self.multiPulse(wavegens)
        self.mode = 'waving'
        self.logmsg(action='waving', status='started', level=mms.LOGLVLSTATE, message="ready to start waves with %d motors" % len(wavegens))
        self.wavemaker()

    def wavemaker(self):
        """
        This implementation generates the first 2 waves and passes them to pigpio. It prepares (but does not create) the next wave.
        
        It uses simpleAgent timers to poll pigpio in separate methods and as a wave finishes, it deletes the finished wave, creates
        the prepared wave and appends it to the running waveset, and finally prepares the next one.

        """
        self.morewaves=(True,)
        self.pio.wave_clear()
        self.wavids = deque()
        self.wavecount = 0
        self.waveends = {}
        self.pendingwave=None
        self.wavenowaitcount = 0
        while self.morewaves[0]:
            self.morewaves = self.prepareWave()
            if len(self.wavids) < 2:
                if self._createWave():
                    return
            else:
                self.pendingwave = self.wavids.popleft()
                self.runafter(self.wavepollinterval, self.checkPending)
                return

        progmsg="all sent in initial set, running wave %d there are %d waves left." % (self.pio.wave_tx_at(), len(self.wavids))
        self.logmsg(action='waving', status='progress', level=mms.LOGLVLSTATE, message=progmsg)
        self.pendingwave = self.wavids.popleft()
        self.runafter(self.wavepollinterval*2, self.checkFinal)

    def _createWave(self):
        try:
            newwave=self.pio.wave_create()
        except pigpio.error as pe:
            newwave = None
            self.logmsg(action='waving', status='failed', level=mms.LOGLVLFAIL
                    , message="pigpio wave create failed: {0}".format(pe))
            return True
        self.wavids.append(newwave)
        self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        mposns = {m: (self.motors[m].gensteps,False if self.morewaves[1] is None else m in self.morewaves[1]) for m in self.activewavemotors}
        self.waveends[newwave] = mposns
        if True:
            mposnstrs = ((' %s(%s) at %4.3f' % (m, 'done' if pos[1] else 'more', pos[0]) for m, pos in mposns.items()))
            if self.pendingwave is None:
                self.logmsg(action='waving', status='progress', level=mms.LOGLVLDETAIL
                    , message="wave %d dispatched, %s" % (newwave, ', '.join(mposnstrs)))
            else:
                self.logmsg(action='waving', status='progress', level=mms.LOGLVLDETAIL
                    , message="deleted %d, dispatched %d, %s" % (self.pendingwave, newwave, ', '.join(mposnstrs)))
        return False

    def _deleteWave(self):
        self.pio.wave_delete(self.pendingwave)
        waveends = self.waveends.pop(self.pendingwave)
        for m,info in waveends.items():
            if info[1]:
                self.motors[m].pulsedone(info[0])
                self.activewavemotors = tuple(mid for mid in self.activewavemotors if mid != m)
            else:
                self.motors[m].pulsereached(info[0])

    def checkPending(self):
        wactive = self.pio.wave_tx_at()
        if wactive == self.pendingwave:
            self.runafter(self.wavepollinterval, self.checkPending)
            return
        self.nextwave()

    def nextwave(self):
        self._deleteWave()
        if self._createWave():
            return
        if self.morewaves[0]:
            self.morewaves = self.prepareWave()
            if not self.morewaves[1] is None:
                self.logmsg(action='waving', status='progress', level=mms.LOGLVLDETAIL
                , message="%s completed" % str(self.morewaves[1]))
            self.pendingwave = self.wavids.popleft()
            if self.pio.wave_tx_at() != self.pendingwave:
                self.wavenowaitcount += 1
                self.nextwave()
            else:
                self.runafter(self.wavepollinterval, self.checkPending)
        else:
            self.pendingwave = self.wavids.popleft()
            self.runafter(self.wavepollinterval*2, self.checkFinal)
            progmsg="all sent, running wave %d there are %d waves left." % (self.pio.wave_tx_at(), len(self.wavids))
            self.logmsg(action='waving', status='progress', level=mms.LOGLVLSTATE, message=progmsg)

    def checkFinal(self):
        if self.pio.wave_tx_at() == self.pendingwave:
            self.runafter(self.wavepollinterval*2, self.checkFinal)
            return
        self._deleteWave()
        self.logmsg(action='waving', status='progress', level=mms.LOGLVLDETAIL
                , message="wave %d deleted." % self.pendingwave)
        if len(self.wavids) > 0:
            self.pendingwave = self.wavids.popleft()
            self.runafter(.2, self.checkFinal)
            return
        self.allstop()
        self.logmsg(action='waving', status='complete', level=mms.LOGLVLSTATE
            , message="all done in %d waves with %d fast loop rounds" % (self.wavecount, self.wavenowaitcount))
        self.mode='idle'

    def prepareWave(self):
        """
        Uses the multiPulse generator to fetch more pulses and add them to pigpio until we reach threshold.
        
        returns a tuple:
            0: True if there is more to do
            1: dict with keys for any motor that has finished pulse train in this set
        """
        onewave=True
        wavetime=0
        self.wavecount += 1
        groupleft=25
        moretodo = True
        donedict = None
        while onewave:
            pulseparams = tuple(self.wpgen.__next__() for i in range(groupleft))
            if len(pulseparams) == 0:
                onewave = False
                moretodo = False
            else:
                self.pio.wave_add_generic((pgpulse(0,0,wavetime),) + tuple(pgpulse(*wp[0]) for wp in pulseparams if wp[0] is not None))
                for wp in pulseparams:
                    if not wp[0] is None:
                        wavetime += wp[0][2]
                    if not wp[1] is None:
                        if donedict is None:
                            donedict= {}
                        for mid in wp[1]:
                            donedict[mid]=1
                wpc=self.pio.wave_get_pulses()
                wcbc=self.pio.wave_get_cbs()
                wus=self.pio.wave_get_micros()
                if wpc > self.pwpulselim or wus > self.pwmslim or wcbc > self.pwcblim:
                    onewave=False
        return moretodo, donedict

    def allstop(self):
        self.pio.wave_tx_stop()
        for m in self.motors.values():
            m._enabledrive(False)

    def done(self):
        self.pio.stop()
        self.running = False
        self.logmsg(action='run agent', status='complete', level=mms.LOGLVLLIFE, message="closed")

    def setpin(self,pin,val):
        self.pio.write(pin,val)
        self.pio.write(pin,val)

    def multiPulse(self, multipulses):
        """
        A generator that merges a number of pulse generators into single ordered sequence.
        
        multipulses is a tuple of tuple about each pulse sequence as follows:
            0 : a pulser for this pulse sequence
            1 : the motor id for this pulse stream

        The generator returns a tuple with:
            0: a tuple with the bits to waggle on now, bits to waggle off now and the delay until the next pulse is due.
            1: a list of the motors which have now finished their pulse train - None if empty, which is usually the case 
             
        """
        nextpulses = []
        nextpulsedue = float('inf')
        for pg in multipulses:
            try:
                pon, poff, pd = pg[0].__next__()
                if pd < nextpulsedue:
                    nextpulsedue = pd
                nextpulses.append([pd, pon, poff, pg])
            except StopIteration:
                pass
        if len(nextpulses) == 0:
            self.logmsg(action='waving', status='failed', level=mms.LOGLVLFAIL, message="multiPulse has nothing to do!")
            raise StopIteration
        active = True
        self.activewavemotors = tuple((ph[3][1] for ph in nextpulses))
        while active:
            ponbits = int(0)
            poffbits= int(0)
            thispulse = nextpulsedue
            nextpulsedue = float('inf')
            donepulsers = None
            active = False
            for pg in nextpulses:
                pg[0] -= thispulse
                if pg[0] <= 0:
                    ponbits |= pg[1]
                    poffbits |= pg[2]
                    try:
                        pon, poff, pd = pg[3][0].__next__()
                        if pd < nextpulsedue:
                            nextpulsedue = pd
                        active=True
                        pg[0] = pd
                        pg[1] = pon
                        pg[2] = poff
                    except StopIteration:
                        if donepulsers is None:
                            donepulsers = [pg,]
                        else:
                            donepulsers.append(pg)
                else:
                    active=True
                    if pg[0] < nextpulsedue:
                        nextpulsedue = pg[0]
            if not donepulsers is None:
                for pg in donepulsers:
                    nextpulses.remove(pg)
            if nextpulsedue == float('inf'):
                active = False
                yield None, tuple(pg[3][1] for pg in donepulsers)
            else:
                yield (ponbits, poffbits, nextpulsedue), None if donepulsers is None else tuple(pg[3][1] for pg in donepulsers)

LOGLVLFAIL=mms.LOGLVLFAIL
LOGLVLSTATE=mms.LOGLVLSTATE
LOGLVLLIFE=mms.LOGLVLLIFE
LOGLVLINFO=mms.LOGLVLINFO
LOGLVLDETAIL=mms.LOGLVLDETAIL
LOGLVLSCHED=mms.LOGLVLSCHED
