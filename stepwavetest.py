#!/usr/bin/python3
#these imports are for the low level code
import pigpio
from pigpio import pulse as pgpulse
import time
from collections import deque
import heapq
#these imports are for the thready / ui parts
import queue
from threading import Thread
import os, sys, traceback

class simpleAgent():
    """
    A very basic class that provides the core an event driven app which can be adapted to run in different environments without affecting the
    lower level code.
    
    This version uses threads and picks up method requests from a queue and passes them to the appropriate method, as well as allowing
    for timer functionality.
    
    Items on the input queue must be tuples, the first item is the name of a method to be called, the second is the keyword args (a dict)
    of the method's parameters.
    
    The app can send requests back the other way using the same mechanism. At this level agents are symmetric with no concept of master / slave or
    primary / secondary.
    
    The app can also nominate a 'killhandler' which is called on keyboard interrupt (or kill signal one day), to allow a graceful close down.
    
    All these methods (method call, timeout handler and kill handler) are run in a simple wrapper which catches and reports on any exceptions.
    
    Typically used by creating two subclasses, a primary and secondary, and a pair of queues, then instantiating the subclasses, passing
    in the pair of queues swapped over between the two.
    """
    def __init__(self, inq, outq):
        """
        runs up a thread and starts it.
        """
        self.inq = inq
        self.outq = outq
        self.timervals=[]
        self.timers={}
        self.killhandler=None
        self.hthread=Thread(target=self.agentRun)
        self.running = True
        self.hthread.start()
        self.crashabortcount=5

    def agentRun(self):
        """
        This is the main loop for the agent. It waits for incoming messages to process, optionally with a timeout which will call
        any other code (such as a polling function).
        """
        self.timeTotal = 0
        self.cpuTotal = 0
        while self.running:
            try:
                try:
                    try:
                        nexttimerdue = self.timervals[0]    
                    except IndexError:
                        nexttimerdue = None
                    if nexttimerdue is None:
                        inMsgMeth, inMsgKwargs = self.inq.get()
                    else:
                        inMsgMeth, inMsgKwargs = self.inq.get(block=True, timeout=nexttimerdue-time.time())
                except queue.Empty:
                    nexttimerdue = heapq.heappop(self.timervals)
                    self.wrappedRunMethod(self.timers.pop(nexttimerdue),{})
                    inMsgMeth = None
                if not inMsgMeth is None:
                    try:
                        cmeth = getattr(self,inMsgMeth)
                    except:
                        cmeth = None
                        print("unable to find method %s in object" % str(inMsgMeth))
                    if not cmeth is None:
                        self.wrappedRunMethod(cmeth, inMsgKwargs)
            except KeyboardInterrupt:
                if not self.killhandler is None:
                    self.wrappedRunMethod(self.killhandler,{})
                else:
                    self.running=False
        print('agent exits')

    def setpollat(self, timedue, thandler):
        """
        where maintaining the tick (even if individual ticks are delayed) is important, use this method
        """
        assert not timedue in self.timers
        self.timers[timedue] = thandler
        heapq.heappush(self.timervals, timedue)

    def setpollfrom(self, delay, thandler):
        """
        where the poll time is not critical, we'll just wait a bit from now
        """
        self.setpollat(time.time()+delay, thandler)

    def runit(self, meth, **kwargs):
        """
        Passes a request to the 'other' agent to run the named method, with the given (keyword) parameters.
        """
        self.outq.put((meth, kwargs))

    def wrappedRunMethod(self, meth, kwargs):
        msgStartTime = time.time()
        msgCPUstart = time.process_time()
        try:
            meth(**kwargs)
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            print("wrapped call got exception >%s<" % type(e))
            try:
                print("wrappedCall: %s exception in called code - \n%s\n%s" %
                    (str(exc_type), str(exc_value), ''.join(traceback.format_tb(exc_traceback))))
            except:
                pass
            if self.crashabortcount > 0:
                self.crashabortcount -= 1
                if self.crashabortcount == 0:
                    if not self.killhandler is None:
                        self.wrappedRunMethod(self.killhandler,{})
                    else:
                        self.running=False

        self.timeTotal += time.time() - msgStartTime
        self.cpuTotal += time.process_time() - msgCPUstart

class motor():
    """
    knows about and drives a single motor, with help from the parent which has the pigpio interface.
    
    It has a few modes which limit possible actions:
        idle     : nothing happening and drive chip is disabled
        fastwave : a fast move is in progress, passing steps to the parent through the pulser generator
        slowstep : slow stepping in progress, potentially with feedback correction
    """
    
    """
    default mapping of microstep levels to enable / disable on the mspins
    """
    microstepset={
        1: (0,0,0),
        2: (1,0,0),
        4: (0,1,0),
        8: (1,1,0),
        16:(1,1,1)
        }

    def __init__(self, name, parent, loglevel, pins):
        """
        sets up a motor instance (for steppers in this case)
        
        name     : the name of this motor - used in notifications etc.
        parent   : the thing that owns this motor - must support method, subAgentStateUpdate, setpin
        loglevel : bitwise notification control:
                    1: log create, destroy actions
                    2: log state changes (direction, mode, enable, start / end sequences / phases
                    4: fails / errors
        """
        self.enablepin=pins['enable']
        self.dirpin=pins['direction']
        self.steppin=pins['step']
        self.steppinmask=1<<self.steppin
        self.steplevelpins = tuple(int(x) for x in pins['steplevels'])
        self.allpins = self.steplevelpins + (self.enablepin, self.dirpin, self.steppin)
        self.mname=name
        self.parent=parent
        self.loglevel = loglevel
        self.mode='idle'
        self.notifyStateUpdate(1, 'setup', 'started', '')
        self.setpinmodes()
        self._enabledrive(False)

    def notifyStateUpdate(self, level, action, status, message):#agent, action, status, mode, message
        if self.loglevel & level:
            self.parent.subAgentStateUpdate(agent=(self.mname,), action=action, status=status, mode=self.mode, message=message)

    def setpinmodes(self):
        for p in self.allpins:
            self.parent.pio.set_mode(p,pigpio.OUTPUT)
        self.notifyStateUpdate(2, 'pins mode set', 'complete', 'output')

    def _enabledrive(self, enable):
        self.parent.setpin(self.enablepin,0 if enable else 1)
        self.notifyStateUpdate(2, 'enable', 'complete', 'enable' if enable else 'disable')
        return True
        
    def _setdir(self, fwd):
        pval = 0 if fwd else 1
        self.notifyStateUpdate(2, 'direction', 'complete', 'forward' if fwd else 'reverse')
        self.parent.setpin(self.dirpin,pval)
        return True

    def _set_ms_mode(self, microsteps):
        if microsteps in self.microstepset:
            mslevels = self.microstepset[microsteps]
            for slentry in range(len(mslevels)):
                self.parent.setpin(self.steplevelpins[slentry], mslevels[slentry])
            self.notifyStateUpdate(2, 'micro step level', 'complete', "1/%d" % microsteps)
            return True
        else:
            self.notifyStateUpdate(4, 'micro step level', 'failed', "cannot do warp factor %d for motor %d" % microsteps)
            return False

    def pulser(self, startsr, maxsr, overreach, ramp, totalsteps, pulseontime):
        """
        A generator that returns pulse info to ramp up from initial to final step rate, then maintain
        constant speed, and finally ramp back down.
        
        It returns a tuple with the bits to waggle on now, bits to waggle off now and the delay until the next pulse is due.
    
        The code will abort the ramp up and switch directly to ramp down if it finds it has reached the 1/2 way
        point during ramp up.
        
        This code knows nothing about direction or stepping mode - that is all handled elsewhere
        
        startsr     : initial step rate in steps per second
        maxsr       : maximum step rate in steps per second
        overreach   : asymptotic approach to beyond final speed by this amount so we arrive at 
                      the maximum step rate in finite time (1.1 is good)
        ramp        : factor used to control the rate of ramp
        totalsteps  : total number of steps to take
        pulseontime : the number of microseconds to hold the pulse on
        """
        if self.mode == 'fastwave':
            self.notifyStateUpdate(4, 'pulsegen', 'failed', "cannot do fast pulses - already active")
            return None
        elif self.mode == 'slowstep':
            #cancel slow step
            pass
        elif self.mode == 'idle':
            pass
        else:
            self.notifyStateUpdate(4, 'pulsegen', 'failed', "cannot do fast pulses - from unknown mode >%s<." % str(self.mode))
            return None
        self.mode == 'fastwave'
        tickunit = 1000000                  # the underlying time is microseconds - this converts seconds to microseconds
        target   = maxsr*overreach          # asymptotically approach this step rate so we actually intercept the maximum rate
        trange   = (target-startsr)*ramp    # pre-calculate the range of rates we will use
        elapsed  = ramp                     # start at ramp for easy calculation
        ramping  = True
        flipover = totalsteps/2             # check if we reach this step count during ramp up and switch direct to ramp down
        tot_ticks= 0
        self.pulseAbort = None
        yield self.steppinmask, 0, 0
        yield 0, self.steppinmask, pulseontime
        while ramping and self.pulseAbort is None:
            tps=target + trange / -elapsed
            if tps > maxsr:
                ramping = False
                tps = maxsr
            ticksec = 1 / tps
            elapsed += ticksec
            tot_ticks += 1
            if tot_ticks >= flipover:
                ramping = False
            yield self.steppinmask, 0, int(ticksec*tickunit)-pulseontime
            yield 0, self.steppinmask, pulseontime

        if self.pulseAbort is None:
            maxcount = totalsteps-(tot_ticks*2)     # ticks to do at full speed (-ve if we are already half way)
            self.notifyStateUpdate(2, 'pulsegen', 'progress', "ramp up complete at %5.2f in %d ticks with %d to do at full speed" % (
                    elapsed, tot_ticks, (0 if maxcount < 0 else maxcount)))
            tick = int(tickunit / maxsr)-pulseontime
        else:
            if self.pulseAbort == 'softstop':
                maxcount = -1     # skip straight to ramp down
                self.notifyStateUpdate(2, 'pulsegen', 'progress', "ramp up aborted at %5.2f in %d ticks." % (
                        elapsed, tot_ticks, ))
                totalsteps = tot_ticks *2 # update total ticks to twice ticks spent ramping up so far
            else:
                self.mode='idle'
                return
        ramptime = elapsed
        rampticks = tot_ticks
        while maxcount > 0 and self.pulseAbort is None:
            maxcount -= 1
            tot_ticks += 1
            yield self.steppinmask, 0, tick
            yield 0, self.steppinmask, pulseontime
        if not self.pulseAbort is None:
            if self.pulseAbort == 'softstop':
                self.notifyStateUpdate(2, 'pulsegen', 'progress', "full speed aborted at %5.2f in %d ticks." % (
                        elapsed, tot_ticks, ))
                totalsteps = tot_ticks+rampticks
            else:
                self.mode='idle'
                return
        
        timeleft = ramptime
        self.notifyStateUpdate(2, 'pulsegen', 'progress', "starting ramp down")
        while tot_ticks < totalsteps and (self.pulseAbort is None or self.pulseAbort == 'softstop'):
            tps = target + trange / -timeleft
            if tps < startsr:
                tps = startsr
            if tps > maxsr:
                tps = maxsr
            ticksec = 1 / tps
            timeleft -= ticksec
            tot_ticks += 1
            yield self.steppinmask, 0, int(ticksec*tickunit)-pulseontime
            yield 0, self.steppinmask, pulseontime

        self.mode='idle'
        self.notifyStateUpdate(2, 'pulsegen', 'complete', "pulse train complete in %d ticks" % tot_ticks)

class onewave(simpleAgent):
    """
    testing class to run stepper motor using pigpio waves. This runs as a secondary agent, controlled by messages from a primary agent.
    """
    def __init__(self, **kwargs):
        self.mlist = kwargs.pop('motors')
        super().__init__(**kwargs)

    def agentRun(self):
        self.pio = pigpio.pi()
        self.motors={mdef['name']: motor(parent=self, **mdef) for mdef in self.mlist}
        self.mlist = None
        self.activeaction=None
        self.killhandler = self.done
        self.maxpulsesperwave = self.pio.wave_get_max_pulses()
        self.maxmsperwave = self.pio.wave_get_max_micros()
        self.maxdblocksperwave = self.pio.wave_get_max_cbs()
        self.pwpulselim=self.maxpulsesperwave/3
        self.pwmslim = self.maxmsperwave/3
        self.pwcblim = self.maxdblocksperwave/8.3
        startmsg = "wave agent: wave limits, pulses per wave: %d, seconds per wave: %5f, control blocks per wave: %d" % (self.maxpulsesperwave
            , self.maxmsperwave/1000000, self.maxdblocksperwave)
        self.sendStateUpdate(action='run agent', status='started', message=startmsg)
        self.logwave = False
        self.setupreport = False
        super().agentRun()

    def sendStateUpdate(self, action, status, message):
        """
        a convenience method to report status updates back to the primary agent. See statusUpdate method in class clif.
        """
        self.runit('statusUpdate', agent=('stepper control',), action = action, status = status, message = message)

    def subAgentStateUpdate(self, agent, action, status, mode, message):
        self.runit('statusUpdate', agent=('stepper control',)+agent, action = action, status = status, message = message)

    def softstop(self):
        if self.activeaction != None:
            for m in self.motors.values():
                m.pulseAbort = 'softstop'
            self.sendStateUpdate(action='softstop', status='started', message='')

    def crashstop(self):
        if self.activeaction != None:
            for m in self.motors.values():
                self.pulseAbort = 'crash'
            self.sendStateUpdate(action='crashstop', status='started', message='')
       
    def wavepulsemaker(self, pulseparams):
        """
        Given tuple of pulseparams, creates a pulser for each set of params and then prepares to dynamically
        generate a succession of pigpio waves.
        
        By using generators and creating new waves on the fly, arbitrarily long and complex waves can be driven through pigpio, avoiding 
        the various limits on pulse counts and dma blocks imposed by both hardware and software.
        
        This implementation generates the first 2 waves and passes them to pigpio. It prepares (but does not create) the next wave.
        
        It uses simpleAgent timers to poll pigpio in sepatate methods and as a wave finishes, it deletes the finished wave, creates
        the prepared wave and appends it to the running waveset, and finally prepares the next one.
        """
        self.activeaction = 'wavepulsemaker'
        wavegens = []
        for pset in pulseparams:
            paramset = pset.copy()
            assert paramset['motor'] in self.motors
            warpfact = int(paramset.pop('warp', 2))
            assert warpfact >= 1
            if warpfact != 1:
                paramset['totalsteps'] = paramset.get('totalsteps',3000) * warpfact
                paramset['startsr'] = paramset.get('startsr',50) * warpfact
                paramset['maxsr'] = paramset.get('maxsr',100) * warpfact
            assert paramset['maxsr'] > paramset['startsr']
            mkey = paramset.pop('motor')
#            mkey = 'RA' if motor == 0 else 'DEC'
            minst = self.motors[mkey]
            minst._enabledrive(True)
            minst._setdir(paramset.pop('forward', True))
            if not minst._set_ms_mode(warpfact):
                self.allstop()
                return
            wavegens.append((minst.pulser(**paramset), mkey))
        self.wpgen = self.multiPulse(wavegens, self.singlepulsedone)
        self.sendStateUpdate(action=self.activeaction, status='started', message="ready to start waves with %d motors" % len(wavegens))
        self.wavemaker()

    def wavemaker(self):
        self.morewaves=True
        self.pio.wave_clear()
        self.wavids = deque()
        self.wavecount = 0
        while self.morewaves:
            self.morewaves = self.prepareWave()
            if len(self.wavids) < 2:
                newwave=self.pio.wave_create()
                self.wavids.append(newwave)
                self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
                if self.logwave:
                    self.sendStateUpdate(action=self.activeaction, status='progress', message="wave %d created and sent" % newwave)
            else:
                self.pendingwave = self.wavids.popleft()
                self.setpollfrom(.1, self.checkPending)
                return

        progmsg="all sent in initial set, running wave %d there are %d waves left." % (self.pio.wave_tx_at(), len(self.wavids))
        self.sendStateUpdate(action=self.activeaction, status='progress', message=progmsg)
        self.setpollfrom(.3, self.checkFinal) 

    def checkPending(self):
        wactive = self.pio.wave_tx_at()
        if wactive == self.pendingwave:
            self.setpollfrom(.1, self.checkPending)
            return
        self.pio.wave_delete(self.pendingwave)
        try:
            newwave=self.pio.wave_create()
        except pigpio.error as pe:
            newwave = None
            self.sendStateUpdate(action=self.activeaction, status='failed'
                    , message="pigpio wave create failed: {0}".format(pe))
        if newwave is None:
            self.setpollfrom(.3, self.checkFinal)
            return
        self.wavids.append(newwave)
        self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
        if self.logwave:
            self.sendStateUpdate(action=self.activeaction, status='progress'
                    , message="wave %d deleted, wave %d created and sent" % (self.pendingwave, newwave))
        if self.morewaves:
            self.morewaves = self.prepareWave()
            self.pendingwave = self.wavids.popleft()
            self.setpollfrom(.1, self.checkPending)
        else:
            self.setpollfrom(.3, self.checkFinal)
            progmsg="all sent in initial set, running wave %d there are %d waves left." % (self.pio.wave_tx_at(), len(self.wavids))
            self.sendStateUpdate(action=self.activeaction, status='progress', message=progmsg)

    def checkFinal(self):
        if self.pio.wave_tx_busy():
            self.setpollfrom(.2, self.checkFinal)
            return
        for wid in self.wavids:
            self.pio.wave_delete(wid)
            if self.logwave:
                self.sendStateUpdate(action=self.activeaction, status='progress'
                    , message="wave %d deleted." % (wid))
        self.allstop()
        self.sendStateUpdate(action=self.activeaction, status='complete', message="all done in %d waves" % self.wavecount)
        self.activeaction = None

    def prepareWave(self):
        onewave=True
        wavetime=0
        self.wavecount += 1
        groupleft=25
        moretodo = True
        while onewave:
            pulseparams = tuple(self.wpgen.__next__() for i in range(groupleft))
            if len(pulseparams) == 0:
                onewave = False
                moretodo = False
            else:
                self.pio.wave_add_generic((pgpulse(0,0,wavetime),) + tuple(pgpulse(*wp) for wp in pulseparams))
                for wp in pulseparams:
                    wavetime += wp[2]
                wpc=self.pio.wave_get_pulses()
                wcbc=self.pio.wave_get_cbs()
                wus=self.pio.wave_get_micros()
                if wpc > self.pwpulselim or wus > self.pwmslim or wcbc > self.pwcblim:
                    onewave=False
        return moretodo

    def allstop(self):
        self.pio.wave_tx_stop()
        for m in self.motors.values():
            m._enabledrive(False)

    def done(self):
        self.pio.stop()
        self.running = False
        self.sendStateUpdate(action='run agent', status='complete', message="closed")

    def setpin(self,pin,val):
        self.pio.write(pin,val)
        self.pio.write(pin,val)

    def singlepulsedone(self, pid):
        self.sendStateUpdate(action=self.activeaction, status='progress', message="pulse generator %s is complete." % str(pid))

    def multiPulse(self, multipulses, notifycomplete):
        """
        A generator that merges a number of pulse generators into single ordered sequence.
        
        multipulses is a tuple of tuple about each pulse sequence as follows:
            0 : a pulser for this pulse sequence
            1 : some id or other to identify this pulse stream
        notifycomplete is called as each pulser completes iteration, passing the pulse id that has completed
    
        The generator returns a tuple with the bits to waggle on now, bits to waggle off now and the delay until the next pulse is due.
        """
        nextpulses = []
        nextpulsedue = float('inf')
        active = False
        for pg in multipulses:
            try:
                pon, poff, pd = pg[0].__next__()
                if pd < nextpulsedue:
                    nextpulsedue = pd
                nextpulses.append([pd, pon, poff, pg])
                active = True
            except StopIteration:
                if not notifycomplete is None:
                    notifycomplete(pg[2])
        if len(nextpulses) == 0:
            self.sendStateUpdate(action=self.activeaction, status='failed', message="multiPulse has nothing to do!")
            raise StopIteration
        while active:
            ponbits = int(0)
            poffbits= int(0)
            thispulse = nextpulsedue
            nextpulsedue = float('inf')
            active = False
            for pg in nextpulses:
                if pg[0] is None:
                    if pg[1] == 0:
                        if not notifycomplete is None:
                            notifycomplete(pg[3][1])
                        pg[1] = 1
                else:
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
                            pg[0]=None
                            pg[1]=0
                    else:
                        active=True
                        if pg[0] < nextpulsedue:
                            nextpulsedue = pg[0]
            if nextpulsedue == float('inf'):
                active = False
            else:
                yield ponbits, poffbits, nextpulsedue

class clif(simpleAgent):
    """
    a very basic agent running in its own thread that prints responses and can be used to send to another agent
    """
    def statusUpdate(self, agent, action, status, message):
        """
        A simple status update method that prints out the update.
        
        agent  : the hierarchic name of the agent sending the message (a tuple)
        action : The action running that caused this update to be sent
        status : The status of the action; can be one of 'started', 'failed', 'progress' or 'complete'
        message: A simple string message describing the status update to be shown to the user.
        """
        print("%s: action >%s< %s: %s." % ('.'.join(agent), action, status, message))
        if action=='run agent' and status=='complete':
            self.running=False

    def stop(self):
        """
        closes down motors - uses rampdown if they are running in wave mode
        """
        self.runit('softstop')

    def crash(self):
        """
        instantly stops motors - accurate position will be lost if they are running fast
        """
        self.runit('crash')

    def runmotors(self,paramslist):
        """
        run motors using a list of parameters - 1 list entry per motor.
        
        See motor.pulser for details on the parameters to pass to each motor
        """
        self.runit('wavepulsemaker',pulseparams=paramslist)

def makeagents():
    cmdq = queue.Queue(5)
    respq = queue.Queue(5)
    sa = onewave(inq=cmdq, outq=respq, motors=(
          {'name':'RA',  'loglevel':7, 'pins':{'enable':21, 'direction': 12, 'step': 13, 'steplevels': (20,19,16)}}
        , {'name':'DEC', 'loglevel':7, 'pins':{'enable':25, 'direction': 18, 'step': 27, 'steplevels': (24,23,22)}}))
    return clif(inq=respq, outq=cmdq)


#    clif=stepwavetest.makeagents()
#    m1p={'motor':'RA', 'startsr':300, 'maxsr':900, 'overreach':1.1, 'ramp':1, 'totalsteps':1*16*48*120, 'pulseontime':2, 'forward':True,'warp':2}
#    m2p={'motor':'DEC', 'startsr':300, 'maxsr':850, 'overreach':1.1, 'ramp':.5, 'totalsteps':1*16*48*95, 'pulseontime':2, 'forward':True,'warp':2}
#    clif.runmotors((m1p,m2p))
