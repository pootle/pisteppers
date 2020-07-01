#!/usr/bin/python3
"""
This module has a couple of step timing classes for stepper motors.

Timings are generated on demand by a generator that uses various settings held by the class.

Some of these settings can be changed as the generator is running, as can a couple of settings
within the motor instance that contains this class; the generator adapts dynamically to 
these changes.

The generators work in their own time frame so can be used both in real time and to pre-prepare
step timings, this is handled by the code calling these generators.

"""
import pootlestuff.watchables as wv
from pootlestuff.watchables import loglvls

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
        usvlist=motor.usteplevel.vlist
        wabledefs.append(('usteps',   wv.enumWatch,   usvlist[1],     False,    {'vlist': usvlist})),     # selects the microstep level to use with these settings)
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
        uslevelv = self.motor.getusteplevelint() # usteplevel.getValue()         # current microstep level
        print('maxstep type: %s' % type(self.motor.maxstepfactor).__name__, self.motor.maxstepfactor, 'type level', type(uslevelv).__name__)
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
