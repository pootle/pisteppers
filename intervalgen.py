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
    base class for tick interval generator.
    
    Instances of this class are created for each entry in a motor's 'stepmodes' in the json confog file.
    
    It records 
    """
    def __init__(self, name, motor, value, wabledefs, ticklogfile=None, **kwargs):
        """
        base class for step interval generators.
        
        name:   it's the name!
        
        motor:  the motor that owns this generator
        
        value:  a dict of further settings for creating an instance. This base class uses:
        
            ['mode']:   defines if this generator should use software to drive the stepping, or DMA (high precision) to drive the stepping.
                        This is not used within this class (it doesn't care) but is relevant when a step operation is being setup.
            ['usteplevel']: defines the microstep level that will be used. This is only used to scale between each step issued by this
                        generator and the resulting position change.
        
            Other entries in value are defined by subclasses.
        
        wabledefs: more watchable vars wanted by  subclasses
        
        ticklogfile: If not None then steps are written to this file as csv.
        """
        self.name=name
        self.motor=motor
        self.mode=value['mode']
        usvlist=motor.getustepnames()
        wabledefs.append(('usteplevel',   wv.enumWatch,   usvlist[0],     False,    {'vlist': usvlist})),     # selects the microstep level to use with these settings)
        super().__init__(value=value, wabledefs=wabledefs, **kwargs)
        self.ticklogf=ticklogfile

    def log(self, *args, **kwargs):
        self.motor.log(*args, **kwargs)

    def getmicrosteps(self):
        """
        returns a 2 tuple:
            0: number of microsteps per step for the microsteplevel 
            1: position change per actual tick
        """
        uslevelv = self.motor.getusteplevel(self.usteplevel.getValue())# number of microsteps per step
        return uslevelv, self.motor.maxstepfactor // uslevelv

    def tickgen(self):
        raise NotImplementedError()

class stepgensimple(stepgen):
    """
    A very basic step generator that defines the (only) step speed that is used.
    """
    def __init__(self, **kwargs):
        wables=[
            ('step',            wv.floatWatch,      .01,        False),     # The step interval this generator uses
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.log(loglvls.DEBUG,'stepgen %s setup for usteplevel %s and tick of %4.3f' % (self.name, self.usteplevel.getValue(), self.step.getValue()))

    def tickgen(self, command, initialpos):
        """
        generator function for step times.
        
        yields a sequence of values until target is reached, at which point it returns None for the time interval.
        This function keeps it's own time so can generate ticks independent of actual time - the caller must sync
        to a clock if required.
        
        command: 'run' or 'goto'.  run only terminates if the motor's stepactive variable is set False.
                                   goto will return no step required once target is reached, and terminate if running set False
        
        initialpos: starting position to use returned positions are all relative to this value
        
        returns:
            when changing direction this returns a 2 - tuple
                0: 1 character string: 'F' or 'R'
                1: time to next step (this will be 2* normal interval)
                
            otherwise this returns a single (float) value of time to next step (or None if no step is required)
        """
        if self.ticklogf is None:
            tlog=None
        else:
            tlog=open(self.ticklogf,'w')
        isgoto=command=='goto'                              # goto or move?
        uslevelv, usteps = self.getmicrosteps()
        tickget= self.step.getValue                         # time for full step - needs scaled to account for microstepping
        currentposv=initialpos                              # get the current position - we assume we are in control of this so don't re-read it
        activedirection=None                                # which direction motor currently running
        if isgoto:
            targetposget= self.motor.targetrawpos.getValue  # target position when in goto mode
            while self.motor.stepactive:
                movesteps=targetposget()-currentposv
                if abs(movesteps) < usteps:                     # if remaining usteps less than the current step size stop now
                    self.log(loglvls.INFO, 'at target pos')
                    yield None
                    if tlog:
                        tlog.write('X,0,%d\n' % currentposv)
                    currenttick=None
                else:
                    newdir='F' if movesteps > 0 else 'R'
                    if newdir != activedirection:
                        activedirection=newdir
                        currentposv += usteps if activedirection =='F' else -usteps
                        yield newdir, 2 * tickget()/uslevelv
                        if tlog:
                            tlog.write('%s,%7.5f,%d\n' % (' ' if setdir is None else setdir, tickget()/uslevelv, currentposv))
                    else:
                        currentposv += usteps if activedirection =='F' else -usteps
                        yield tickget()/uslevelv
                        if tlog:
                            tlog.write('%s,%7.5f,%d\n' % (' ' if setdir is None else setdir, tickget()/uslevelv, currentposv))
        else:
            targetdirget=self.motor.target_dir.getValue         # and the target dir when in run mode (-ve for reverse)
            while self.motor.stepactive:
                newdir = 'F' if targetdirget() > 0 else 'Z'
                if newdir != activedirection:
                    activedirection=newdir
                    yield newdir, 2 * tickget()/uslevelv
                else:
                    yield tickget()/uslevelv
        if tlog:
            tlog.close()

class stepconstacc(stepgen):
    """
    generates step timings with constant slope ramping.
    
    See stepgensimple for outputs
    """
    def __init__(self, **kwargs):
        wables=[
            ('startstep',           wv.floatWatch,      1,          False),     # The initial step interval
            ('minstep',             wv.floatWatch,      .1,         False),     # The fastest (minimum) step interval
            ('slope',               wv.floatWatch,      .1,         False),     # change in step interval per second
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.log(loglvls.DEBUG, 'stepgen %s setup for usteplevel %s and initial tick of %4.3f' % (self.name, self.usteplevel.getValue(), self.startstep.getValue()))

    def tickgen(self, command, initialpos):
        isgoto=command=='goto'                              # goto or move?
        uslevelv, usteps = self.getmicrosteps()
        startstepget = self.startstep.getValue
        minstepget   = self.minstep.getValue
        slopeget     = self.slope.getValue
        currtick=startstepget()
        currtps=1/currtick
        starttps=currtps
        maxtps=1/minstepget()
        ramptime = (maxtps-starttps) / slopeget()
        bothrampticks = (maxtps+starttps) * ramptime
        curtime=0
        rampupstart=curtime
        tickpos=initialpos
        recalc=True
        self.motor.stepactive=True
        if isgoto:
            targetget= self.motor.targetrawpos.getValue
            target=targetget()
            offset=target-tickpos
            currdir = 1 if offset > 0 else -1
            absoffset=abs(offset)
        else:
            dirget=self.motor.target_dir.getValue 
            currdir = 1 if dirget() > 0 else -1
#        print('KKKKKKKKKKKKKKKK', 'F' if currdir == 1 else 'R', startstepget()/uslevelv)
        yield 'F' if currdir == 1 else 'R', startstepget()/uslevelv
        if self.ticklogf is None:
            tlog=None
        else:
            tlog=open(self.ticklogf,'w')
        while (isgoto and absoffset > 0) or not isgoto:
            if isgoto:
                if target != targetget():
                    target=targetget()
                    recalc=True
            else:
                checkdir=1 if dirget() > 0 else -1
                recalc=currdir!= checkdir
            if recalc:
                if isgoto:
                    offset=target-tickpos
                    newdir, absoffset = (1, offset ) if offset > 0 else (-1 , -offset)
                    rampdown=usteps*bothrampticks//2 if absoffset > usteps*bothrampticks else (absoffset+2.5)//2
#                    print('brt', bothrampticks, 'test', absoffset > usteps*bothrampticks, 'result', rampdown)
                    rampupstart=curtime
                else:
                    newdir = 1 if dirget() > 0 else -1
                recalc=False
            else:
                absoffset=abs(target-tickpos)
            if (isgoto and absoffset  < rampdown) or newdir != currdir or not self.motor.stepactive: # we need to slow down
#                print('slowing')
                if currtps==maxtps:
                    self.log(loglvls.DEBUG, 'start slow down at %d' % tickpos)
                slope=slopeget()
                if currtps > starttps:
                    timeleft=((currtps-starttps) / slope)-currtick
                    currtps=starttps+timeleft*slope
                else:
                    if not self.motor.stepactive:
                        break
                    currtps = starttps      # if we end up slower than starttps, clamp to starttps
                if newdir != currdir and currtps<=starttps:
                    print('slow down and change dir')
                    tickpos += currdir*usteps
                    currtick=1/currtps
                    ustick=currtick/uslevelv
#                    print('yyyyyyyyyyyyyyyy', ustick)
                    yield ustick
                    if tlog:
                        tlog.write('B: %s,%7.5f,%d\n' % (' ' , ustick, tickpos))
                    curtime += ustick
                    tickpos += currdir*usteps
                    fliptick=startstepget()/uslevelv
#                    print('AAAAAAAAAAAAAAAAA', fliptick)
                    yield fliptick
                    if tlog:
                        tlog.write('C: %s,%7.5f,%d\n' % (' ' , fliptick, tickpos))
                    curtime += fliptick
                    currdir=newdir
                    tickpos += currdir*usteps
#                    print('oooooooooooooo','F' if currdir > 0 else 'R',fliptick * 2)
                    yield 'F' if currdir > 0 else 'R',fliptick * 2
                    if tlog:
                        tlog.write('D: %s,%7.5f,%d\n' % ('F' if currdir > 0 else 'R' , fliptick, tickpos))
                    curtime += fliptick
                    currtps=starttps
                    recalc=True
                    rampupstart=curtime
                    self.log(loglvls.DEBUG, 'flipdir at %d' % tickpos)
                elif currtps < starttps:
#                    print('clamping min speed')
                    curtps=starttps
                    self.log(loglvls.DEBUG, 'slow speed reached - ')
                    if not self.motor.stepactive:
                        break
            elif currtps<maxtps:
                currtps = starttps+(curtime-rampupstart)*slopeget()
#                print('speeding up', currtps, curtime, rampupstart, slopeget())
                if currtps > maxtps:
                    self.log(loglvls.DEBUG, 'clamp to %7.5f at tick %d'% (maxtps, tickpos))
                    currtps=maxtps
#                    print('max speed reached at', tickpos)
            else:
                pass
            currtick=1/currtps
            yield currtick/uslevelv
            curtime += currtick
            tickpos += currdir*usteps
        if tlog:
            tlog.write('%s,%7.5f,%d\n' % ('X' if currdir > 0 else 'R' , currtick/uslevelv, tickpos))
            tlog.close()