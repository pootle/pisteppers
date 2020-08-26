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
import time

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

class stepgenonespeed(stepgen):
    """
    A very basic step generator that defines the (only) step speed that is used.
    """
    def __init__(self, **kwargs):
        wables=[
            ('steprate',            wv.floatWatch,      .01,        False),     # The step rate in full steps per second
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.log(loglvls.DEBUG,'stepgen %s setup for usteplevel %s and tick of %4.3f' % (self.name, self.usteplevel.getValue(), self.steprate.getValue()))

    def tickgen(self, command, initialpos):
        """
        generator function for step times.
        
        yields a sequence of values until motor.stepactive is set False.
        
        command: 'run' or 'goto'.  
                run only terminates if the motor's stepactive variable is set False. motor.target_dir is monitored as is this instance's step.
                goto will return no step required once target is reached, and terminate if running set False. the motor's targetpos is monitored and
                direction is changed if necessary. This instance's step variable is monitored.
        
        initialpos: starting position to use only used for goto to track current position relative to target position
        
        returns:
            when changing direction this returns a 2 - tuple
                0: 1 character string: 'F' or 'R'
                1: time to next step (this will be same interval)
                
            otherwise this returns a single (float) value of time to next step (or None if no step is required)
        """
        if self.ticklogf is None:
            tlog=None
        else:
            tlog=open(self.ticklogf,'w')
        isgoto=command=='goto' or command=='onegoto'        # goto or move?
        uslevelv, usteps = self.getmicrosteps()
        activedirection=None                                # which direction motor currently running
        motor=self.motor
        motor.updatetickerparams=True
        print('softstep 1')
        if isgoto:
            currentposv=initialpos                              # get the current position - we assume we are in control of this so don't re-read it
            while self.motor.stepactive:
                if motor.updatetickerparams:
                    stepsleft=motor.targetrawpos.getValue()-currentposv
                    absstepsleft=abs(stepsleft)
                    stepsps=self.steprate.getValue()
                    newdir='F' if stepsleft > 0 else 'R'
                    usetick=1/stepsps/uslevelv
                    if newdir != activedirection:
                        activedirection=newdir
                        yield newdir, 0.00002 if activedirection is None else usetick
                        if tlog:
                            tlog.write('%s,%7.5f,%d\n' % (newdir, usetick, currentposv))
                    print('using tick', usetick)
                    motor.updatetickerparams=False
                if absstepsleft >= usteps:                     # if remaining usteps less than the current step size stop now
                    currentposv += usteps if activedirection =='F' else -usteps
                    absstepsleft -= usteps
                    yield usetick
                    if tlog:
                        tlog.write('%s,%7.5f,%d\n' % (' ', usetick, currentposv))
                else:
                    yield None
                    if tlog:
                        tlog.write('X,0,%d\n' % currentposv)
                    currenttick=None
        else:
            while self.motor.stepactive:
                if motor.updatetickerparams:
                    newdir = 'F' if self.motor.target_dir.getValue() > 0 else 'R'
                    stepsps=self.steprate.getValue()
                    usetick=1/stepsps/uslevelv
                    if newdir != activedirection:
                        usetick = 0.00002 if activedirection is None else usetick
                        activedirection=newdir
                        if tlog:
                            tlog.write('%s,%7.5f\n' % (newdir, usetick))
                        yield newdir, usetick
                    motor.updatetickerparams=False
                yield usetick
                if tlog:
                    tlog.write('%s,%7.5f\n' % (' ', usetick))    
        if tlog:
            tlog.close()

class stepconstacc(stepgen):
    """
    generates step timings with constant slope ramping.
    
    See stepgenonespeed for outputs
    """
    def __init__(self, **kwargs):
        wables=[
            ('slowtps',            wv.floatWatch,      1,          False),     # The initial steps per second
            ('fasttps',            wv.floatWatch,      .1,         False),     # The fastest steps per second
            ('slope',               wv.floatWatch,      .1,         False),     # change in step interval per second
        ]
        super().__init__(wabledefs=wables, **kwargs)
        self.log(loglvls.DEBUG, 'stepgen %s setup for usteplevel %s and initial tick of %4.3f' % (self.name, self.usteplevel.getValue(), self.slowtps.getValue()))

    def tickgen(self, command, initialpos):
        """
        yields a sequence of
        """
        isgoto=command=='goto' or command=='onegoto'                # goto or move?
        tickpos=initialpos
        uslevelv, usteps = self.getmicrosteps()                     # uslevelv is number of microsteps per step for the microsteplevel
        uslevelv=float(uslevelv)                                    # so it's right for later
        print('uslevelv', uslevelv, 'usteps',usteps)                # usteps is the change in position for each step
        motor=self.motor
        motor.updatetickerparams=True
        currdir=None
        currtps=self.slowtps.getValue()                            # number of full steps per second to start at
        xelapsed=time.time()    # just a testing var
        ramping=True            # just a testing var
        if self.ticklogf is None:
            tlog=None
        else:
            tlog=open(self.ticklogf,'w')
        while self.motor.stepactive:
            if motor.updatetickerparams:
                slowtps=self.slowtps.getValue()
                fasttps=self.fasttps.getValue()
                slope=self.slope.getValue()
                if isgoto:
                    target=self.motor.targetrawpos.getValue()
                    offset=target-tickpos
                    absoffset=abs(offset)
                    newdir=1 if offset > 0 else -1
                else:
                    newdir = 1 if self.motor.target_dir.getValue() > 0 else -1
                currtick = 1/currtps/uslevelv
                if isgoto:
                    deceltime=(currtps-slowtps) / slope                # time it will take to get to slow speed from current speed
                    averagetps=(currtps+slowtps) / 2
                    decelfullsteps = averagetps * deceltime
                    decelusteps = (decelfullsteps+5) * self.motor.maxstepfactor # distance from target deceleration should start
                motor.updatetickerparams=False

            if newdir != currdir or isgoto and (absoffset < decelusteps):   # check if slowdown needed
                if currtps <= slowtps:                                       # reached min speed
                    if newdir != currdir:                                    # are we waiting to change direction?
                        yield 'F' if newdir > 0 else 'R', .00002 if currdir is None else currtick
                        currdir = newdir
                    elif isgoto:
                        if absoffset < usteps/2:                           # and as close as possible to target
                            print('XXXXXXXXXXXXXXXXXXXXxxxxxat target', absoffset, decelusteps, usteps, target, tickpos)
                            yield None
                        else:
                            print('nearly.....................................', absoffset)
                            yield currtick                                  # just a bit further.....
                            tickpos += usteps*currdir
                            if isgoto:
                                absoffset -= usteps
                else:                                                       # slowing down
                    currtps -= slope*currtick                                # not exactly right, but simple calc
                    if currtps < slowtps:
                        currtps=slowtps
                    currtick=1/currtps/uslevelv
                    yield currtick
                    tickpos += usteps*currdir
                    if isgoto:
                        absoffset -= usteps
            elif currtps < fasttps:                                         # go faster
                currtps += slope*currtick # over accelerates more at low speeds - but simple calculation
                if currtps > fasttps:
                    currtps = fasttps
                currtick = 1/currtps/uslevelv
                yield currtick
                tickpos += usteps*currdir
                if isgoto:
                    absoffset -= usteps
                    deceltime=(currtps-slowtps) / slope                     # time it will take to get to slow speed from current speed
                    averagetps=(currtps+slowtps) / 2
                    decelfullsteps = averagetps * deceltime
                    decelusteps = (decelfullsteps+5) * self.motor.maxstepfactor # distance from target deceleration should start

            else: # speed is max - keep going
                if ramping:
                    print('rapmped at', time.time()-xelapsed)
                    ramping=False
                yield currtick
                tickpos += usteps*currdir
                if isgoto:
                    absoffset -= usteps

        while currtps > slowtps:
            currtps -= slope * currtick
            currtick=1/currtps/uslevelv
            yield currtick
            if tlog:
                tlog.write('B: %s,%7.5f,%d\n' % (' ' , currtick/uslevelv, tickpos))
            tickpos += currdir*usteps
        if tlog:
            tlog.write('%s,%7.5f,%d\n' % ('X' if currdir > 0 else 'R' , currtick/uslevelv, tickpos))
            tlog.close()