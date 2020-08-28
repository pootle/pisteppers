#!/usr/bin/python3

"""
The module provides the multimotor class which gives coordinated access to multiple motors, in particular it coordinates faststep
across multiple motors, merging the step streams of the motors using faststep into a single sequence of DMA
blocks using pigpio waves. (http://abyz.me.uk/rpi/pigpio/python.html#wave_add_new).
"""
import pootlestuff.watchables as wv
from pootlestuff.watchables import loglvls
import threading, time
import pigpio


controllermodes=('closed', 'off', 'faststep')

class multimotor(wv.watchablepigpio):
    def __init__(self, gpiolog=False, loglvl=loglvls.INFO, **kwargs):
        wables=[
            ('pigpmspw',    wv.intWatch,    0,                  False),
            ('pigpppw',     wv.intWatch,    0,                  False),
            ('pigpbpw',     wv.intWatch,    0,                  False),
            ('mode',        wv.enumWatch,   controllermodes[1], False,  {'vlist': controllermodes}),
            ('doitnow',     wv.btnWatch,    'Action',           False),
            ('wavepulses',  wv.intWatch,    1000,               True,   {'minv':100}),
            ('max_wave_time', wv.intWatch,  500000,             True,   {'minv': 1000, 'maxv': 1000000}),
            ('max_waves',   wv.intWatch,    3,                  True,   {'minv':2, 'maxv': 9}),
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
            newmotor=self.makeChild(defn=(motname, self.classdefs[motinfo['motorclass']], None, False, {'name': motname, 'loglevel': wv.loglvls.INFO}), value=motinfo)
            if newmotor is None:
                raise ValueError('motor construction failed for %s' % motname)
            self.motors[motname]=newmotor
#        self.cmndq=queue.Queue()
#        self.cthread=threading.Thread(name='controller(%s)' % type(self).__name__, target=self._thrunner)
#        self.cthread.start()

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

    def runfast(self, *args):
        cmode=self.mode.getValue()
        if cmode=='off':
            self.mode.setValue('faststep', wv.myagents.app)
            self.cthread=threading.Thread(name=type(self).__name__, target=self._threadfaststep, args=args)
            self.cthread.start()
        elif cmode=='closed':
            raise ValueError('controller is closed')
        else:
            raise ValueError('fast stepping already active')
 
    def _threadfaststep(self, motorlist):
        """
        fast step request - get the motors involved and .....
        motorlist: a dict with a key for each motor to use, and the value is the params for the motor
        the incoming pulses are:
            0: mask of bits to turn on
            1: mask of bits to turn off
            2: microsecond time of this pulse or None if the motor is now stationary
            3: raw position of this motor after this pulse
            4: the name of this motor
            5: action:
                0: normal pulse
                -1:action complete - motor should be stopped
                1: no-op - motor is stationary but may start again later
        """
        self.log(loglvls.INFO, "starting mode 'fast")
        timestart=time.time()
        mgens=[]
        motors={}
        for motor, moveparams in motorlist:
            tgen=motor.dmarun(**moveparams)
            if not tgen is None:
                mgens.append(tgen)
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
            maxwaves=self.max_waves.getValue()
            wavepercent=100//maxwaves
#            logf=open('wavelog.txt','w')
            logf=None
            sentbuffs=[]
            savedbuffs=[]
            while moredata:
                while len(pendingbufs) < maxwaves and moredata:
                    mposns={}
                    bufftime=self.max_wave_time.getValue()            # count the time and stop adding pulses if we get to this time
                    if len(savedbuffs) > 0:
                        nextbuff=savedbuffs.pop(0)
                    else:
                        nextbuff=[]
                    nextbuffi = 0
                    while nextbuffi < maxpulses:
                        try:
                            nextp=next(mergegen)
                        except StopIteration:
                            nextp=None
                        if nextp is None:
                            if len(nextbuff) <= nextbuffi:
                                nextbuff.append(pigpio.pulse(thisp[0], thisp[1], 1))
                            else:
                                rebuff = nextbuff[nextbuffi]
                                rebuff.gpio_on=thisp[0]
                                rebuff.gpio_off=thisp[1]
                                rebuff.delay=1
                            nextbuffi += 1
                            moredata=False
                            thisp=nextp
                            break
                        else:
                            dtime=nextp[2]-thisp[2]
                            assert dtime >= 0
                            if len(nextbuff) <= nextbuffi:
                                nextbuff.append(pigpio.pulse(thisp[0], thisp[1], dtime))
                            else:
                                rebuff = nextbuff[nextbuffi]
                                rebuff.gpio_on=thisp[0]
                                rebuff.gpio_off=thisp[1]
                                rebuff.delay= dtime
                            nextbuffi += 1
                            thisp=nextp
                            bufftime -= dtime
                            if bufftime <= 0:
                                break
                        mposns[thisp[4]]=thisp # 
                    if nextbuffi > 0:
#                        if not logf is None:
#                            for pp in nextbuff[:nextbuffi]:
#                                if pp.delay != 2:
#                                    logf.write('%8x, %8x, %5d\n' % (pp.gpio_on, pp.gpio_off, pp.delay))
                        try:
                            pcount=self.pio.wave_add_generic(nextbuff[:nextbuffi])
                            if not logf is None:
                                logf.write('wave_add_generic - count now %d\n' % pcount)
                        except Exception as ex:
                            '''oh dear we screwed up - let's print the the data we sent'''
                            print('FAIL in wave_add_generic' + str(ex))
                            for i, p in enumerate(nextbuff[:nextbuffi]):
                                print('%4d: on: %8x, off: %8x, delay: %8d' % (i, p.gpio_on, p.gpio_off, p.delay ))
                            raise
                        cbcount=self.pio.wave_get_cbs()
                        waveid=self.pio.wave_create_and_pad(wavepercent)
                        if not logf is None:
                            logf.write('wave %d created with %d cbs\n' % (waveid, cbcount))
                        sentbuffs.append(nextbuff)
                        pendingbufs.append(waveid)
                        buffends.append(mposns)
                        self.log(loglvls.DEBUG,'wave %d, duration %7.5f, size %d, cbs: %d' % (waveid, self.pio.wave_get_micros()/1000000, nextbuffi, cbcount))
                        self.pio.wave_send_using_mode(waveid, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
                        if not logf is None:
                            logf.write('wave_send_using_mode_one_shot_sync - %d\n' % (waveid))
                        nextbuffi = 0
                        if timestart:
                            self.log(loglvls.DEBUG,'startup time:::::::::::::: %6.3f' % (time.time()-timestart))
                            timestart=None
                if len(pendingbufs) > 0:
                    current=self.pio.wave_tx_at()
                    if current == 9999:
                        self.log(loglvls.WARN,'9999 wave id received')
                    endposns=None
                    while len(pendingbufs) > 0  and current != pendingbufs[0]:
                        donebuf = pendingbufs.pop(0)
                        try:
                            self.pio.wave_delete(donebuf)
                            if not logf is None:
                                logf.write('wave %d deleted\n' % donebuf)
                            self.log(loglvls.DEBUG,'wave %d complete, remains: %s' % (donebuf, pendingbufs))
                        except pigpio.error:                            
                            self.log(loglvls.DEBUG,'wave delete failed for wave %d with %s' % (donebuf, pendingbufs))
                            raise
                        endposns = buffends.pop(0)
                        savedbuffs.append(sentbuffs.pop(0))
                    if not endposns is None:
                        for mn, mp in endposns.items():
                            self.motors[mn].rawposn.setValue(mp[3], wv.myagents.app)
                            if mp[5] == -1:
                                motors[mp[4]].endstepping()
                                motors[mp[4]].opmode.setValue('stopped', wv.myagents.app)
#                                self.stepactive=False
                if len(pendingbufs) >= maxwaves: # check if we have room for another wave right now - if not, wait a bit
                    time.sleep(.1)
            if not logf is None:
                logf.close()
            self.log(loglvls.DEBUG, 'final waves %d' % len(pendingbufs))
            while len(pendingbufs) > 0:
                time.sleep(.2)
                current=self.pio.wave_tx_at()
                if current == 9999 or pendingbufs.index(current) != 0:
                    donebuf = pendingbufs.pop(0)
                    self.log(loglvls.DEBUG,'wave %d complete' % donebuf )
                    self.pio.wave_delete(donebuf)
                    endposns = buffends.pop(0)
                    for mn, mp in endposns.items():
                        self.motors[mn].rawposn.setValue(mp[3], wv.myagents.app)
                        if mp[5] == -1:
                            motors[mp[4]].endstepping()
                            motors[mp[4]].opmode.setValue('stopped', wv.myagents.app)
#                            self.stepactive=False
                elif current ==pendingbufs[0]:
                    pass
#                    self.log(loglvls.DEBUG,'wave %d running' % current)
                else:
                    self.log(loglvls.DEBUG,'BBBBBBBBBBBBBBBBBBBBBBBBBBAArg')
#            self.pio.wave_clear()
        if not logf is None:
            logf.close()
        self.log(loglvls.INFO, "motoset leaving mode fast")
        self.mode.setValue('off', wv.myagents.app)
#        for motor in motors.values():
#            motor.endstepping()
#            motor.dothis(command =   'stop')
        self.mode.setValue('off', wv.myagents.app)
#            self.log(logging.INFO, self.reporttimelog('fastrun'))

    def pulsemerge(self, mgens):
        """
        produces an ordered sequence of pulse requests from all sources
        
        mgens: list of pulse sources, each entry is a pulse generator
            
        The incoming pulses are tuples:

            0: don't care
            1: don't care
            2: microsecond time of this pulse (or None if the motor is now stationary?)
            3.... don't care
        """
        mpulses=[]
        newgens=[]
        for gen in mgens:
            try:
                mpulses.append(next(gen))
                newgens.append(gen)
            except StopIteration:
                pass
        if len(mpulses) == 0:
            pass
        elif len(mpulses) == 1:
            gen=newgens[0]
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
                            pulseB=next(genA)
                        except StopIteration:
                            pulseB = None
                else:
                    if pulseB is None or pulseA[2] < pulseB[2]:
                        yield pulseA
                        try:
                            pulseA = next(genA)
                        except StopIteration:
                            pulseA = None
                    else:
                         yield pulseB
                         try:
                             pulseB = next(genB)
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
                        mpulses[nextix] = next(newgens[nextix])
                    except StopIteration:
                        mpulses[nextix] = None
                else:
                    for nx in nextix:
                        yield mpulses[nx]
                        try:
                            mpulses[nx] = next(newgens[nx])
                        except StopIteration:
                            mpulses[nx] = None