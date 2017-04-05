import pigpio
import time
from collections import deque

m1dirpin=12
m1steppin=13
m1ms3pin=16
m1ms2pin=19
m1ms1pin=20
m1enablepin=21
m2dirpin=18
m2steppin=27
m2ms3pin=22
m2ms2pin=23
m2ms1pin=24
m2enablepin=25

scripttrippin=4

ms1pin=0
ms2pin=1
ms3pin=2
steppin=3
dirpin=4
enablepin=5

mpins=[[m1ms1pin, m1ms2pin, m1ms3pin, m1steppin, m1dirpin, m1enablepin],[m2ms1pin,m2ms2pin,m2ms3pin, m2steppin, m2dirpin, m2enablepin]]
#mpins[motorno][pinname]

mpinbits=[[1<<m1ms1pin, 1<<m1ms2pin, 1<<m1ms3pin, 1<<m1steppin, 1<<m1dirpin, 1<<m1enablepin],
        [1<<m2ms1pin,1<<m2ms2pin,1<<m2ms3pin, 1<<m2steppin, 1<<m2dirpin, 1<<m2enablepin]]

allpins=(m1dirpin, m1steppin, m1ms3pin, m1ms2pin, m1ms1pin, m1enablepin
        , m2dirpin, m2steppin, m2ms3pin, m2ms2pin, m2ms1pin, m2enablepin, scripttrippin)

microstepset={
    1: (0,0,0),
    2: (1,0,0),
    4: (0,1,0),
    8: (1,1,0),
    16:(1,1,1)
}

class onewave():
    """
    testing class to run stepper motor using pigpio waves
    """
    
    def __init__(self):
        self.pio = pigpio.pi()
        self.maxpulsesperwave = self.pio.wave_get_max_pulses()
        self.maxmsperwave = self.pio.wave_get_max_micros()
        self.maxdblocksperwave = self.pio.wave_get_max_cbs()
        self.pwpulselim=self.maxpulsesperwave/3
        self.pwmslim = self.maxmsperwave/3
        self.pwcblim = self.maxdblocksperwave/3
        print("wave limits, pulses per wave: %d, seconds per wave: %5f, control blocks per wave: %d" % (self.maxpulsesperwave
            , self.maxmsperwave/1000000, self.maxdblocksperwave))
        
    def ramp1(self,start=10,final=50,overreach=1.2,ramp=1, motor=0, forward=True, warp=2, pulselen=2, runfor = 3):
        assert motor in (0,1)
        assert final > start
        self.setopins(mpins[motor])
        self.enabledrive(True, motor)
        self.setdir(forward, motor)
        self.set_ms_mode(warp, motor)
        mbits = mpinbits[0][steppin] if motor == 0 else mpinbits[1][steppin]
        pulsegen = pulser(start*warp, final*warp, overreach, ramp, runfor)
        morewaves=True
        self.pio.wave_clear()
        wavids = deque()
        firstwave=True
        while morewaves:
            onewave=True
            wavetime=0
            while onewave:
                try:
                    tick=pulsegen.__next__()
                    tick2=pulsegen.__next__()
                    tick3=pulsegen.__next__()
                    self.pio.wave_add_generic([pigpio.pulse(0,0,wavetime)
                        , pigpio.pulse(mbits, 0, pulselen), pigpio.pulse(0, mbits, tick-pulselen)
                        , pigpio.pulse(mbits, 0, pulselen), pigpio.pulse(0, mbits, tick2-pulselen)
                        , pigpio.pulse(mbits, 0, pulselen), pigpio.pulse(0, mbits, tick3-pulselen)])
                    wavetime += tick + tick2 + tick3
                    
                    if self.pio.wave_get_pulses() > self.pwpulselim or self.pio.wave_get_micros() > self.pwmslim or self.pio.wave_get_cbs() > self.pwcblim:
                        onewave=False

                except StopIteration:
                    onewave=False
                    morewaves=False
            print("ready to build 1 wave with %d pulses, %d control blocks" % (self.pio.wave_get_pulses(), self.pio.wave_get_cbs()))
            if firstwave:
                newwave=self.pio.wave_create()
                wavids.append(newwave)
                self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
                print("sent first wave ", newwave)
                firstwave=False
            else:
                if len(wavids) < 2:
                    newwave=self.pio.wave_create()
                    wavids.append(newwave)
                    self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
                    print("sent another wave - ", newwave)
                else:
                    wactive = self.pio.wave_tx_at()
                    oldest = wavids.popleft()
                    pausetime = 0
                    while wactive == oldest:
                        time.sleep(.1)
                        pausetime += .1
                        wactive = self.pio.wave_tx_at()
                    print("oldest (%d) has finished after waiting %3.2f" % (oldest, pausetime))
                    self.pio.wave_delete(oldest)
                    newwave=self.pio.wave_create()
                    wavids.append(newwave)
#                    print("created further wave id %d" % newwave)
                    self.pio.wave_send_using_mode(newwave, pigpio.WAVE_MODE_ONE_SHOT_SYNC)
#                    print("sent another wave - ", newwave)

        print("all sent, running ", self.pio.wave_tx_at(), ' there are ', len(wavids), " waves", 'left.')
        print("I have these waves %s" % str(wavids))
        for wid in wavids:
            print(wid)
        while self.pio.wave_tx_busy():
            print("running wave ", self.pio.wave_tx_at())
            time.sleep(.3)
        for wid in wavids:
            self.pio.wave_delete(wid)
        self.allstop()

    def setopins(self, ports):
        """reset all pins that we use to output mode"""
        for pin in ports:
            self.pio.set_mode(pin,pigpio.OUTPUT)

    def setdir(self, fwd, motor):
        if fwd:
            self.pio.write(mpins[motor][dirpin],0)
        else:
            self.pio.write(mpins[motor][dirpin],1)

    def enabledrive(self, enable, motor):
        if enable:
            self.pio.write(mpins[motor][enablepin],0)
        else:
            self.pio.write(mpins[motor][enablepin],1)

    def set_ms_mode(self, microsteps, motor):
        if microsteps in microstepset:
            motp = mpins[motor]
            self.pio.write(motp[ms1pin], microstepset[microsteps][0])
            self.pio.write(motp[ms2pin], microstepset[microsteps][1])
            self.pio.write(motp[ms3pin], microstepset[microsteps][2])
            print("pin %2d:%d, pin %2d:%d, pin %2d:%d" %(motp[ms1pin], microstepset[microsteps][0]
                        , motp[ms2pin], microstepset[microsteps][1], motp[ms3pin], microstepset[microsteps][2]))
        else:
            print("cannot do warp factor %d for motor %d" % (warp, motor))

    def allstop(self):
        self.pio.wave_tx_stop()
        self.enabledrive(False,0)
        self.enabledrive(False,1)

    def done(self):
        self.pio.stop()

def pulser(initial, final, overreach, ramp, flattime):
    """
    returns pulse lengths to ramp up from initial to final step rate, then maintain constant speed
    
    initial  : initial step rate in steps per second
    final    : final step rate in steps per second
    overreach: asymptotic approach to beyond final speed by this amount so we arrive in finite time (1.1 is good)
    ramp     : factor used to control the rate of ramp
    flattime : time in seconds to run flat out
    
    """
    print("pulser setup with ", initial, final, overreach, ramp)
    target = final*overreach
    trange = (target-initial)*ramp
    elapsed=ramp #  start at ramp for easy calculation
    ramping = True
    while ramping:
        inv = ramp / -elapsed
        tps=target+inv*trange/ramp
        if tps > final:
            ramping = False
            tps = final
        tick = int(1000000 / tps)
        elapsed += tick/1000000
        yield tick

    flatruncount = int(flattime * final)
    tick = int(1000000 / final)
    while flatruncount > 0:
        yield tick
        flatruncount -=1
