#!/usr/bin/python3

import stepperdriver as sd
import stellarGeometry as sg
import time

class smartmount(sd.onewave):
    """
    stepper motor based moving updated with functions for a telescope mount
    """

    def getStateTicker(self, interval=None):
        if not interval is None:
            self.srinterval = int(interval)
        mstates = {motid:mot.getState() for motid,mot in self.motors.items()}
        motdec = mstates['DEC']['degpos']
        if motdec <= 90:
            motlh = mstates['RA']['degpos']
        elif motdec <= 270:
            motdec = 180-motdec
            motlh = (mstates['RA']['degpos']+180) %360
        else:
            motdec = motdec - 360
            motlh = mstates['RA']['degpos']
        tnow = time.time()
        for mk, ms in mstates.items():
            if ms['action']=='fastwave':
                ms['timeleft'] = self.gotoendtimes[mk]-tnow
        self.hostConn.runFunc('statusReport', **{'LH': motlh, 'DEC': motdec, 'state': self.mode, 'motors': mstates})
        if self.srinterval > 0:
            self.runafter(self.srinterval, self.getStateTicker)

    def motorfastends(self,mot):
        timeerror = time.time() - self.gotoendtimes[mot]
        print("time error %3.2f for %s" % (timeerror, mot))

    def slowRun(self, slowparams):
        """
        Give a tuple of slowparams, passes each set of params to the relevant motor.
        """
        for pset in slowparams:
            mkey = pset['motor']
            print('slow run parms for',mkey, '->',pset)
            mot=self.motors[mkey]
            smparms={'steprate':mot.degtosteps(abs(pset['degrate'])), 'totalsteps':-1
                    , 'pulseontime':pset['pulseontime'], 'forward':pset['degrate'] > 0, 'warp':pset['warp']}
            print('slowmove parms',smparms)
            mot.slowmove(**smparms)

    def goto(self, calconly, **kwargs):
        target = self._resolveLocationParams('goto', **kwargs)
        if target is None:
            return   #couldn't make sense of the params
        if isinstance(target, sg.motorPair):
            movedef = self._bestmovetime(target)
        elif isinstance(target, sg.localEquatorial):
            targetpair = self._convertlocal(target)
            targetmoves = tuple(self._bestmovetime(mpair) for mpair in targetpair)
            bm1 = max(targetmoves[0]['RA'][0], targetmoves[0]['DEC'][0])
            bm2 = max(targetmoves[1]['RA'][0], targetmoves[1]['DEC'][0])
            movedef = targetmoves[0] if bm1 < bm2 else targetmoves[1]
        else:
            self.logmsg(level=sd.LOGLVLFAIL, action='goto', status='failed', message='unable to use a' + str(type(target)))
            return

        self.logmsg(level=sd.LOGLVLDETAIL, action='goto', status='started', func='goto', message=str(target))
        moveparams=[]
        self.gotoendtimes={}
        tnow = time.time()
        gotorep={}
        for mot, mmove in movedef.items():
            gotorep[mot]={'current':self.motors[mot].degpos(), 'target':mmove[3], 'move':mmove[1], 'time':mmove[0]}
            if abs(mmove[2])>16:
                msets = self.motors[mot].fastdefaults.copy()
                msets['forward'] = mmove[1] >= 0
                msets['totalsteps'] = mmove[2]
                msets['motor']=mot
                moveparams.append(msets)
                self.gotoendtimes[mot]=mmove[0]+tnow
        self.logmsg(action='goto', status='progress', level=sd.LOGLVLINFO
            , message=', '.join('%s move by %3.2f should take %3.2f seconds' % (mot, mmove[1], mmove[0]) for mot, mmove in movedef.items()))
        if len(gotorep) > 0:
            self.hostConn.runFunc('gotoReport', movereport=gotorep)
        if len(moveparams) > 0 and not calconly:
            self.wavepulsemaker(pulseparams=moveparams)
        else:
            self.logmsg(action='run agent', status='complete', level=sd.LOGLVLINFO,message='no movement required')
        return

    def setPosition(self, **kwargs):
        """
        Set the mount motor positions from the given arguments.

        If the mount has been moved manually, or the current sky position has been accurately measured by plate solving
        or equivalent, then use this to update the mount's position. This can only be done when idle or tracking, not when fast 
        slewing.
        """
        target = self._resolveLocationParams('setPosition', **kwargs)
        if target is None:
            return
        self.motors['RA'].setpos(target.ramot.deg)
        self.motors['DEC'].setpos(target.decmot.deg)
        self.logmsg(level=sd.LOGLVLDETAIL, action='setposition', status='complete', func='setPosition', message=str(target))

    def updateSettings(self, **updatesdict):
        for k,v in updatesdict.items():
            self.motors[k].updateSettings(**v)

    def requestSettings(self):
        self.hostConn.runFunc('settingsReport', settings={key: m.settings for key, m in self.motors.items()})

    def _resolveLocationParams(self, request, **kwargs):
        """
        housekeeping routine for setPosition and goto functions. Checks state and parses params to create and return 
        a stellarGeometry object, which can be one of several types defining the target in different ways 
        """
        fails=[]
        for motor in self.motors.values():
            if not motor.motmode in ('slowstep','idle'):
                fails.append(motor.mname + ' cannot goto from state ' + motor.motmode)
        if len(fails) > 0:
            self.logmsg(level=sd.LOGLVLFAIL, action=request, status='failed', message=', '.join(fails))
            return None
        return sg.makeGeom(**kwargs)

    def _convertlocal(self, lcoords):
        """
        returns the 2 solutions for mount positions to match the given local equatorial co-ordinates
        
        dec 0 to 90 => motor 0 to 90, 180 to 90
        dec -1 to - 90 => motor 359 to 270, 181 to 270
        """
        hours=lcoords.hour.deg
        dec=lcoords.dec.deg
        targetabove = not 90 < hours < 270
        sols=[]
        decwrap = (dec+90)%180-90
        if dec>=0:
            decs = (decwrap, 180-decwrap)
        else:
            decs = (360+decwrap,180-decwrap)
        rams=((hours) % 360, (hours+180) % 360)
        for ram in rams:
            for decm in (decs):
                if self.aboveeqh(ram,decm) == targetabove:
                    sols.append(sg.motorPair(ramot=ram, decmot=decm))
                    print(sols[-1])
        return sols

    def _bestmovetime(self, motpair):
        """
        given a motorPair returns the fastest move time for the target location
        """
        targvals={'RA': motpair.ramot.deg, 'DEC': motpair.decmot.deg}
        return {mname: mot.timetomove(target=targvals[mname])[0] + (targvals[mname],) for mname, mot in self.motors.items()}

    def aboveeqh(self, hapos, decpos):
        if hapos <= 90 or hapos >= 270:
            return decpos < 90 or decpos > 270
        else:
            return 90 < decpos < 270

    def quick360(self, move):
        if move > 180:
            return move-360
        if move < -180:
            return move+360
        return move

if __name__=="__main__":
    import mountSetup
    sa = smartmount(**mountSetup.mountParams)
    sa.runforever()
