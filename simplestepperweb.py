import simplestepper, pagelink

from pootlestuff.watchables import myagents

import pootlestuff.watchables as wv

from webstrings import tablefieldinputhtml, tablefielddropdnhtml, tablefieldcyclicbtndnhtml, tablesectwrapper

import locale
locale.setlocale(locale.LC_ALL, '')

class webapp(simplestepper.multimotor):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.gotonow.addNotify(self.multigoto, myagents.user)

    def allfields(self, pagelist, fielddefs):
        fieldstrs = [defn[0](wable=getattr(self, defn[1]), pagelist=pagelist, updators=defn[2], label=defn[3], shelp=defn[5], **(defn[6] if len(defn) > 6 else {})).
                webitem(fformat=defn[4]) for defn in fielddefs]
        return ''.join(fieldstrs)

    def makePanel(self, pagelist):
        return tablesectwrapper.format(style='focusstyle', flipid='xfcs', fields=self.allfields(pagelist,allfielddefs), title='stepper controls')

    def makeMainPage(self, pagelist, qp,pp,page):
        fields=self.allfields(pagelist=pagelist, fielddefs=topfielddefs)
        pagecols={}
        for motor in self.motors.values():
            for mfn, mf in motor.makefieldset(pagelist=pagelist).items():
                if mfn in pagecols:
                    pagecols[mfn].append(mf)
                else:
                    pagecols[mfn]=[mf]
        fields += '<tr><td></td>'+ ''.join(['<td>%s</td>' % mn for mn in self.motors.keys()]) + '</tr>\n'
        for row in pagecols.values():
            rowlink, rowstrs =row[0]
            colitems=[]
            for flink, fstrs in row:
                attrs={'id': str(flink.fid)} if hasattr(flink, 'fid') else {}
                if flink.updators and flink.updators & flink.pagelist.userupdate:
                    if wv.wflags.DISABLED in flink.wable.flags:
                        attrs['disabled']='disabled'
                    attrs[flink.clickact]='appNotify(this, {})'.format(flink.pagelist.pageid)
                attrstr=' '.join([('%s="%s"' % (ak, av)) for ak, av in attrs.items()])
                colitems.append((fstrs['fieldu'] if flink.updators & flink.pagelist.userupdate else fstrs['fieldf']).format(wl=flink, tattrs=attrstr))
            rowhtml='<tr><td class="fieldlabel" {wshelp}>{wlabel}</td>{colfields}<td>{whelp}</td></tr>'.format(
                wshelp = rowstrs['shelp'].format(wl=rowlink) if hasattr(rowlink, 'shelp') else '',
                wlabel = rowstrs['label'].format(wl=rowlink) if hasattr(rowlink, 'label') else '',
                colfields=''.join(['<td>%s</td>' % ci for ci in colitems]),
                whelp  = rowstrs['fhelp'].format(wl=rowlink) if hasattr(rowlink, 'doclink') else ''
                )
            fields += rowhtml
        with open(page,'r') as pf:
            templ=pf.read()
        return {'resp':200, 'headers': (('Content-Type', 'text/html; charset=utf-8'),), 'data':templ.format(pageid = pagelist.pageid, fields=fields)}

    def multigoto(self, oldValue, newValue, agent, watched):
        """
        simple button that starts goto using the modes and targets setup on the individual motors
        """
        assert self.mode.getValue()=='off'
        fastgens=False
        for motor in self.motors.values():
            mmode=motor.runmode.getIndex()  # 0 for ignore this motor, 1 for slow goto, 2 for slow track, 3 for fast goto
            if mmode==1 or mmode==2:
                motor.setValue(value={'mode': 'slowgoto' if mmode==1 else 'slowtrack'}, agent=myagents.app)
            elif mmode==3:
                fastgens=True
        if fastgens:
            self.mode.setValue('faststep', myagents.user)

topfielddefs=(
    (pagelink.wwenum, 'mode',       myagents.app,   'controller mode',      tablefielddropdnhtml,       'motorset controller mode'),
    (pagelink.wwbutton,'gotonow',   myagents.user,  'goto now',             tablefieldcyclicbtndnhtml,  'starts motors running in their selected mode'),
    (pagelink.wwlink, 'pigpmspw',   myagents.NONE,  'max us per wave',      tablefieldinputhtml,        'maximum microseconds per wave (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'pigpppw',    myagents.NONE,  'max pulses per wave',  tablefieldinputhtml,        'maximum pulses per wave (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'pigpbpw',    myagents.NONE,  'max cbs per wave',     tablefieldinputhtml,        'maximum DMA control blocks (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'wavepulses', myagents.user,  'max pulses per wave',  tablefieldinputhtml,        'max pulses per wave - if pigpio runs out of CBs, decrease this'),
)

class webgroup():
    def __init__(self, webdefs, prefix=''):
        self.webdefs=webdefs
        self.prefix=prefix

    def makefield(self, pagelist, defn):
        """
        creates the link class for a field, if the field is user or app updateable it is also added to the pagelist.
        
        This object is used to create the html snippets for the various parts of the field. If it is not updateable it is discarded after the page is built
        """
        return defn[0](wable=getattr(self, defn[1]), pagelist=pagelist, updators=defn[2], label=self.prefix+defn[3], shelp=defn[5], **(defn[6] if len(defn) > 6 else {}))

    def makefieldset(self, pagelist, fieldlist=None):
        return {self.prefix+fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}

class webmotor(simplestepper.stepper, webgroup):
    def __init__(self, **kwargs):
        simplestepper.stepper.__init__(self, stepcontroller=webstepcontrols, **kwargs)
        webgroup.__init__(self, webdefs=motorfieldindex, prefix=self.name+'_')

    def makefieldset(self, pagelist, fieldlist=None):
        basefields = {fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}
        for stepnames in ('fast', 'slow'):
            stepgroup=getattr(self,stepnames)
            basefields.update(stepgroup.makefieldset(pagelist, fieldlist=None))
        return basefields

motorfields=(
    (pagelink.wwenum, 'mode',       myagents.app,   'motor mode',      tablefielddropdnhtml,       'motor controller mode'),
    (pagelink.wwenum, 'start_mode', myagents.user,  'initial mode',    tablefielddropdnhtml,       'mode used when motor initialised'),
    (pagelink.wwenum, 'runmode',    myagents.user,  'next action mode',tablefielddropdnhtml,       'mode for next controller move'),
    (pagelink.wwlink, 'drive_enable',myagents.NONE, 'drive enable pin',tablefieldinputhtml,        'gpio (broadcom) pin used to drive enable this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'direction',  myagents.NONE,  'direction pin',   tablefieldinputhtml,        'gpio (broadcom) pin used to set step direction for this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'step',       myagents.NONE,  'step pin',        tablefieldinputhtml,        'gpio (broadcom) pin used to step this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'holdstopped',myagents.user,  'stop hold time',  tablefieldinputhtml,        'when motor stops, disable output current after this time (in seconds) - 0 means drive chip stays enabled'),
    (pagelink.wwenum, 'usteplevel', myagents.user,  'microstep level', tablefielddropdnhtml,       'selects level of microstepping to use'),
    (pagelink.wwlink, 'rawposn',    myagents.app,   'current position',tablefieldinputhtml,        'current absolute position in microsteps (does not vary with changes to microstep level)'),
    (pagelink.wwlink, 'targetrawpos',myagents.user, 'target position' ,tablefieldinputhtml,        'target absolute position in microsteps'),
)

motorfieldindex={f[1]: f for f in motorfields}

class webstepcontrols(simplestepper.stepcontrols, webgroup):
    def __init__(self, **kwargs):
        simplestepper.stepcontrols.__init__(self, **kwargs)
        webgroup.__init__(self, webdefs=stepfieldindex, prefix=self.name)

stepcontrolfields=(
     (pagelink.wwlink, 'minstep',    myagents.user,   'minimum step interval',  tablefieldinputhtml,        'absolute lower limit on step interval in seconds'),
     (pagelink.wwlink, 'startstep',  myagents.user,   'initial step interval',  tablefieldinputhtml,        'starting step interval in seconds'),
     (pagelink.wwlink, 'rampfact',   myagents.user,   'ramp scaling factor',    tablefieldinputhtml,        'change applied to step interval each ramp interval'),
     (pagelink.wwlink, 'rampintvl',  myagents.user,   'ramp scaling interval',  tablefieldinputhtml,        'interval between changes to ramp speed while ramping'),
     (pagelink.wwlink, 'rampticks',  myagents.user,   'ramp decel ticks',       tablefieldinputhtml,        'number of ticks from target that we start slowing down'),
     (pagelink.wwlink, 'ramputicks', myagents.app,    'actual ramp ticks',      tablefieldinputhtml,        'count of ticks actually used in last ramp operation'),
)

stepfieldindex={f[1]: f for f in stepcontrolfields}
