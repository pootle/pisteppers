import simplestepper, pagelink

from pootlestuff.watchables import myagents

import pootlestuff.watchables as wv

from webstrings import tablefieldinputhtml, tablefielddropdnhtml, tablefieldcyclicbtndnhtml, tablesectwrapper, tablefieldstatic

import locale
locale.setlocale(locale.LC_ALL, '')

class wwenumint(pagelink.wwenum):
    def websetter(self, webval):
        """
        convert strings to ints
        """
        super().websetter([int(x) for x in webval])

class webapp(simplestepper.multimotor):
    def __init__(self, **kwargs):
        self.classdefs=classlookups
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
        mcols=list(self.motors.keys())
        mrows={}
        for mix, mname in enumerate(mcols):
            for fname, field in self.motors[mname].makefieldset(pagelist=pagelist).items():
                if fname in mrows:
                    arow=mrows[fname]
                else:
                    arow=[None]*len(mcols)
                    mrows[fname]=arow
                arow[mix]=field
        fields += '<tr><td>Motors:</td>'+ ''.join(['<td><span class="sectheadtext" >%s</span></td>' % mn for mn in self.motors.keys()]) + '</tr>\n'
        for row in mrows.values():
            for rent in row:
                if not rent is None:
                    rowlink, rowstrs = rent
                    break
            colitems=[]
            for rent in row:
                if rent is None:
                    colitems.append('<td></td>')
                else:
                    flink, fstrs = rent
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
        fastmotors=[]
        for motor in self.motors.values():
            mparams={'command':motor.usercmd.getValue(), 
                    'targetpos':motor.userpos.getValue(),
                    'targetdir':motor.userdir.getValue(),
                    'stepmode':motor.userstepm.getValue()}
            iomode=motor.dothis(**mparams)
            print(iomode)
            if iomode=='wave':
                fastmotors.append((motor,mparams))
        if fastmotors:
            self.runfast(fastmotors)

topfielddefs=(
    (pagelink.wwlink, 'pigpmspw',   myagents.NONE,  'max us per wave',      tablefieldinputhtml,        'maximum microseconds per wave (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'pigpppw',    myagents.NONE,  'max pulses per wave',  tablefieldinputhtml,        'maximum pulses per wave (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'pigpbpw',    myagents.NONE,  'max cbs per wave',     tablefieldinputhtml,        'maximum DMA control blocks (pigpio limit)', {'liveformat': '{wl.varvalue:n}'}),
    (pagelink.wwlink, 'wavepulses', myagents.user,  'max pulses per wave',  tablefieldinputhtml,        'max pulses per wave - if pigpio runs out of CBs, decrease this'),
    (pagelink.wwenum, 'mode',       myagents.app,   'controller mode',      tablefielddropdnhtml,       'motorset controller mode'),
    (pagelink.wwbutton,'gotonow',   myagents.user,  'run now',              tablefieldcyclicbtndnhtml,  'starts motors running in their selected mode'),
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
        return defn[0](wable=None if defn[0] == pagelink.wwdummy else getattr(self, defn[1]),
                        pagelist=pagelist, updators=defn[2], label=defn[3], shelp=defn[5], **(defn[6] if len(defn) > 6 else {}))

    def makefieldset(self, pagelist, fieldlist=None):
        return {self.prefix+fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}

class webmotor(simplestepper.stepper, webgroup):
    def __init__(self, **kwargs):
        wables=[  # some extra fields used just for the ui
            ('usercmd',     wv.enumWatch,   self.commands[0],   False,  {'vlist': self.commands}),  # user preset command
            ('userpos',     wv.intWatch,    0,                  False),                             # user target pos for goto mode
            ('userdir',     wv.enumWatch,   'fwd',              False,  {'vlist': ('fwd', 'rev')}), # user direction for run mode
        ]
        simplestepper.stepper.__init__(self, wabledefs=wables, **kwargs)
        webgroup.__init__(self, webdefs=motorfieldindex, prefix=self.name+'_')

    def makefieldset(self, pagelist, fieldlist=None):
        basefields = {fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}
#        basefields['stepgroups']={stepname: getattr(self.stepmodes,stepname).makefieldset(pagelist, fieldlist=None)}
#        return
        for stepname in self.stepmodenames:
            stepdeffields=getattr(self.stepmodes,stepname).makefieldset(pagelist, fieldlist=None)
            basefields.update(stepdeffields)
        return basefields

motorfields=(
    (pagelink.wwenum, 'usercmd',    myagents.user,  'next command',    tablefielddropdnhtml,       'next command for this motor'),
    (pagelink.wwlink, 'userpos',    myagents.user,  'target position', tablefieldinputhtml,        'target position for next goto command'),
    (pagelink.wwenum, 'userdir',    myagents.user,  'direction',       tablefielddropdnhtml,       'direction for next run command'),
    (pagelink.wwenum, 'userstepm',  myagents.user,  'step settings',   tablefielddropdnhtml,       'step settings to use for next goto or run command'),
    (pagelink.wwenum, 'opmode',     myagents.app,   'motor mode',      tablefielddropdnhtml,       'motor controller mode'),
    (pagelink.wwlink, 'targetrawpos',myagents.app, 'target position',  tablefieldinputhtml,        'target absolute position in microsteps'),
    (pagelink.wwlink, 'rawposn',    myagents.app,   'current position',tablefieldinputhtml,        'current absolute position in microsteps (does not vary with changes to microstep level)'),
    (pagelink.wwlink, 'drive_enable',myagents.NONE, 'drive enable pin',tablefieldinputhtml,        'gpio (broadcom) pin used to drive enable this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'direction',  myagents.NONE,  'direction pin',   tablefieldinputhtml,        'gpio (broadcom) pin used to set step direction for this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'step',       myagents.NONE,  'step pin',        tablefieldinputhtml,        'gpio (broadcom) pin used to step this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'holdstopped',myagents.user,  'stop hold time',  tablefieldinputhtml,        'when motor stops, disable output current after this time (in seconds) - 0 means drive chip stays enabled'),
    (wwenumint,       'usteplevel', myagents.app,   'microstep level', tablefielddropdnhtml,       'selects level of microstepping in use'),
)

motorfieldindex={f[1]: f for f in motorfields}

class webstepcontrolv1(simplestepper.stepcontrols, webgroup):
    def __init__(self, **kwargs):
        simplestepper.stepcontrols.__init__(self, **kwargs)
        sfi={'stephead': (pagelink.wwdummy,'settings',   myagents.NONE,   'stepper settings',       tablefieldstatic,           None, {'value':self.name}),}
        sfi.update(sfiv1)
        webgroup.__init__(self, webdefs=sfi, prefix=self.name)

stepcontrolfieldsv1=(
     (pagelink.wwlink, 'minstep',    myagents.user,   'minimum step interval',  tablefieldinputhtml,        'absolute lower limit on step interval in seconds'),
     (pagelink.wwlink, 'startstep',  myagents.user,   'initial step interval',  tablefieldinputhtml,        'starting step interval in seconds'),
     (pagelink.wwlink, 'rampfact',   myagents.user,   'ramp scaling factor',    tablefieldinputhtml,        'change applied to step interval each ramp interval'),
     (pagelink.wwlink, 'rampintvl',  myagents.user,   'ramp scaling interval',  tablefieldinputhtml,        'interval between changes to ramp speed while ramping'),
     (pagelink.wwlink, 'rampticks',  myagents.user,   'ramp decel ticks',       tablefieldinputhtml,        'number of ticks from target that we start slowing down'),
     (wwenumint,       'usteps',     myagents.user,   'microstep level',        tablefielddropdnhtml,       'microstep level to use'),
     (pagelink.wwlink, 'ramputicks', myagents.app,    'actual ramp ticks',      tablefieldinputhtml,        'count of ticks actually used in last ramp operation'),
)

sfiv1={f[1]: f for f in stepcontrolfieldsv1}

class webstepcontrolv2(simplestepper.stepcontrolgrad, webgroup):
    def __init__(self, **kwargs):
        simplestepper.stepcontrolgrad.__init__(self, **kwargs)
        sfi={'stephead': (pagelink.wwdummy,'settings',   myagents.NONE,   'stepper settings',       tablefieldstatic,           None, {'value':self.name}),}
        sfi.update(sfiv2)
        webgroup.__init__(self, webdefs=sfi, prefix=self.name)

stepcontrolfieldsv2=(
     (pagelink.wwlink, 'minstep',    myagents.user,   'minimum step interval',  tablefieldinputhtml,        'absolute lower limit on step interval in seconds'),
     (pagelink.wwlink, 'startstep',  myagents.user,   'initial step interval',  tablefieldinputhtml,        'starting step interval in seconds'),
     (pagelink.wwlink, 'rampfact',   myagents.user,   'ramp scaling factor',    tablefieldinputhtml,        'change applied to step interval each ramp interval'),
     (pagelink.wwlink, 'rampintvl',  myagents.user,   'ramp scaling interval',  tablefieldinputhtml,        'interval between changes to ramp speed while ramping'),
     (pagelink.wwlink, 'rampticks',  myagents.user,   'ramp decel ticks',       tablefieldinputhtml,        'number of ticks from target that we start slowing down'),
     (wwenumint,       'usteps',     myagents.user,   'microstep level',        tablefielddropdnhtml,       'microstep level to use'),
     (pagelink.wwlink, 'ramputicks', myagents.app,    'actual ramp ticks',      tablefieldinputhtml,        'count of ticks actually used in last ramp operation'),
)

sfiv2={f[1]: f for f in stepcontrolfieldsv2}

classlookups = {  # maps the class names in json settings file to actual classes
    'A4988'     : webmotor,
    'stepramp1' : webstepcontrolv1,
    'stepramp2' : webstepcontrolv2, 
}