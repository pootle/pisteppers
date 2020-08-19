import steppers, intervalgen, pagelink

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

class webapp(steppers.multimotor):
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
            print('multigoto', motor.name, mparams)
            iomode=motor.dothis(**mparams)
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
    (pagelink.wwbutton,'gotonow',   myagents.user,  'Action',               tablefieldcyclicbtndnhtml,  'starts motors running in their selected mode'),
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
        try:
            return defn[0](wable=None if defn[0] == pagelink.wwdummy else getattr(self, defn[1]),
                        pagelist=pagelist, updators=defn[2], label=defn[3], shelp=defn[5], **(defn[6] if len(defn) > 6 else {}))
        except:
            print('Exceptopn in class %s makefield using defn:' % type(self).__name__, defn)
            raise

    def makefieldset(self, pagelist, fieldlist=None):
        return {self.prefix+fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}

class webbasemotor(webgroup):
    def __init__(self, webdefs):
        super().__init__(webdefs=webdefs, prefix=self.name+'_')

    def makefieldset(self, pagelist, fieldlist=None):
        basefields = {fname: (self.makefield(pagelist, self.webdefs[fname]), self.webdefs[fname][4]) for fname in (self.webdefs.keys() if fieldlist is None else fieldlist)}
        for stepname in self.stepmodenames:
            stepdeffields=getattr(self.stepmodes,stepname).makefieldset(pagelist, fieldlist=None)
            basefields.update(stepdeffields)
        return basefields

stepchipmotorfields=(
    (pagelink.wwenum, 'usercmd',    myagents.user,  'next command',    tablefielddropdnhtml,       'next command for this motor'),
    (pagelink.wwlink, 'userpos',    myagents.user,  'target position', tablefieldinputhtml,        'target position for next goto command'),
    (pagelink.wwenum, 'userdir',    myagents.user,  'direction',       tablefielddropdnhtml,       'direction for next run command'),
    (pagelink.wwenum, 'userstepm',  myagents.user,  'step settings',   tablefielddropdnhtml,       'step settings to use for next goto or run command'),
    (pagelink.wwenum, 'opmode',     myagents.app,   'motor mode',      tablefielddropdnhtml,       'motor controller mode'),
    (pagelink.wwlink, 'targetrawpos',myagents.app, 'target position',  tablefieldinputhtml,        'target absolute position in microsteps'),
    (pagelink.wwlink, 'rawposn',    myagents.app,   'current position',tablefieldinputhtml,        
            'current absolute position in microsteps (does not vary with changes to microstep level)'),
    (pagelink.wwlink, 'drive_enable',myagents.NONE, 'drive enable pin',tablefieldinputhtml,        
            'gpio (broadcom) pin used to drive enable this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'direction',  myagents.NONE,  'direction pin',   tablefieldinputhtml,        
            'gpio (broadcom) pin used to set step direction for this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'step',       myagents.NONE,  'step pin',        tablefieldinputhtml,        
            'gpio (broadcom) pin used to step this motor', {'liveformat': '{wl.wable.pinno:d}'}),
    (pagelink.wwlink, 'holdstopped',myagents.user,  'stop hold time',  tablefieldinputhtml,        
            'when motor stops, disable output current after this time (in seconds) - 0 means drive chip stays enabled'),
    (pagelink.wwlink, 'activestepm',myagents.app,   'active step mode',tablefielddropdnhtml,       'shows step mode in use'),
)

stepchipmotorfieldindex={f[1]: f for f in stepchipmotorfields}
 
class webstepchipmotor(steppers.A4988stepper, webbasemotor):
    def __init__(self,app, value, name, loglevel):
        wables=[  # some extra fields used just for the ui
            ('usercmd',     wv.enumWatch,   self.commands[0],   False,  {'vlist': self.commands}),  # user preset command
            ('userpos',     wv.intWatch,    0,                  False),                             # user target pos for goto mode
            ('userdir',     wv.enumWatch,   'fwd',              False,  {'vlist': ('fwd', 'rev')}), # user direction for run mode
        ]
        steppers.A4988stepper.__init__(self, wabledefs=wables, app=app, value=value, name=name, loglevel=loglevel)
        webbasemotor.__init__(self, webdefs=stepchipmotorfieldindex)

unimotorfields=(
    (pagelink.wwenum, 'usercmd',    myagents.user,  'next command',    tablefielddropdnhtml,       'next command for this motor'),
    (pagelink.wwlink, 'userpos',    myagents.user,  'target position', tablefieldinputhtml,        'target position for next goto command'),
    (pagelink.wwenum, 'userdir',    myagents.user,  'direction',       tablefielddropdnhtml,       'direction for next run command'),
    (pagelink.wwenum, 'userstepm',  myagents.user,  'step settings',   tablefielddropdnhtml,       'step settings to use for next goto or run command'),
    (pagelink.wwenum, 'opmode',     myagents.app,   'motor mode',      tablefielddropdnhtml,       'motor controller mode'),
    (pagelink.wwlink, 'targetrawpos',myagents.app, 'target position',  tablefieldinputhtml,        'target absolute position in microsteps'),
    (pagelink.wwlink, 'rawposn',    myagents.app,   'current position',tablefieldinputhtml,        
            'current absolute position in microsteps (does not vary with changes to microstep level)'),
    (pagelink.wwlink, 'holdstopped',myagents.user,  'stop hold time',  tablefieldinputhtml,        
            'when motor stops, disable output current after this time (in seconds) - 0 means drive chip stays enabled'),
    (pagelink.wwlink, 'activestepm',myagents.app,   'active step mode',tablefielddropdnhtml,       'shows step mode in  use'),
    (pagelink.wwlink, 'drive_pins', myagents.NONE,  'pins to chip'    ,tablefieldinputhtml,        'pins used to drive output chip'),
)

unimotorfieldindex={f[1]: f for f in unimotorfields}


class webunimotor(steppers.directstepper, webbasemotor):
    def __init__(self, app, value, name, loglevel):
        wables=[  # some extra fields used just for the ui
            ('usercmd',     wv.enumWatch,   self.commands[0],   False,  {'vlist': self.commands}),  # user preset command
            ('userpos',     wv.intWatch,    0,                  False),                             # user target pos for goto mode
            ('userdir',     wv.enumWatch,   'fwd',              False,  {'vlist': ('fwd', 'rev')}), # user direction for run mode
        ]
        steppers.directstepper.__init__(self, wabledefs=wables, app=app, value=value, name=name, loglevel=loglevel)
        webbasemotor.__init__(self, webdefs=unimotorfieldindex)

class webstepsimple(intervalgen.stepgensimple, webgroup):
    def __init__(self, **kwargs):
        intervalgen.stepgensimple.__init__(self, **kwargs)
        sfi={'stephead': (pagelink.wwdummy,'settings',   myagents.NONE,   'stepper settings',       tablefieldstatic,           None, {'value':self.name}),}
        sfi.update(stepfieldssimplex)
        webgroup.__init__(self, webdefs=sfi, prefix=self.name)

stepfieldssimple=(   
    (pagelink.wwenum, 'usteplevel', myagents.user,  'microstep level',          tablefielddropdnhtml,       'microsteplevel used'),
    (pagelink.wwlink, 'step',       myagents.user,  'step interval',            tablefieldinputhtml,        'step interval in seconds'),
)
stepfieldssimplex={f[1]: f for f in stepfieldssimple}

class webstepconstacc(intervalgen.stepconstacc, webgroup):
    def __init__(self, **kwargs):
        intervalgen.stepconstacc.__init__(self, **kwargs)
        sfi={'stephead': (pagelink.wwdummy,'settings',   myagents.NONE,   'stepper settings',       tablefieldstatic,           None, {'value':self.name}),}
        sfi.update(stepfieldsconstacc)
        webgroup.__init__(self, webdefs=sfi, prefix=self.name)

sfconstacc=(
    (pagelink.wwenum, 'usteplevel', myagents.user,  'microstep level',          tablefielddropdnhtml,       'microsteplevel used'),
    (pagelink.wwlink, 'startstep',  myagents.user,  'start step',               tablefieldinputhtml,        'initial step interval in seconds'),
    (pagelink.wwlink, 'minstep',    myagents.user,  'fastest step',             tablefieldinputhtml,        'minimum step interval in seconds'),
    (pagelink.wwlink, 'slope',      myagents.user,  'slope',                tablefieldinputhtml,        'acceleration factor'),
)
stepfieldsconstacc={f[1]: f for f in sfconstacc}

classlookups = {  # maps the class names in json settings file to actual classes
    'ULN'       : webunimotor,
    'A4988'     : webstepchipmotor,
    'onespeed'  : webstepsimple,
    'constramp' : webstepconstacc,
}