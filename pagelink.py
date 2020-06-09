"""
This module provides the means to manage linkage between a web page (and the specific fields on the web page) and app vars.

This allows individual fields on the webpage to be updated by the user to be immediately passed back to the webserver and update
watchables in the app, and also updates by the app to any watchables are batched up and forwarded to the web browser to update the
web page.
"""
import time, threading, datetime, math
from pootlestuff import watchables as wv

class pageupdatelist():
    """
    when a web page is built using 'pagemaker' this class is instantiated to hold the links between app var's and
    fields on the web page. Fields on the web page that can update the app and vice-versa are given id's that uniquely
    identify the the field on the web page and the var to which they are connected. Vars that need to dynamically update fields on the
    web page use an onchange notification to alert this class, and periodically all outstandng updates are gathered and sent to the
    web page.
    
    There are 2 maps:
        var -> field id (which can be 1 to many)
        field id -> var (several fields can link to same var)
    """
    def __init__(self, pageid, useragent='user', userupdate=wv.myagents.user, liveupdate=wv.myagents.app):
        self.pageid=pageid
#        self.useragent=useragent
        self.userupdate=userupdate
        self.liveupdate=liveupdate
        self.var_field_map={}  # maps vars to 1 or more fields on the web page
        self.field_var_map={}  # maps screen fields to vars 
        self.created=time.time()
        self.lastused=self.created  # keep a last used timestamp so we can run through the list and discard unused lists
        self.lock=threading.Lock()
        self.pendingupdates=set()
        self.nextid=1

    def add_field(self, upper, userupa=None, liveupa=None, updators=None, usefid=None):
        """
        called as the html for a field is created....
        If the field is dynamic (user can update or server can update) then allocate an id, otherwise we don't need one

        upper       : updater instance that handles updates between a var and a screen field
        
        userupa     : None if user cannot update via web page, otherwise the name to use as the agent when the user changes a field
        
        liveupa     : None if application updates to the field are not dynamically sent to update the web page, otherwise the 
                      agent name that will be used when such updates happen

        updators    : flag class with required update capabilities
        
        usefid      : allows a fixed id to be nominated for the field. This will be returned if the field updates, or None
                      is returned if the field does not change on the page.
        
        returns None if the web page field has no id, otherwise the id of the field
        """
        if userupa or liveupa or updators:
            if usefid is None:
                fid=str(self.nextid)
                self.nextid +=1
            else:
                fid=usefid
            if userupa or updators & self.userupdate:
                self.field_var_map[fid]=upper
            if liveupa or updators & self.liveupdate:
                assert not upper in self.var_field_map
                self.var_field_map[upper]=fid
            return fid
        else:
            return None

    def applyUpdate(self, fid, value):
        return self.field_var_map[fid[0]].websetter(value)

    def haslinks(self):
        return len(self.var_field_map) > 0 or len(self.field_var_map) > 0

    def closelist(self):
        for upper in self.var_field_map.keys():
            upper.drop()

    def markpending(self, upd):
        with self.lock:
            self.pendingupdates.add(upd)

    def getupdates(self):
        with self.lock:
            toupdate=list(self.pendingupdates)
            self.pendingupdates.clear()
        self.lastused=time.time()
        return [upd.getUpdate() for upd in toupdate] if toupdate else 'kwac'

    def hasexpired(self):
        return (self.lastused+30) < time.time()

class wwlink():
    """
    the base class that links a watchable to a screen field.
    
    This is used to generate the initial html when building the page, as well as enabling dynamic update
    of the web page from the watchable or the watchable from the web page.
    """
    def __init__(self, wable, pagelist, userupa=None, liveupa=None, updators=None, liveformat=None,
                 fixedfid=None, label=None, shelp=None, doclink=None):
        """
        sets up an updator that links an app var to a field on a web page
        
        wable       : the watchable linked
        
        pagelist    : the pageupdatelist instance this instance is used by
        
        liveupa     : the agent that will trigger a dynamic update on the web page (or None if no dynamic update)
    
        userupa     : the agent that will be used to update the watchable when the user changes a field on the web page (or none if
                      the user cannot update this field

        updators    : specifies updates using enum flags - livepa and userpa are ignored

        liveformat  : a string to be used to format the new value when sent to the web browser, None means no formatting applied
                      liveformat.format is applied to this string with varval containing getValue() result from the watchable

        fixedfid    : id's for fields on screen can be allocated dynamically for a page, or can be fixed using this parameter.

        label       : a label for the field

        shelp       : short help for the field
        
        doclink     : hyperlink to further documentation
        """
        assert userupa is None
        assert liveupa is None
        self.wable=wable
        self.pagelist=pagelist
        self.liveformat=liveformat
        if isinstance(updators, wv.myagents):
            self.updators=updators
        else:
            if userupa is None:
               self.updators = liveupa
            else:
                self.updators=userupa
                if not liveupa is None:
                    self.updators |= self.liveupa
        if not label is None:
            self.label=label
        if not shelp is None:
            self.shelp=shelp
        if not doclink is None:
            self.doclink=doclink
        if self.updators:   # an id and class instance is only needed if there are going to be dynamic updates (in either direction)
            self.fid=self.pagelist.add_field(userupa=userupa, liveupa=liveupa, updators=updators, upper=self, usefid=fixedfid)
            if self.updators & self.pagelist.liveupdate:
                self.wable.log(wv.loglvls.DEBUG,'setting notify for var value %s' % self.wable.getValue())
                self.wable.addNotify(self.varchanged, self.pagelist.liveupdate)

    def varchanged(self, watched=None, agent=None, newValue=None, oldValue=None):
        """
        triggered if the value is changed using liveupdate agent. Add this to the set of pending updates
        """
        self.pagelist.markpending(self)

    def drop(self):
        """
        when the updatelist times out, we ditch the notifications set in the constructor
        """
        if self.updators & self.pagelist.liveupdate:
            self.wable.dropNotify(self.varchanged, self.pagelist.liveupdate)
        
    def getUpdate(self):
        """
        called to get the current value when an update list about to be sent to the web page
        """
        return self.fid, self.webvalue

    clickact='onchange'

    def webitem(self, fformat):
        """
        returns the full html for field with a label, short help etc.
        """
        attrs={'id': str(self.fid)} if hasattr(self, 'fid') else {}
        if self.updators and self.updators & self.pagelist.userupdate:
            if wv.wflags.DISABLED in self.wable.flags:
                attrs['disabled']='disabled'
            attrs[self.clickact]='appNotify(this, {})'.format(self.pagelist.pageid)
        attrstr=' '.join([('%s="%s"' % (ak, av)) for ak, av in attrs.items()])
        fullf = fformat['all'].format(
            wshelp = fformat['shelp'] if hasattr(self, 'shelp') else '',
            wlabel = fformat['label'] if hasattr(self, 'label') else '',
            wfield = fformat['fieldu'] if self.updators & self.pagelist.userupdate else fformat['fieldf'],
            whelp  = fformat['fhelp'] if hasattr(self, 'doclink') else '',
        )
        if hasattr(self,'label') and self.label=='current analogue gain':
            print('updators', self.updators, 'pagey', self.pagelist.userupdate, 'combined', self.updators & self.pagelist.userupdate)
        return fullf.format(wl=self, tattrs=attrstr)

    @property
    def varvalue(self):
        return self.wable.getValue()

    @property
    def webvalue(self):
        """
        returns a string representation of the field's current value, uses a format it available
        """
        return str(self.varvalue) if self.liveformat is None else self.liveformat.format(wl=self)

    def websetter(self, webval):
        """
        called when the user updates a field on the web page that has dynamic updating set.
        
        This default action checks the update value is a 1 entry list and then tries to update the watchable 
        """
        if len(webval) == 1:
            try:
                self.wable.setValue(webval[0], self.pagelist.userupdate)
            except ValueError:
                return {'OK': False, 'fail': 'invalid value for this field (%s)' % webval[0]}
            return {'OK': True, 'value': self.webvalue}
        else:
            return {'OK' : False, 'fail': 'bad request'}

class wwdummy(wwlink):
    """
    A degnerate link that just enables a heading or other fixed column heading to be inserted
    """
    def __init__(self, value, **kwargs):
        self.value=value
        super().__init__(**kwargs)

class wwtime(wwlink):
    """
    The value in the associated wable is a timestamp, the fieldformat is used with strftime to produce the string
    """
    @property
    def webvalue(self):
        """
        returns a string representation of the field's current value
        """
        val=self.wable.getValue()
        if math.isnan(val):
            return 'never'
        else:
            return datetime.datetime.fromtimestamp(self.wable.getValue()).strftime(self.liveformat)

class wwenum(wwlink):
    """
    uses a dropdown for the enumeration (if the user can change it)
    """
    @property
    def webvalue(self):
        if self.updators & self.pagelist.userupdate:
            cval=self.wable.getValue()
            opts=[
                '<option value="{oval}"{sel}>{odisp}</option>'.format(
                        oval=optname, odisp=optname, sel=' selected' if optname==cval else '')
                                for optname in self.wable.vlist]
            return ''.join(opts)
        return super().webvalue

class wwenumbtn(wwlink):
    """
    used for buttons that cycle though a range of values
    """
    clickact='onclick'
    def websetter(self, webval):
        """
        called when the user updates a field on the web page that has dynamic updating set.
        
        This default action checks the update value is a 1 entry list and then tries to update the watchable 
        """
        if len(webval) == 1:
            try:
                self.wable.increment(self.pagelist.userupdate)
            except ValueError:
                return {'OK': False, 'fail': 'invalid value for this field (%s)' % webval[0]}
            return {'OK': True, 'value': self.webvalue}
        else:
            return {'OK' : False, 'fail': 'bad request'}

class wwbutton(wwlink):
    clickact='onclick'
    def websetter(self, webval):
        if len(webval) == 1:
            try:
                self.wable.setValue(1, self.pagelist.userupdate)
            except ValueError:
                return {'OK': False, 'fail': 'invalid value for this field (%s)' % webval[0]}
            return {'OK': True, 'value': self.webvalue}
        else:
            return {'OK' : False, 'fail': 'bad request'}
