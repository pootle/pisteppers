trfieldall      = '''<tr><td class="fieldlabel" {wshelp}>{wlabel}</td><td>{wfield}</td><td class="fieldhelp">{whelp}</td></tr>\n'''
spanfieldall    = '''<span {wshelp}>{wlabel}</span><span>{wfield}</span>'''
fieldlabel      =  '{wl.label}:'
fieldshelp      = 'title="{wl.shelp}"'
fieldfhelp      = '<a class="fieldlink" href="{wl.doclink}" rel="noopener noreferrer">details</a>'
fielduinput     = '<input type="text" value="{wl.webvalue}" style="width: 8em" {tattrs}/>' # for input fields that use input with type="text"
fieldnouser     = '<span {tattrs}>{wl.webvalue}</span>'

fieldudropdn= '''<select {tattrs} >{wl.webvalue}</select>'''  # for input fields that use a drop down selection

fieldcyclicbtn = '<div class="btnlike" {tattrs}>{wl.webvalue}</div>'

tablefieldinputhtml  = {
    'all'   : trfieldall,
    'label' : fieldlabel,
    'shelp' : fieldshelp,
    'fhelp' : fieldfhelp,
    'fieldu': fielduinput,
    'fieldf': fieldnouser, 
}

tablefielddropdnhtml = {
    'all'   : trfieldall,
    'label' : fieldlabel,
    'shelp' : fieldshelp,
    'fhelp' : fieldfhelp,
    'fieldu': fieldudropdn,
    'fieldf': fieldnouser, 
}

tablefieldcyclicbtndnhtml = {
    'all'   : trfieldall,
    'label' : fieldlabel,
    'shelp' : fieldshelp,
    'fhelp' : fieldfhelp,
    'fieldu': fieldcyclicbtn,
    'fieldf': fieldnouser, 
}

# wraps a set of fields as part of an existing table. 1 row with a header which has a button to open and close the group of fields that follow
# expects parameters:
#   style   : class used for all parts - typically used just to set background color
#   title   : title used in the header line of the section
#   flipid  : id used for the open close button and the main body of fields that are shown / hidden
#   fields  : bunch of html rows that form the main part of the section
tablesectwrapper = """
    <tr class="{style}">
        <td colspan="3" class="sectheadtext" >{title}</td>
        <td><span class="sectheadoc" onclick="flipme('{flipid}', '{flipid}x')"><img class="cbtn" id="{flipid}x" src="static/opendnarrow.svg" /></span></td>
    </tr>\n
    <tbody id="{flipid}" class="{style}" style="display: none;">
        {fields}
    </tbody>\n"""


spanfieldinputhtml = {
    'all'   : spanfieldall,
    'label' : fieldlabel,
    'shelp' : fieldshelp,
    'fhelp' : fieldfhelp,
    'fieldu': fielduinput,
    'fieldf': fieldnouser, 
}