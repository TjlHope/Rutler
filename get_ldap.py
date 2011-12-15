#!/usr/bin/env python

import sys
import cPickle as pickle
from collections import Mapping, Iterable

import ldap


class LDAPScrape(object):
    def __init__(self, uri='', *args, **kwds):
        """If uri given, initialise LDAP, else return empty object."""
        if uri:
            self.scrape_ldap(uri, *args, **kwds)

    def init_ldap(self, uri='', dn='', pw=''):
        """Initialise LDAP"""
        prot, sep, uri = uri.partition('://')
        if not sep:
            uri = prot
            prot = 'ldap'
        url, sep, port = uri.partition(':')
        if not sep:
            port = 389
        self.connection = ldap.init(url, port)
        result = self.connection.simple_bind_s(dn, pw)
        if result[0] != 97:
            raise ldap.LDAPError("Connection was not successfull: result = {0}"
                                    .format(result))

    def search(self, dn):
        self.result = self.connection.search_s(dn, ldap.SCOPE_SUBTREE)
        return self.result

    def get_people(self, location, refresh=False):
        if isinstance(location, Mapping):
            field, value = location.iteritems.next()
        elif ((not isinstance(location, basestring)) and 
              isinstance(location, Iterable)):
            field, value = location[:2]
        else:
            field = 'o'
            value = location
        value = value.lower()
        # get data for specified location
        if not refresh and hasattr(self, value):
            data = getattr(self, value)
        else:
            # Only want data dicts, not id strings
            data = [d[1] for d in self.result
                     if field in d[1] and value in ','.join(d[1]['o']).lower()]
        # store it in the class for later access.
        setattr(self, value, data)
        # get people from 'location'
        self.people = [d for d in data
                        if 'objectClass' in d and 'person' in d['objectClass']]
        return self.people

    def __del__(self):
        try:
            self.connection.unbind()
        except AttributeError:
            pass


def new_ic_ldap(return_data=False):
    url = "addressbook.ic.ac.uk"
    base_dn = "o=Imperial College,c=GB"
    ic_ldap = LDAPScrape(url)
    data = ic_ldap.search(base_dn)
    return data if return_data else ic_ldap


new_kv_template = """
def new_keyval(item, new_key, keep, remove, old_key):
    key = {_k}
    val = {_v}
    {_o}
    return key, val
"""


def rekey_dicts(d, new_key, keep=[], remove=[], old_key='_old',
                pop_new=True, expand=True):
    if keep:    # If key given in 'keep' and 'delete', prefer 'keep'
        remove = [i for i in remove if i not in keep]
    # Is 'd' a dictionary (mapping)?
    mapping = isinstance(d, Mapping)
    # value part of item depending on whether it's a mapping
    _v = "item[1]" if mapping else "item"
    # Function used to retrieve new key (pop vs get)
    _g = "pop" if pop_new else "__getitem__"
    # Expand single length values to internal value.
    _x = ("{x}[0] if not isinstance({x}, basestring) and len({x}) == 1 else {x}"
	    if expand else "{x}")
    # Generate new kes
    _k = "{_v}.{_g}(new_key)"
    # Only have complicated conditional list comprehension if needed.
    if keep or remove or expand:
        _n = "dict([(k, {_x}) for k, v in {_v}.iteritems(){o}{kp}{a}{rm}])"
        # Need the 'if' for either keep or remove
        o = " if " if keep or remove else ""
        # Only keep wanted keys
        kp = "k in keep" if keep else ""
        # Need the 'and' if both either keep and remove
        a = " and " if keep and remove else ""
        # Remove unwanted keys
        rm = "k not in remove" if remove else ""
        # Generate complete string
    else:
        _n = "{_v}"
    # Add old key to val if required
    _o = "val[old_key] = item[0]" if mapping and old_key is not None else ""
    # Generate new_keyval function
    new_kv_str = new_kv_template.format(_k=_x.format(x=_k.format(_v=_v, _g=_g)),
                                        _v=_n.format(_x=_x.format(x='v'), _v=_v,
                                                     o=o, kp=kp, a=a, rm=rm),
                                        _o=_o)
    exec new_kv_str
    # Iterate over 'd', generating new (key, val) pairs.
    nd = dict([new_keyval(i, new_key, keep, remove, old_key)
                    for i in (d.iteritems() if mapping else d)])
    return nd


def map_people_to_room(people):
    pwr = {}
    for p in ee_people:
        l = p['physicalDeliveryOfficeName'][0].split(', ')
        if len(l) > 2 and l[1] == 'Electrical Engineering':
            for el in l:
                el = el.strip('()[]{}:,.')
                if el[1:-1].isdigit():
                    pwr[p['cn'][0]] = el.upper()
    return pwr


if __name__ == '__main__':
    # Get LDAP user data...
    pickle_addressbook = 'Imperial_addressbook.pkl'
    pickle_ee_people = 'Imperial_elec.pkl'
    if len(sys.argv) > 1:
	if sys.argv[1] == 'download':
	    # ... from server
	    ic_ldap = new_ic_ldap()
	    # Save local copy
	    with open(pickle_addressbook, 'w') as fl:
		pickle.dump(ic_ldap.result, fl)
	elif sys.argv[1] == 'new':
	    # ... from local copy
	    ic_ldap = LDAPScrape()
	    with open(pickle_addressbook, 'r') as fl:
		data = pickle.load(fl)
	    ic_ldap.result = data
	# Get it in a usable format...
	ee_people = ic_ldap.get_people("elec")
	with open(pickle_ee_people, 'w') as fl:
	    pickle.dump(ee_people, fl)
    else:
        ic_ldap = LDAPScrape()
	with open(pickle_ee_people, 'r') as fl:
	    ee_people = pickle.load(fl)
	    ic_ldap.people = ee_people
    print rekey_dicts(ee_people[:20], 'sn', keep=['sn', 'givenName', 'l'],
	    pop_new=False)
    sys.exit(0)
