#!/usr/bin/env python

import sys
import cPickle as pickle
import csv

import ldap
import rospy


class Imperial_LDAP(object):
    def __init__(self, uri, dn='', pw=''):
        """Initialise LDAP, return LDAPObject."""
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
        #filt = '(objectclass=person)'
        self.result = self.connection.search_s(base_dn, ldap.SCOPE_SUBTREE)
        return self.result

    def __del__(self):
        self.connection.unbind()


if __name__ == '__main__':
    url = "addressbook.ic.ac.uk"
    base_dn = "o=Imperial College,c=GB"
    ic_ldap = Imperial_LDAP(url)
    users = ic_ldap.search(base_dn)
    with open('Imperial_addressbook.pk', 'w') as fl:
        pickle.dump(users, fl)
    sys.exit(0)
