#utility.py
#Nathan Zimmerberg (nhz2)
#7 MAY 2020
""" Utilities for estimator test file gen."""

from dulwich.repo import Repo
from dulwich import porcelain

def getgitinfo():
    """ Return a string of some useful if run inside a git repo"""
    try:
        r= Repo.discover('.')
        rpath= r.commondir()[:-5]
        gitinfo= {
            'branch': porcelain.active_branch(rpath).decode('utf-8'),
            'commit': r[r.head()].as_pretty_string().decode('utf-8'),
            'status': str(porcelain.status(repo=rpath)),
        }
        return str(gitinfo)
    except:
        return 'No git info found'