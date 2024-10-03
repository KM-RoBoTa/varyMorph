import os
import re


def get_files(_dir, reg_str, sort=False, sort_type=None):
    """Returns all files with regex in a folder."""
    files = os.listdir(_dir)
    pattern = re.compile(reg_str)
    list_ = [os.path.join(_dir, f) for f in files if pattern.match(f)]
    if sort:
        if sort_type == "time":
            list_ = sorted(list_, key=os.path.getmtime)
        else:
            list_.sort()
    return list_

