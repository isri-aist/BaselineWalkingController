#! /usr/bin/env python

import sys
import yaml

def mergeDict(dict1, dict2):
    """Merge dict2 into dict1."""
    for k, v in dict2.items():
        if k in dict1 and isinstance(dict1[k], dict):
            mergeDict(dict1[k], v)
        else:
            dict1[k] = v

yaml_data = {}
for yaml_filename in sys.argv[1:]:
    with open(yaml_filename) as yaml_file:
        mergeDict(yaml_data, yaml.safe_load(yaml_file))

yaml.dump(yaml_data, sys.stdout, default_flow_style=False)
