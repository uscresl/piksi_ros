#!/usr/bin/env python
"""
This script reads the Piksi configuration parameters from the settings.yaml from piksi_tools' console:
https://github.com/swift-nav/piksi_tools/blob/master/piksi_tools/console/settings.yaml
Using these descriptions, it generates a dynamic_reconfigure cfg file for this package.
This script is not meant to run all the time, just if parameters on the piksi are changed/added.
The cfg file will be commited in this repo.
"""

PKG='piksi_ros'

import sys
import yaml
import os
import ast

import rospkg

from dynamic_reconfigure.parameter_generator import *


rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path(PKG)

from jinja2 import FileSystemLoader, Environment

env = Environment(loader=FileSystemLoader(os.path.join(PKG_PATH, 'dr_generation')))

template = env.get_template('PiksiDriverConfig.cfg.template')
print env

SKIP_EXPERT = True

IGNORE_GROUPS = ['system_info']

TYPE_MAP = {
    'double': ('double_t', float),
    'boolean': ('bool_t', bool),
    'integer': ('int_t', int),
    'float': ('double_t', float),
    'string': ('str_t', str),
}

ROOT_GROUP = 'gen'

def get_default(default, setting_type):
    if isinstance(default, str):
        if setting_type[0] == 'str_t':
            return default
        else:
            try:
                return ast.literal_eval(default)
            except:
                if setting_type[0] == 'bool_t':
                    return default.lower()=='true'
                else:
                    try:
                        return setting_type[1](default.split(' ')[0])
                    except:
                        return
    elif default is None:
        return setting_type[1]()
    else:
        return default

output = {'groups': [], 'params': [], 'enums': []}

print sys.argv

if len(sys.argv) < 2:
    sys.exit()

f = open(sys.argv[1], 'r')
settings = yaml.load(f)

groups = set()
for s in settings:
    if s['group'] in IGNORE_GROUPS:
        continue

    enum_name = None

    expert = False
    if 'expert' in s and s['expert']:
        expert = True
        if SKIP_EXPERT:
            print 'Skipping expert setting %s:%s' % (s['group'],s['name'])
            continue
        #s['group'] = 'expert_%s' % s['group']

    #print s
    if s['group'] not in groups:
        parent = ROOT_GROUP
        if expert:
            parent = 'expert'
        groups.add(s['group'])

        output['groups'].append({'name': s['group'], 'parent': parent})

    if s['type']:
        if s['type'].lower() in TYPE_MAP:
            setting_type = TYPE_MAP[s['type'].lower()]
        elif s['type'].lower() == 'enum':
            enum_name = '%s_enum' % s['name']
            fields = []
            for e in s['enumerated possible values'].split(','):
                fields.append({'name':e.strip(), 'type': 'str_t', 'desc': e.strip(), 'value': e.strip()})
            setting_type = TYPE_MAP['string']
            output['enums'].append({'name': enum_name, 'fields': fields, 'desc': 'An enum for %s' % s['name']})
        else:
            print 'Skipping unknown type: %s' % s['type']
            continue
    else:
        print s
        continue

    default = get_default(s['default value'], setting_type)

    output['params'].append({'name': s['name'], 'default': repr(default), 'group': s['group'], 'type': setting_type[0], 'level': 0, 'desc': s['Description'].replace("\"","\\\""), 'enum_name': enum_name})
        # Add group to cfg

output['root_group'] = ROOT_GROUP

template_rendered = template.render(output)

with open(os.path.join(PKG_PATH,'cfg', 'PiksiDriverConfig.cfg'), "wb") as fh:
    fh.write(template_rendered)
