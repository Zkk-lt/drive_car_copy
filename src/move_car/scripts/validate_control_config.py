#!/usr/bin/env python3
"""Validate research configuration; no ROS parameter writes."""
import argparse
import json
from control_research.config import load_config, TERRAINS
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('config_dir');p.add_argument('--terrain',choices=TERRAINS,default='unknown')
if __name__=='__main__':
 a=p.parse_args();c=load_config(a.config_dir,a.terrain)
 print(json.dumps({'terrain':c['terrain'],'config_hash':c['config_hash'],'valid':True,
                   'applied_to_controller':False},indent=2))
