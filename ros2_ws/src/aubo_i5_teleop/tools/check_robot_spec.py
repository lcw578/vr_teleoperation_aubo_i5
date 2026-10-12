#!/usr/bin/env python3
"""Check generated model limits against config/robot_spec.yaml without ROS/DDS."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

PKG=Path(__file__).resolve().parents[1]
ROOT=PKG.parents[2]
sys.path.insert(0,str(PKG/'scripts'))
from robot_parameters import GROUP
spec=yaml.safe_load((PKG/'config/robot_spec.yaml').read_text())
lo=spec['position_limit_rad']; velocities=spec['velocity_limits_rad_s']
limits=yaml.safe_load((PKG/'config/joint_limits.yaml').read_text())['joint_limits']
model=ET.parse(PKG/'urdf/aubo_i5_model.urdf').getroot()
for i,name in enumerate(GROUP):
 j=model.find(f"joint[@name='{name}']/limit")
 assert abs(float(j.get('lower'))+lo)<1e-10 and abs(float(j.get('upper'))-lo)<1e-10,name
 assert abs(float(j.get('velocity'))-velocities[i])<1e-10,name
 assert limits[name]['min_position']==-lo and limits[name]['max_position']==lo,name
 assert limits[name]['max_velocity']==velocities[i],name
for filename in ['scene_aubo_i5.xml','scene_aubo_i5_rg.xml','actuators_position.xml','actuators_position_rg.xml','actuators_position_demo.xml']:
 root=ET.parse(ROOT/'assets/aubo_i5'/filename).getroot()
 for element in root.iter():
  if element.get('name') in GROUP:
   attribute='range' if element.tag=='joint' else 'ctrlrange' if element.tag=='position' else None
   if attribute:
    values=list(map(float,element.get(attribute).split()))
    assert abs(values[0]+lo)<1e-10 and abs(values[1]-lo)<1e-10,(filename,element.get('name'))
print('i5/CB4 manual limits consistent across URDF, MoveIt and active MJCF/actuators')
