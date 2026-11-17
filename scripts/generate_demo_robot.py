#!/usr/bin/env python3
"""Original primitive-only teaching robot; dimensions are chosen for this demo.

This generator reads no external model, mesh, CAD, or hardware calibration.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import yaml

PKG = Path(__file__).resolve().parents[1] / 'src/dynamic_nav_sim'
FOOTPRINT = [[-.32, -.28], [.32, -.28], [.32, .28], [-.32, .28]]


def add(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def rigid_link(robot, name, shape, dimensions, mass, xyz, color, rpy='0 0 0'):
    link = add(robot, 'link', name=name)
    inertial = add(link, 'inertial')
    add(inertial, 'origin', xyz=xyz)
    add(inertial, 'mass', value=str(mass))
    if shape == 'box':
        x, y, z = map(float, dimensions['size'].split())
        inertia = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
    elif shape == 'sphere':
        inertia = [2*mass*float(dimensions['radius'])**2/5]*3
    else:  # Wheel cylinder axis is aligned with Y.
        radius, length = float(dimensions['radius']), float(dimensions['length'])
        transverse = mass*(3*radius*radius+length*length)/12
        inertia = [transverse, mass*radius*radius/2, transverse]
    add(inertial, 'inertia', ixx=str(inertia[0]), iyy=str(inertia[1]),
        izz=str(inertia[2]), ixy='0', ixz='0', iyz='0')
    for kind in ('visual', 'collision'):
        element = add(link, kind, name=name+'_'+kind)
        add(element, 'origin', xyz=xyz, rpy=rpy)
        add(add(element, 'geometry'), shape, **dimensions)
        if kind == 'visual':
            add(add(element, 'material', name=name+'_color'), 'color', rgba=color)
    return link


def joint(robot, name, parent, child, xyz, kind='fixed'):
    item = add(robot, 'joint', name=name, type=kind)
    add(item, 'parent', link=parent)
    add(item, 'child', link=child)
    add(item, 'origin', xyz=xyz)
    if kind == 'continuous':
        add(item, 'axis', xyz='0 1 0')
        add(item, 'limit', effort='12', velocity='20')
        add(item, 'dynamics', damping='.01', friction='0')


def main():
    robot = ET.Element('robot', name='generic_demo_robot')
    add(robot, 'link', name='base_footprint')
    rigid_link(robot, 'base_link', 'box', {'size':'.60 .44 .12'}, 8.,
               '0 0 .16', '.12 .45 .8 1')
    joint(robot, 'base_joint', 'base_footprint', 'base_link', '0 0 0')
    for side, y in [('left', .24), ('right', -.24)]:
        name = side+'_wheel'
        rigid_link(robot, name, 'cylinder', {'radius':'.08', 'length':'.04'}, .3,
                   '0 0 0', '.08 .09 .12 1', rpy=f'{math.pi/2} 0 0')
        joint(robot, name+'_joint', 'base_link', name, f'0 {y} .08', 'continuous')
        gazebo = add(robot, 'gazebo', reference=name)
        for key, value in {'mu1':1., 'mu2':1., 'kp':1000000., 'kd':100.,
                           'minDepth':.001, 'maxVel':.1, 'material':'Gazebo/Black'}.items():
            add(gazebo, key, value)
    for side, x in [('front', .23), ('rear', -.23)]:
        name = side+'_support'
        rigid_link(robot, name, 'sphere', {'radius':'.04'}, .1,
                   '0 0 0', '.2 .2 .2 1')
        joint(robot, name+'_joint', 'base_link', name, f'{x} 0 .04')
        gazebo = add(robot, 'gazebo', reference=name)
        add(gazebo, 'mu1', 0.)
        add(gazebo, 'mu2', 0.)
    rigid_link(robot, 'laser_link', 'sphere', {'radius':'.025'}, .05,
               '0 0 0', '.1 .8 .6 1')
    joint(robot, 'laser_joint', 'base_link', 'laser_link', '.20 0 .25')
    gazebo = add(robot, 'gazebo', reference='laser_link')
    sensor = add(gazebo, 'sensor', name='generic_laser', type='ray')
    add(sensor, 'always_on', 'true')
    add(sensor, 'update_rate', 10)
    ray = add(sensor, 'ray')
    scan = add(add(ray, 'scan'), 'horizontal')
    for key, value in {'samples':400, 'resolution':1, 'min_angle':-math.pi, 'max_angle':math.pi}.items():
        add(scan, key, value)
    ranges = add(ray, 'range')
    for key, value in {'min':.05, 'max':12., 'resolution':.01}.items():
        add(ranges, key, value)
    plugin = add(sensor, 'plugin', name='scan', filename='libgazebo_ros_ray_sensor.so')
    add(add(plugin, 'ros'), 'remapping', '~/out:=/scan_raw')
    add(plugin, 'output_type', 'sensor_msgs/LaserScan')
    add(plugin, 'frame_name', 'laser_link')
    add(add(robot, 'gazebo', reference='base_link'), 'material', 'Gazebo/Blue')
    drive = add(add(robot, 'gazebo'), 'plugin', name='diff_drive', filename='libgazebo_ros_diff_drive.so')
    parameters = {'update_rate':50, 'left_joint':'left_wheel_joint', 'right_joint':'right_wheel_joint',
                  'wheel_separation':.48, 'wheel_diameter':.16, 'max_wheel_torque':12,
                  'max_wheel_acceleration':1., 'publish_odom':'true', 'publish_odom_tf':'true',
                  'publish_wheel_tf':'false', 'odometry_frame':'odom',
                  'robot_base_frame':'base_footprint', 'odometry_source':1}
    for key, value in parameters.items():
        add(drive, key, value)
    states = add(add(robot, 'gazebo'), 'plugin', name='joint_states',
                 filename='libgazebo_ros_joint_state_publisher.so')
    add(states, 'update_rate', 30)
    for name in ('left_wheel_joint', 'right_wheel_joint'):
        add(states, 'joint_name', name)
    ET.indent(robot)
    ET.ElementTree(robot).write(PKG/'urdf/generic_robot.urdf', encoding='utf-8', xml_declaration=True)
    (PKG/'config/robot_geometry.json').write_text(json.dumps({
        'model':'synthetic_primitive_demo', 'source':'scripts/generate_demo_robot.py',
        'physical_envelope_m':[.60,.52], 'footprint_with_2cm_margin':FOOTPRINT,
        'note':'Invented educational geometry; not a real hardware model.'}, indent=2)+'\n')
    (PKG/'config/generic_sensors.json').write_text(json.dumps({
        'lidar':{'model':'ideal_simulated_laser', 'optical_xyz_in_sim_base_m':[.20,0.,.25],
                 'samples':400, 'update_rate_hz':10, 'max_range_m':12.}}, indent=2)+'\n')
    for path in (PKG/'config').glob('nav2*.yaml'):
        config = yaml.safe_load(path.read_text())
        config['controller_server']['ros__parameters']['FollowPath']['footprint_model']['vertices'] = str(FOOTPRINT)
        for name in ('local_costmap', 'global_costmap'):
            config[name][name]['ros__parameters']['footprint'] = str(FOOTPRINT)
        path.write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    main()
