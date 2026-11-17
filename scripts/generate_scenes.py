#!/usr/bin/env python3
"""Generate synthetic test scenes; no prior navigation map is generated or loaded."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/dynamic_nav_sim'

def add(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node

def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)

def scene_layout(name):
    """Single source of truth for geometry, start/goal and obstacle schedules."""
    boxes=[('north',0,3.1,12.4,.2),('south',0,-3.1,12.4,.2),('east',6.1,0,.2,6.4),('west',-6.1,0,.2,6.4),('shelf_north',-1.8,2.35,1.4,.7),('shelf_south',2.0,-2.35,1.4,.7)]
    gates=[];oriented_boxes=[];start=[-4.,0.];goal=[4.,0.];bounds=[-7.,7.,-4.,4.];half_width=3.
    obstacles=[dict(name='crossing_obstacle',x=0.,start_y=1.6,radius=.45,speed=.6,hold_seconds=8.,trigger_x=-2.2,enabled=name!='narrow_static')]
    if name.startswith('narrow_'):
        boxes=boxes[:4]
        gates=[dict(name='entry',x=-1.9,y=0.,width=1.05,length=.8),dict(name='exit',x=2.,y=.3,width=1.,length=.8)]
        boxes.append(('island',.3,-2.35,.8,.8))
    elif name=='long_corridor':
        half_width=2.4
        boxes=[('north',0,2.5,40.4,.2),('south',0,-2.5,40.4,.2),('east',20.1,0,.2,5.2),('west',-20.1,0,.2,5.2),
               # Extend the two downstream crates inward, keeping their outer edges fixed.
               ('crate_entry',-16.2,1.0,.8,1.2),('crate_middle',-6.0,-.6,1.,2.0),('crate_exit',10.5,.55,1.,2.2)]
        gates=[dict(name='passage_1',x=-13.5,y=-.35,width=.84,length=.16,kind='door',hinge_side='south',opens_toward='goal',open_angle_deg=80.),
               dict(name='passage_2',x=0.,y=.4,width=1.10,length=2.),
               dict(name='passage_3',x=15.,y=-.3,width=.84,length=.16,kind='door',hinge_side='south',opens_toward='start',open_angle_deg=80.)]
        start=[-18.5,0.];goal=[18.5,0.];bounds=[-21.,21.,-3.4,3.4]
        obstacles=[dict(name='crossing_obstacle',x=-9.5,start_y=1.75,radius=.4,speed=.55,hold_seconds=0.,trigger_x=-12.,enabled=True,motion='stop_at_center'),
                   dict(name='crossing_obstacle_2',x=6.,start_y=-1.75,radius=.4,speed=.5,hold_seconds=0.,trigger_x=3.8,enabled=True,motion='stop_at_center')]
    for gate in gates:
        label,cx,cy,gap,length=(gate[k] for k in ('name','x','y','width','length'))
        lower,upper=cy-gap/2,cy+gap/2
        boxes.extend([(label+'_south',cx,(-half_width+lower)/2,length,lower+half_width),
                      (label+'_north',cx,(upper+half_width)/2,length,half_width-upper)])
        if gate.get('kind')=='door':
            sign=1 if gate['hinge_side']=='north' else -1
            direction=1 if gate['opens_toward']=='goal' else -1
            angle=math.radians(gate['open_angle_deg'])
            yaw=math.atan2(-sign*math.cos(angle),direction*math.sin(angle))
            hinge=[cx+direction*length/2,cy+sign*gap/2]
            oriented_boxes.append(dict(name=label+'_door_leaf',x=hinge[0]+gap/2*math.cos(yaw),y=hinge[1]+gap/2*math.sin(yaw),sx=gap,sy=.035,yaw=yaw,height=2.05,hinge=hinge))
    static_polygons=[]
    for item in [dict(name=n,x=x,y=y,sx=sx,sy=sy,yaw=0.) for n,x,y,sx,sy in boxes]+oriented_boxes:
        c,s=math.cos(item['yaw']),math.sin(item['yaw'])
        points=[[item['x']+c*x-s*y,item['y']+s*x+c*y] for x,y in [(-item['sx']/2,-item['sy']/2),(item['sx']/2,-item['sy']/2),(item['sx']/2,item['sy']/2),(-item['sx']/2,item['sy']/2)]]
        static_polygons.append(dict(name=item['name'],points=points))
    return dict(name=name,boxes=boxes,oriented_boxes=oriented_boxes,static_polygons=static_polygons,gates=gates,start=start,goal=goal,map_bounds=bounds,
                recommended_timeout=360. if name=='long_corridor' else 90.,dynamic_obstacles=obstacles)

def contact_sensor(link,name,collision,topic):
    sensor=add(link,'sensor',name=name+'_contact',type='contact')
    add(sensor,'always_on','true');add(sensor,'update_rate',50)
    add(add(sensor,'contact'),'collision',collision)
    bumper=add(sensor,'plugin',name=name+'_bumper',filename='libgazebo_ros_bumper.so')
    add(add(bumper,'ros'),'remapping','bumper_states:='+topic)
    add(bumper,'frame_name','world')

def scene(scene_name="dynamic_crossing"):
    layout=scene_layout(scene_name);boxes=layout['boxes']
    (PKG/'config'/f'{scene_name}_scene.json').write_text(json.dumps(layout,indent=2)+'\n')
    sdf=ET.Element('sdf',version='1.6'); w=add(sdf,'world',name=scene_name)
    add(w,'gravity','0 0 -9.81')
    physics=add(w,'physics',name='default_physics',type='ode')
    add(physics,'max_step_size',.002); add(physics,'real_time_update_rate',500)
    sun=add(w,'light',name='sun',type='directional'); add(sun,'pose','0 0 10 0 0 0');add(sun,'diffuse','.8 .8 .8 1');add(sun,'direction','-.5 .2 -1')
    ground=add(w,'model',name='ground_plane');add(ground,'static','true');l=add(ground,'link',name='ground')
    for tag in ('collision','visual'):
        plane=add(add(add(l,tag,name='ground_'+tag),'geometry'),'plane');add(plane,'normal','0 0 1');add(plane,'size','50 30')
    for name,x,y,sx,sy in boxes:
        is_door_wall=any(gate.get('kind')=='door' and name in (gate['name']+'_north',gate['name']+'_south') for gate in layout['gates'])
        height=2.2 if is_door_wall else 1.2
        m=add(w,'model',name=name); add(m,'static','true');add(m,'pose',f'{x} {y} {height/2} 0 0 0');l=add(m,'link',name='body')
        for tag in ('collision','visual'):
            el=add(l,tag,name=name+'_'+tag);add(add(add(el,'geometry'),'box'),'size',f'{sx} {sy} {height}')
            if tag=='visual': add(add(el,'material'),'diffuse','.8 .55 .15 1' if name.startswith('crate') else '.55 .6 .65 1')
        contact_sensor(l,name,name+'_collision','/static_contacts')
    for door in layout['oriented_boxes']:
        name=door['name'];m=add(w,'model',name=name);add(m,'static','true')
        add(m,'pose',f"{door['x']} {door['y']} {door['height']/2} 0 0 {door['yaw']}")
        link=add(m,'link',name='body')
        for tag in ('collision','visual'):
            el=add(link,tag,name=name+'_'+tag)
            add(add(add(el,'geometry'),'box'),'size',f"{door['sx']} {door['sy']} {door['height']}")
            if tag=='visual':add(add(el,'material'),'diffuse','.22 .55 .52 1')
        handle=add(link,'visual',name='handle');add(handle,'pose',f"{door['sx']*.35} -.045 0 0 0 0")
        add(add(add(handle,'geometry'),'box'),'size','.12 .035 .025');add(add(handle,'material'),'diffuse','.2 .2 .22 1')
        contact_sensor(link,name,name+'_collision','/static_contacts')
    # Door frame inner faces coincide with the stated 0.84 m opening.
    for gate in layout['gates']:
        if gate.get('kind')!='door':continue
        model=add(w,'model',name=gate['name']+'_frame');add(model,'static','true');link=add(model,'link',name='frame')
        for side in (-1,1):
            visual=add(link,'visual',name='jamb_'+str(side));add(visual,'pose',f"{gate['x']} {gate['y']+side*(gate['width']/2+.02)} 1.05 0 0 0")
            add(add(add(visual,'geometry'),'box'),'size',f"{gate['length']+.02} .04 2.1");add(add(visual,'material'),'diffuse','.15 .18 .2 1')
        for tag in ('visual','collision'):
            el=add(link,tag,name='lintel_'+tag);add(el,'pose',f"{gate['x']} {gate['y']} 2.125 0 0 0")
            add(add(add(el,'geometry'),'box'),'size',f"{gate['length']+.02} {gate['width']+.08} .05")
            if tag=='visual':add(add(el,'material'),'diffuse','.15 .18 .2 1')
    for index,obstacle in enumerate(layout['dynamic_obstacles']):
        name=obstacle['name']
        m=add(w,'model',name=name);add(m,'pose',f"{obstacle['x']} {obstacle['start_y']} .5 0 0 0")
        l=add(m,'link',name='body');add(l,'gravity','false');add(l,'kinematic','true')
        inert=add(l,'inertial');add(inert,'mass',10);it=add(inert,'inertia')
        for k,v in {'ixx':1,'iyy':1,'izz':1,'ixy':0,'ixz':0,'iyz':0}.items():add(it,k,v)
        for tag in ('collision','visual'):
            el=add(l,tag,name='obstacle_'+tag);c=add(add(el,'geometry'),'cylinder');add(c,'radius',obstacle['radius']);add(c,'length',1.0)
            if tag=='visual':add(add(el,'material'),'diffuse','.9 .15 .1 1' if index==0 else '.65 .15 .8 1')
        contact_sensor(l,name,'obstacle_collision','/obstacle_contacts')
    plugin=add(w,'plugin',name='gazebo_ros_state',filename='libgazebo_ros_state.so');add(add(plugin,'ros'),'namespace','/gazebo');add(plugin,'update_rate',30)
    if scene_name=='long_corridor':
        camera=add(add(w,'gui',fullscreen='0'),'camera',name='overview')
        add(camera,'pose','0 -32 35 0 .83 1.57');add(camera,'view_controller','orbit')
    write_xml(sdf,PKG/'worlds'/f'{scene_name}.world')


if __name__ == '__main__':
    for name in ('dynamic_crossing','narrow_static','narrow_dynamic','long_corridor'):
        scene(name)
