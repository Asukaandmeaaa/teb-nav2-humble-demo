#!/usr/bin/env python3
"""Long-corridor layout and measured, multi-obstacle trajectory replay."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'src/dynamic_nav_sim/config'
FOOTPRINT=np.array(json.loads((CONFIG/'robot_geometry.json').read_text())['footprint_with_2cm_margin'])
COLORS=['#dc3545','#9d42b5']


def footprint(row):
    c,s=np.cos(row['robot_yaw']),np.sin(row['robot_yaw'])
    return FOOTPRINT @ np.array([[c,s],[-s,c]])+[row['robot_x'],row['robot_y']]


def obstacle_xy(data,index):
    prefix='obstacle' if index==0 else f'obstacle_{index+1}'
    return data[prefix+'_x'],data[prefix+'_y']


def draw_layout(ax,layout,labels=True):
    for name,x,y,sx,sy in layout['boxes']:
        ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,facecolor='#d59a35' if name.startswith('crate') else '#7a8490',edgecolor='white',lw=.3))
    door_names={door['name'] for door in layout.get('oriented_boxes',[])}
    for item in layout.get('static_polygons',[]):
        if item['name'] in door_names:ax.add_patch(Polygon(item['points'],facecolor='#287e79',edgecolor='#163f3c',lw=.6))
    for i,gate in enumerate(layout['gates'],1):
        detail=f"door, open {gate['open_angle_deg']:g} deg" if gate.get('kind')=='door' else f"{gate['length']:.1f} m long"
        if labels:ax.text(gate['x'],2.8,f"G{i}: {gate['width']:.2f} m opening\n{detail}",ha='center',fontsize=8)
    for i,spec in enumerate(layout['dynamic_obstacles']):
        ax.annotate('',xy=(spec['x'],0. if spec.get('motion')=='stop_at_center' else -spec['start_y']),xytext=(spec['x'],spec['start_y']),arrowprops=dict(arrowstyle='->',color=COLORS[i%len(COLORS)],lw=1.4,linestyle='--'))
        if labels:ax.text(spec['x'],-3.1,f'D{i+1}',ha='center',color=COLORS[i%len(COLORS)],weight='bold')
    start,goal=layout['start'],layout['goal']
    ax.scatter([start[0],goal[0]],[start[1],goal[1]],c=['#20934c','#1769aa'],s=32,zorder=5)
    if labels:
        ax.text(start[0],.3,'START',ha='center',fontsize=8)
        ax.text(goal[0],.3,'GOAL',ha='center',fontsize=8)
    ax.set(xlim=layout['map_bounds'][:2],ylim=(-3.5,3.7),aspect='equal',xlabel='x (m)',ylabel='y (m)')
    ax.grid(alpha=.15)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--trial',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'results/long_corridor_layout')
    parser.add_argument('--gif',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    report=json.loads(args.trial.read_text()) if args.trial else None
    layout=report['scene_geometry'] if report else json.loads((CONFIG/'long_corridor_scene.json').read_text())
    data=np.atleast_1d(np.genfromtxt(args.trial.with_suffix('.csv'),delimiter=',',names=True)) if report else None
    fig,ax=plt.subplots(figsize=(16,4.5));draw_layout(ax,layout)
    for index,spec in enumerate(layout['dynamic_obstacles']):
        ax.add_patch(Circle((spec['x'],spec['start_y']),spec['radius'],facecolor=COLORS[index],alpha=.55))
    ax.set_title(f"Corridor | 3 static crates + 3 narrow passages + 2 triggered obstacles\nLayout only: arrows show obstacle motion; start-to-goal separation = {layout['goal'][0]-layout['start'][0]:g} m")
    fig.tight_layout();fig.savefig(args.output/'scene.png',dpi=170);plt.close(fig)
    doors=[gate for gate in layout['gates'] if gate.get('kind')=='door']
    if doors:
        fig,axes=plt.subplots(1,len(doors),figsize=(6*len(doors),6),squeeze=False)
        for ax,gate in zip(axes[0],doors):
            draw_layout(ax,layout,labels=False)
            x,y=gate['x'],gate['y'];ax.set(xlim=(x-1.3,x+1.3),ylim=(y-1.1,y+1.1))
            ax.annotate('',xy=(x,y+gate['width']/2),xytext=(x,y-gate['width']/2),arrowprops=dict(arrowstyle='<->',color='#d04444'))
            ax.text(x+.1,y,f"{gate['width']*100:.0f} cm\nframe opening",color='#d04444',va='center',fontsize=9)
            ax.annotate('Travel toward goal',xy=(x+.8,y+.85),xytext=(x-.9,y+.85),arrowprops=dict(arrowstyle='->'),fontsize=9)
            ax.set_title(f"{gate['name']} | right-hand hinge | {gate['open_angle_deg']:g} deg open\nLeaf opens toward {gate['opens_toward']}; geometry view")
            if data is not None:ax.plot(data['robot_x'],data['robot_y'],color='#1769aa',lw=1.5)
        fig.tight_layout();fig.savefig(args.output/'doors.png',dpi=170);plt.close(fig)
    if data is None or not len(data):return
    fig=plt.figure(figsize=(16,8));grid=fig.add_gridspec(2,3,height_ratios=[1,1.3])
    overview=fig.add_subplot(grid[0,:]);draw_layout(overview,layout)
    overview.plot(data['robot_x'],data['robot_y'],color='#1769aa',lw=1.8)
    overview.set_title(f"Measured Gazebo trajectory | {report['action_status']} | {report['sampled_sim_seconds']:.1f} s | contacts={report['physical_contact_count']}\nDashed arrows are obstacle tracks at different times")
    edges=np.linspace(layout['start'][0]-1,layout['goal'][0]+1,4)
    for panel,(lo,hi) in enumerate(zip(edges[:-1],edges[1:])):
        ax=fig.add_subplot(grid[1,panel]);draw_layout(ax,layout,labels=False)
        ax.set(xlim=(lo,hi),ylim=(-2.7,2.7));ax.set_title(f'Section {panel+1}')
        ax.plot(data['robot_x'],data['robot_y'],color='#1769aa',lw=1.5)
        subset=data[(data['robot_x']>=lo)&(data['robot_x']<=hi)]
        for row in subset[::max(1,len(subset)//12)]:ax.add_patch(Polygon(footprint(row),fill=False,edgecolor='#1769aa',alpha=.4))
    fig.tight_layout();fig.savefig(args.output/'trajectory.png',dpi=170);plt.close(fig)
    if not args.gif:return
    fig,(overview,zoom)=plt.subplots(2,1,figsize=(13,7),gridspec_kw={'height_ratios':[1,1.8]})
    artists=[]
    for ax in (overview,zoom):
        draw_layout(ax,layout,labels=ax is overview)
        trail,=ax.plot([],[],color='#1769aa',lw=1.6)
        robot=Polygon(footprint(data[0]),color='#1769aa',alpha=.8);ax.add_patch(robot)
        obstacles=[]
        for i,spec in enumerate(layout['dynamic_obstacles']):
            x,y=obstacle_xy(data[0],i)
            obstacle=Circle((x,y),spec['radius'],color=COLORS[i],alpha=.9);ax.add_patch(obstacle);obstacles.append(obstacle)
        artists.append((trail,robot,obstacles))
    frames=np.append(np.arange(data['sim_time'][0],data['sim_time'][-1],.5),data['sim_time'][-1])
    def update(t):
        i=min(len(data)-1,np.searchsorted(data['sim_time'],t));row=data[i]
        for trail,robot,obstacles in artists:
            trail.set_data(data['robot_x'][:i+1],data['robot_y'][:i+1]);robot.set_xy(footprint(row))
            for j,obstacle in enumerate(obstacles):obstacle.center=obstacle_xy(row,j)
        centre=np.clip(row['robot_x'],layout['map_bounds'][0]+5,layout['map_bounds'][1]-5);zoom.set(xlim=(centre-5,centre+5),ylim=(-2.7,2.7))
        overview.set_title(f"Measured Gazebo replay (5x), t={t-data['sim_time'][0]:.1f} s | contacts={report['physical_contact_count']}")
        zoom.set_title('Robot-following view; blue polygon includes the 2 cm footprint margin')
        return []
    fig.tight_layout()
    FuncAnimation(fig,update,frames=frames,interval=100).save(args.output/'long_corridor_replay.gif',writer=PillowWriter(fps=10))
    plt.close(fig)

if __name__=='__main__':main()
