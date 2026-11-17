#!/usr/bin/env python3
"""Render actual public-robot trajectory and sampled planner paths for the README."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon, Circle, Rectangle
from matplotlib.lines import Line2D
from plot_corridor import draw_layout, footprint, obstacle_xy


def render(report_path, output):
    report = json.loads(report_path.read_text())
    data = np.atleast_1d(np.genfromtxt(report_path.with_suffix('.csv'), delimiter=',', names=True))
    layout = report['scene_geometry']
    plans = report.get('plan_samples', {})
    output.mkdir(parents=True, exist_ok=True)
    palette = {'trail':'#dd8c13', 'global':'#2780d9', 'local':'#009879', 'robot':'#163d65'}
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10,
                         'axes.spines.top':False, 'axes.spines.right':False})
    fig, (overview, detail) = plt.subplots(2, 1, figsize=(12, 6.5),
                                         gridspec_kw={'height_ratios':[1, 2.2]})
    fig.patch.set_facecolor('#f4f7fb')
    fig.subplots_adjust(top=.82, bottom=.18, hspace=.36, left=.065, right=.97)
    title = fig.text(.065, .95, 'TEB + ONLINE SLAM', size=20, weight='bold', color='#163d65')
    subtitle = fig.text(.065, .903, '', size=10, color='#51627a')
    footer = fig.text(.065, .035, '', size=10, color='#163d65')
    artists = []
    for ax in (overview, detail):
        ax.set_facecolor('white')
        draw_layout(ax, layout, labels=False)
        ax.tick_params(labelsize=8)
        ax.set_ylabel('y / m')
        trail, = ax.plot([], [], color=palette['trail'], lw=2, zorder=6)
        global_plan, = ax.plot([], [], color=palette['global'], lw=1.5, ls='--', alpha=.9, zorder=5)
        local_plan, = ax.plot([], [], color=palette['local'], lw=2.3, zorder=7)
        robot = Polygon(footprint(data[0]), fc=palette['robot'], ec='white', lw=.8, zorder=8)
        ax.add_patch(robot)
        obstacles = []
        for j, spec in enumerate(layout['dynamic_obstacles']):
            circle = Circle(obstacle_xy(data[0], j), spec['radius'],
                            fc=['#e44855', '#9552c4'][j], ec='white', lw=.8, zorder=7)
            ax.add_patch(circle)
            obstacles.append(circle)
        window = Rectangle((0, 0), 5, 5, fill=False, ec='#73839b', ls=':', lw=1, zorder=3)
        ax.add_patch(window)
        artists.append((trail, global_plan, local_plan, robot, obstacles, window))
    for i, gate in enumerate(layout['gates'], 1):
        overview.text(gate['x'], 2.9, f"G{i} / {gate['width']:.2f} m", ha='center', fontsize=8)
    overview.set(xlim=(-20.5, 20.5), ylim=(-2.8, 3.4), xlabel='')
    detail.set_xlabel('x / m')
    legend = [Line2D([0], [0], color=palette[key], lw=2, ls='--' if key=='global' else '-', label=label)
              for key, label in [('trail','Measured motion'), ('global','Global plan'), ('local','TEB local plan')]]
    legend.append(Line2D([0], [0], color='#73839b', ls=':', label='5 x 5 m local window'))
    fig.legend(handles=legend, loc='upper center', bbox_to_anchor=(.51, .885), ncol=4,
               frameon=False, fontsize=9)

    def sampled_path(kind, t):
        samples = plans.get(kind, [])
        # Select the most recent recorded message, never a future planned path.
        index = np.searchsorted([sample['sim_time'] for sample in samples], t, side='right')-1
        if index < 0 or samples[index]['frame'] != 'odom':
            return [], []
        points = np.asarray(samples[index]['points'])
        return (points[:, 0], points[:, 1]) if len(points) else ([], [])

    def update(t):
        index = max(0, min(len(data)-1, np.searchsorted(data['sim_time'], t, side='right')-1))
        row = data[index]
        for trail, global_plan, local_plan, robot, obstacles, window in artists:
            trail.set_data(data['robot_x'][:index+1], data['robot_y'][:index+1])
            global_plan.set_data(*sampled_path('global', t))
            local_plan.set_data(*sampled_path('local', t))
            robot.set_xy(footprint(row))
            window.set_xy((row['robot_x']-2.5, row['robot_y']-2.5))
            for j, circle in enumerate(obstacles):
                circle.center = obstacle_xy(row, j)
        center = np.clip(row['robot_x'], -15.7, 15.7)
        detail.set(xlim=(center-5, center+5), ylim=(-2.7, 2.7))
        detail.set_title('Robot-following view / synthetic public robot', loc='left', fontsize=10, color='#51627a')
        footer.set_text(f"Simulation t = {t-data['sim_time'][0]:5.1f} s     |     "
                        f"Measured v = {row['measured_forward_speed']:.2f} m/s     |     "
                        f"Full-run contacts = {report['physical_contact_count']}\n"
                        'Recorded planner messages + measured Gazebo poses; local window outline is schematic.')
        return []

    subtitle.set_text('40 m corridor  /  0.84 m doors  /  2 triggered moving obstacles  /  5x replay')
    frames = np.append(np.arange(data['sim_time'][0], data['sim_time'][-1], .5), data['sim_time'][-1])
    animation = FuncAnimation(fig, update, frames=frames, interval=100)
    animation.save(output/'corridor_planning.gif', writer=PillowWriter(fps=10), dpi=90)
    update(data['sim_time'][-1])
    fig.savefig(output/'corridor_overview.png', dpi=130)
    selection = data[(data['robot_x']>=-15.) & (data['robot_x']<=-8.)]
    if len(selection):
        title.set_text('DOORWAY + DYNAMIC AVOIDANCE')
        subtitle.set_text('First 0.84 m doorway, then a moving cylinder that stops at the center  /  2.5x replay')
        frames = np.arange(selection['sim_time'][0], selection['sim_time'][-1], .25)
        animation = FuncAnimation(fig, update, frames=frames, interval=100)
        animation.save(output/'door_and_obstacle.gif', writer=PillowWriter(fps=10), dpi=90)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('docs/media'))
    args = parser.parse_args()
    render(args.trial, args.output)
