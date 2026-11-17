#!/usr/bin/env python3
"""Audit the Git index before publication; never inspect or copy private sources."""
from pathlib import PurePosixPath
import hashlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


def git(*args):
    return subprocess.check_output(['git', *args])


def main():
    files = git('diff', '--cached', '--name-only', '--diff-filter=ACMR', '-z').decode().split('\0')
    errors = []
    checked = 0
    forbidden_dirs = {'robot_original', 'private', 'results', 'reference', 'build', 'install', 'log', '__pycache__'}
    forbidden_names = {'sensor_mounts.yaml', 'sensor_manifest.json', 'model_manifest.json', 'SOURCE_MANIFEST.json'}
    model_extensions = {'.stl', '.dae', '.step', '.stp', '.sldasm', '.sldprt', '.urdf', '.xacro', '.obj'}
    generic_robot = 'src/dynamic_nav_sim/urdf/generic_robot.urdf'
    for name in filter(None, files):
        path = PurePosixPath(name)
        checked += 1
        if forbidden_dirs.intersection(path.parts) or path.name in forbidden_names:
            errors.append(f'{name}: private/generated output path')
        if path.suffix.lower() in model_extensions and name != generic_robot:
            errors.append(f'{name}: model assets require a separate public provenance review')
        content = git('show', ':'+name)
        if name == generic_robot:
            root = ET.fromstring(content)
            if root.get('name') != 'generic_demo_robot' or root.findall('.//mesh'):
                errors.append(f'{name}: only the primitive-only generic demo is allowed')
        if path.suffix.lower() in {'.png', '.gif', '.jpg', '.jpeg'}:
            upstream_fixture = (name == 'src/nav2_costmap_2d/test/unit/keepout_filter_test.jpg'
                and hashlib.sha256(content).hexdigest() ==
                '6d415e5da0a952a594f7cb1874c21579f7a7d2a642b323db10adce044225fcf0')
            if not name.startswith('docs/media/') and not upstream_fixture:
                errors.append(f'{name}: new visuals must be reviewed in docs/media')
            continue
        try:
            text = content.decode('utf-8')
        except UnicodeDecodeError:
            continue
        for pattern in [r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----',
                        r'gh[pousr]_[A-Za-z0-9]{30,}', r'/home/[a-zA-Z0-9_-]+/',
                        r'package://(?:ftuwb_nav_sim)/']:
            if re.search(pattern, text):
                errors.append(f'{name}: private path or credential pattern')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'Public asset audit passed: {checked} staged files; no private model assets or credential patterns.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
