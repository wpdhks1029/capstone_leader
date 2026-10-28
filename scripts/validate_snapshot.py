"""Offline snapshot verification. No ROS nodes, serial writes or commands."""
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from render_nav2 import render

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'provenance/SOURCE_MANIFEST.json').read_text())
for item in manifest:
    assert hashlib.sha256((root/item['path']).read_bytes()).hexdigest() == item['sha256'], item['path']
# Vendor Python targets may include Python 2 scripts. Validate our Python nodes only.
for base in ('diagnostics', 'scripts', 'src/leader_robot_control', 'src/iahrs_ros2', 'src/convoy_robot'):
    for path in (root/base).rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
for path in (root/'diagnostics').glob('*.yaml'):
    yaml.safe_load(path.read_text())
for path in (root/'src').glob('*/package.xml'):
    ET.parse(path)
ET.parse(root/'diagnostics/navigate_015_recovery.xml')
with tempfile.TemporaryDirectory() as directory:
    out = Path(directory)/'nav2.yaml'
    render(out)
    data = yaml.safe_load(out.read_text())
    follow = data['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert follow['max_vel_x'] == 0.15 and follow['max_vel_theta'] == 0.1
    assert Path(data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']).is_file()
print(f'PASS: {len(manifest)} original file hashes, Python syntax, YAML/XML and portable BT rendering')
