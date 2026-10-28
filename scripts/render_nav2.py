"""Resolve only the copied BT file location; preserve all validated parameters."""
import argparse
from pathlib import Path
import yaml


def render(output):
    root = Path(__file__).resolve().parents[1]
    data = yaml.safe_load((root/'diagnostics/nav2_015_rotation_shim_check.yaml').read_text())
    bt = root/'diagnostics/navigate_015_recovery.xml'
    assert bt.is_file()
    data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(bt)
    Path(output).write_text(yaml.safe_dump(data, sort_keys=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    render(parser.parse_args().output)
