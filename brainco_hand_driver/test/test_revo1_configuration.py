# Copyright (c) 2026 BrainCo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"

EXPECTED_JOINTS = [
    "right_thumb_flex_joint",
    "right_thumb_abduction_joint",
    "right_index_flex_joint",
    "right_middle_flex_joint",
    "right_ring_flex_joint",
    "right_pinky_flex_joint",
]


def test_revo1_modbus_defaults():
    config_path = CONFIG_DIR / "protocol_modbus_revo1_right.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))["hardware"]

    assert config["slave_id"] == 1
    assert config["baudrate"] == 115200
    assert config["auto_detect"] is True
    assert config["auto_detect_port"] == "/dev/ttyUSB0"

    limits = [float(value) for value in config["joint_max_positions_rad"].split(",")]
    assert len(limits) == 6
    assert all(value > 0.0 for value in limits)

    controllers = yaml.safe_load(
        (CONFIG_DIR / "revo1_right_controllers.yaml").read_text(encoding="utf-8")
    )
    assert controllers["controller_manager"]["ros__parameters"]["update_rate"] == 20


def test_revo1_xacro_exports_six_position_joints():
    config_path = CONFIG_DIR / "protocol_modbus_revo1_right.yaml"
    model = xacro.process_file(
        str(CONFIG_DIR / "revo1_right.urdf.xacro"),
        mappings={"protocol_config_file": str(config_path)},
    )
    root = ET.fromstring(model.toxml())
    ros2_control = root.find("ros2_control")

    assert ros2_control is not None
    joints = ros2_control.findall("joint")
    assert [joint.attrib["name"] for joint in joints] == EXPECTED_JOINTS
    for joint in joints:
        assert [item.attrib["name"] for item in joint.findall("command_interface")] == [
            "position"
        ]
        assert [item.attrib["name"] for item in joint.findall("state_interface")] == [
            "position"
        ]

    parameters = {
        item.attrib["name"] for item in ros2_control.find("hardware").findall("param")
    }
    assert "device_model" not in parameters
    assert "protocol" not in parameters
