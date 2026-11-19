"""Writes Nav2's default nav2_params.yaml with a stop zone added to the collision
monitor, as a robot builder would configure it: a polygon just ahead of the
robot where any scan point makes the collision monitor STOP.

Nav2 Jazzy's default collision monitor only has a FootprintApproach polygon
(slow down before contact), which never reports STOP.

    python3 nav2_params_with_stop_zone.py <out.yaml>
"""

import sys

import yaml
from ament_index_python.packages import get_package_share_directory

src = f"{get_package_share_directory('nav2_bringup')}/params/nav2_params.yaml"
params = yaml.safe_load(open(src))
cm = params["collision_monitor"]["ros__parameters"]
cm["polygons"] = list(cm.get("polygons", [])) + ["PolygonStop"]
cm["PolygonStop"] = {
    "type": "polygon",
    # In base_footprint, starting outside the 0.22 m robot radius: a strip just
    # ahead of the robot that static obstacles (kept away by inflation) don't reach.
    "points": "[[0.40, 0.18], [0.40, -0.18], [0.25, -0.18], [0.25, 0.18]]",
    "action_type": "stop",
    "min_points": 3,
    "visualize": True,
    "polygon_pub_topic": "polygon_stop",
    "enabled": True,
}
yaml.safe_dump(params, open(sys.argv[1], "w"), sort_keys=False)
print(f"collision_monitor polygons: {cm['polygons']}")
