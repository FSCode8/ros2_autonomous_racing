import subprocess

def set_pose(name, x, y, z, qx=0, qy=0, qz=0, qw=1):
    pose_str = (
        f"name: '{name}' "
        f"position: {{x: {x}, y: {y}, z: {z}}} "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
    )
    cmd = [
        "ign", "service", "-s", "/world/empty/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "2000",
        "--req", pose_str
    ]
    subprocess.run(cmd)

