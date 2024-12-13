import libtmux
import time
from os import path

if __name__ == "__main__":
    server = libtmux.Server(
        config_file=path.expandvars("/home/user/eyetracking_vs/startup/.tmux.conf")
    )
    if server.has_session("eye_vs"):
        exit()
    else:
        session = server.new_session(
            "eye_vs", start_directory="/home/user/eyetracking_vs", attach=False
        )

    # terminals for the simulation to start
    terminals = {
        # "kortex_bringup": "roslaunch kortex_bringup kortex_bringup.launch",  # launch kortex - note that this starts a roscore
        "simulator": "roslaunch simulator simulator.launch start_rqt:=false",
        # "core": "roscore",
        "eyetracker": "rosrun eyetracker tobii_et_node.py",
        "cam_gui": "rosrun eye_ui qt_cam.py",
        "overlay_gui": "rosrun eye_ui overlay.py",
        # "camera": f"roslaunch --wait camera_node d405.launch",
        "visual_servoing": "rosrun visual_servoing eih_eyetracker_vs.py",
    }

    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)
