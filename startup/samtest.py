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
        "simulator": "roslaunch simulator simulator.launch start_rqt:=true",
        "sam2": "rosrun sam2_ros sam2_node.py",
        "visual_servoing": "rosrun visual_servoing eih_eyetracker_uncal_vs.py",
        "target_picker": "rosrun sam2_ros target_select.py",
    }

    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)
