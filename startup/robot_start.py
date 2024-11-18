import libtmux
import time
from os import path

if __name__ == "__main__":
    server = libtmux.Server(
        config_file=path.expandvars("/home/user/landmine/scripts/.tmux.conf")
    )
    if server.has_session("sim"):
        exit()
    else:
        session = server.new_session(
            "sim", start_directory="/home/user/landmine", attach=False
        )

    # terminals for the simulation to start
    terminals = {
        "rqt": "rqt",
        "kortex_bringup": "roslaunch kortex_bringup kortex_bringup.launch",  # launch kortex - note that this starts a roscore
        "cameras": f"roslaunch --wait camera_node cameras.launch idxone:={0} idxtwo:={1} two_is_rs:=false",
        "tracking": 'rosrun tracking tracking_node.py _idxs:="[0,1]"',
        "visual_servoing": "rosrun visual_servoing visual_servo_node.py",
    }

    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)
