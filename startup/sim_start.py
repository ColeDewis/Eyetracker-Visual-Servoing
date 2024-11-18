import libtmux
import time
from os import path

if __name__ == "__main__":
    server = libtmux.Server(
        config_file=path.expandvars("/home/user/landmine/startup/.tmux.conf")
    )
    if server.has_session("sim"):
        exit()
    else:
        session = server.new_session(
            "sim", start_directory="/home/user/landmine", attach=False
        )

    # terminals for the simulation to start
    terminals = {
        "simulator": "roslaunch simulator simulator.launch",
    }

    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)
