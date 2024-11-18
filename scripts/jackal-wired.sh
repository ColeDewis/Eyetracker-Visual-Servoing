declare -r IP="$1"
if [[ -z "$IP" ]]; then
  echo Put the last entry of your static IP as an argument. Example: source jackal-wired.sh 102
  exit 1
fi

export ROS_MASTER_URI=http://192.168.131.1:11311
export ROS_IP=192.168.131.$IP