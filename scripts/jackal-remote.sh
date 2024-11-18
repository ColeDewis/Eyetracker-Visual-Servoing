declare -r IP1="$1"
declare -r IP2="$2"
if [[ -z "$IP" ]]; then
  echo Put the last two entries of your wireless IP as arguments. Example: source jackal-remote.sh 100 22
  exit 1
fi

export ROS_MASTER_URI=http://192.168.$IP1.171:11311
export ROS_IP=192.168.$IP1.$IP2