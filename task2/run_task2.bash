# !/bin/bash

# Define the working directory and rosbag file path
WORKDIR=$(pwd)
BAGFILE="$WORKDIR/task2/task2.bag"

# Get the timestamp for saving results
TIMESTAMP=$(date +'%Y-%m-%d-%H-%M-%S');

# Get the duration of rosbag
DURATION=$(rosbag info $BAGFILE | grep 'duration:' | awk '{print $3}' | sed 's/[^0-9]*//g')

# Set the default recording time and map saving time
if [[ -z "$DURATION" || ! "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  DURATION=290
else
  SAVE_MAP_TIME=$(echo "$DURATION + 5" | bc)
  RECORD_TIME=$(echo "$DURATION + 10" | bc)
  END_TIME=$(echo "$DURATION + 15" | bc)
fi

# Start roscore and keep it open in a tab
gnome-terminal --tab --title="Start roscore" -- bash -c "roscore; exec bash"
# Wait for roscore to initialize
sleep 3
# Set simulated time
rosparam set use_sim_time true

# Start Cartographer
tmux new-session -d -s Cartographer_session -c "$WORKDIR"
tmux new-window -t Cartographer_session -n "Cartographer" -c "$WORKDIR" bash -c "
  cd catkin_ws/;
  catkin_make_isolated --install --use-ninja;
  source install_isolated/setup.bash;
  sleep 5;
  roslaunch cartographer_ros demo_jackal_2d.launch bag_filename:=${BAGFILE};
  exec bash
"

# Record rosbag
tmux new-session -d -s record_session -c "$WORKDIR"
tmux new-window -t record_session -n "Record" -c "$WORKDIR" bash -c "
  mkdir -p result;
  cd result;
  rosbag record -a -O ${TIMESTAMP}_task2_cartographer_result.bag;
  exec bash
"

# Wait for code compile and start Cartographer
sleep 6

# Echo progress
for ((i=0; i<=DURATION; i++)); do
    # Calculate remaining time
    REMAINING=$((DURATION - i))
    echo -ne "Rosbag playback progress: $i / $DURATION seconds.\r"
    sleep 1
done
echo "Rosbag finished playing. Start to save the map ..."

# Save map
gnome-terminal --tab --title="Save Map" --working-directory="$WORKDIR" -- bash -c "
  cd result;
  rosrun map_server map_saver --occ 70 --free 30 -f ${TIMESTAMP}_task2 map:=/map;
  sleep 3;
"
# Wait for map saving
sleep 5
echo "Done. Shut down Cartographer ..."

# Gracefully terminate Cartographer in tmux session
echo "Stop Cartographer ..."
tmux send-keys -t Cartographer_session C-c
sleep 5
tmux kill-session -t Cartographer_session
echo "Done."

# Gracefully terminate rosbag recording in tmux session
echo "Stop rosbag recording ..."
tmux send-keys -t record_session C-c
sleep 5
tmux kill-session -t record_session

# Kill roscore
echo "Done. Kill roscore ..."
pkill -f roscore
echo "Start evo to evaluate ..."

# Start evo_ape
evo_ape bag result/${TIMESTAMP}_task2_cartographer_result.bag /ground_truth /tf:map.base_link --plot --align --plot_mode xy --save_results result/${TIMESTAMP}_task2_cartographer_result.zip