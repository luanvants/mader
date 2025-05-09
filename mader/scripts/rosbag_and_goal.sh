#!/bin/bash
cd /home/nvhung/Desktop/bags

# Record for mader
rosbag record -o mader_obs_80_sim_ /SQ01s/mader/actual_traj __name:=name_node_record &
sleep 5
rostopic pub /SQ01s/term_goal geometry_msgs/PoseStamped \
"{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: 75, y: -10, z: 1}, orientation: {w: 1.0}}}"

# Record for semantic mader
# rosbag record -o semantic_mader_obs_80_sim_ /SQ02s/semantic_mader/sem_actual_traj __name:=name_node_record &
# sleep 5
# rostopic pub /SQ02s/term_goal geometry_msgs/PoseStamped \
# "{header: {stamp: now, frame_id: 'world'}, pose: {position: {x: 75, y: -10, z: 1}, orientation: {w: 1.0}}}"