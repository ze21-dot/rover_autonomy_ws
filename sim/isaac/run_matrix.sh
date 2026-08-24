#!/bin/bash
# SuRover KARASIMSEK - Isaac 6.0 test matrix: 8 runs, identical scene/seed/camera, rover mode changes only.
# Usage: nohup bash /workspace/run_matrix.sh > /workspace/matrix_log.txt 2>&1 &
set -u
source /workspace/isaac_env/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES
cd /workspace

run () {   # name model mode speed lane rocker_kd kp
  local name=$1 model=$2 mode=$3 speed=$4 lane=$5 rkd=${6:-300} kp=${7:-300}
  local d=/workspace/runs/$name
  rm -rf "$d"; mkdir -p "$d"
  echo "=== $name  start $(date +%H:%M:%S)  model=$model mode=$mode speed=$speed lane=$lane"
  RUN=$name MODEL=$model MODE=$mode SPEED=$speed LANE=$lane ROCKER_KD=$rkd KP=$kp KD=70 TAU=60 \
    timeout 1200 python /workspace/mars_traverse.py > "$d/log.txt" 2>&1
  echo "  (exit $?)"
  ffmpeg -y -loglevel error -framerate 12 -pattern_type glob -i "$d/frames/rgb_*.png" \
    -c:v libx264 -pix_fmt yuv420p -crf 20 "$d/video.mp4"
  echo "=== $name  done  $(date +%H:%M:%S)  $(grep SESSION-ENDED "$d/log.txt" || tail -1 "$d/log.txt")"
}

#   name               model     mode     speed  lane  rocker_kd
run A_rigid_7          rigid     rigid    7      1
run B_passive_7        revolute  passive  7      1     300
run C_hybrid_7         revolute  hybrid   7      1     300   300
run speed_2            rigid     rigid    2      1
run speed_13           rigid     rigid    13     1
run speed_28           rigid     rigid    28     1
run A_rigid_7_nolane   rigid     rigid    7      0
run C_hybrid_13        revolute  hybrid   13     1     300   300

echo "MATRIX-DONE $(date)"
for d in /workspace/runs/*/; do echo "$(basename $d): $(grep SESSION-ENDED $d/log.txt)"; done
