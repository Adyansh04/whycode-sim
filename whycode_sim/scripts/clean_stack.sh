#!/bin/bash
#
# Stops everything this package starts, then proves the ROS graph is empty.
#
# Leftovers are not cosmetic: two bag players, or a bag player alongside a live Isaac,
# both publish /clock. Time then jumps backwards, RViz resets on every jump, and the image
# and TF flicker before RViz dies -- which reads as a corrupt bag.
#
# Run from the host or inside the container:
#   clean_stack.sh            stop everything, then verify
#   clean_stack.sh --verify   report only, kill nothing
#
# Exits 0 only when the graph is actually empty, so it can gate a replay.

set -uo pipefail

CONTAINER="${ROS_CONTAINER:-whycode_dev_jazzy}"
VERIFY_ONLY=0
[[ "${1:-}" == "--verify" ]] && VERIFY_ONLY=1

# Re-enter the container when run from the host. Written to a file and then run, NOT
# piped to `bash -s`: with -s the script arrives on stdin, and the first command that
# reads stdin swallows the rest, truncating the script part-way.
if [ ! -d /root/workspace ]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "not in the container and docker is not on PATH" >&2
        exit 1
    fi
    exec docker exec -i "$CONTAINER" bash -c \
        'cat > /tmp/whycode-clean-stack.sh && bash /tmp/whycode-clean-stack.sh' < "$0"
fi

# Two rounds, because a name list alone is not enough. NAMED is this stack's own
# binaries; ANY_ROS is the safety net for anything running out of a ROS install or
# carrying --ros-args. The second catches nodes that never appear under their own name: a
# composable container runs several inside one process, so a name sweep reports "killed 0"
# while `ros2 node list` still lists them.
NAMED='rviz2|ground_truth_node|ground_truth_viz_node|waypoint_follower_node|run_sim\.py|rosbag2|ros2 bag play|whycode_sim'
ANY_ROS='component_container|rclcpp_components|--ros-args|/opt/ros/[a-z]+/lib/|ros2 run |ros2 launch |ros2 daemon'

# Never kill this script or its ancestors: their command lines contain the patterns being
# searched for.
protected=" $$ $PPID "
walk=$PPID
while [ -n "$walk" ] && [ "$walk" -gt 1 ] 2>/dev/null; do
    walk=$(awk '{print $4}' "/proc/$walk/stat" 2>/dev/null)
    [ -n "$walk" ] && protected="$protected$walk "
done

# ros-dev runs with `pid: host`, so /proc lists the HOST's processes -- including the
# containerd shim that owns this container, and any ROS running natively. Killing the shim
# takes the container down. Sharing our mount namespace is an exact test for "inside this
# container", where a name or executable allowlist is only a guess.
OUR_NS=$(readlink /proc/self/ns/mnt 2>/dev/null)

in_our_container() {
    [ -z "$OUR_NS" ] && return 0   # no namespace info: do not filter
    [ "$(readlink "/proc/$1/ns/mnt" 2>/dev/null)" = "$OUR_NS" ]
}

sweep() {
    local pattern=$1 killed=0 pid cmd
    for proc in /proc/[0-9]*; do
        pid=${proc#/proc/}
        case "$protected" in *" $pid "*) continue ;; esac
        in_our_container "$pid" || continue
        cmd=$(tr '\0' ' ' < "$proc/cmdline" 2>/dev/null) || continue
        [ -z "$cmd" ] && continue
        if printf '%s' "$cmd" | grep -qE "$pattern"; then
            kill -9 "$pid" 2>/dev/null && killed=$((killed + 1))
        fi
    done
    echo "$killed"
}

graph_nodes() { timeout 25 ros2 node list 2>/dev/null | grep -v '^$'; }

total=0
if [ "$VERIFY_ONLY" -eq 0 ]; then
    # Named sweep first so a normal run reports something recognisable, then the broad
    # one. Repeated, because killing a launch file's parent leaves children mid-spawn.
    for pass in 1 2 3 4; do
        n=$(sweep "$NAMED")
        m=$(sweep "$ANY_ROS")
        total=$((total + n + m))
        echo "[clean] pass $pass: killed $n by name, $m by ROS signature"
        [ "$((n + m))" -eq 0 ] && [ "$pass" -gt 1 ] && break
        sleep 3
    done
fi

# The ROS setup scripts reference unbound variables, which under `set -u` aborts here
# silently -- indistinguishable from the cleanup working and skipping its verification.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash" 2>/dev/null || true
[ -f /root/workspace/install/setup.bash ] && source /root/workspace/install/setup.bash
set -u

# The daemon caches the graph and keeps reporting nodes that are already gone, so without
# this the check below shows dead nodes indefinitely and a working cleanup looks broken.
ros2 daemon stop >/dev/null 2>&1
sleep 3

if [ "$VERIFY_ONLY" -eq 0 ] && [ -n "$(graph_nodes)" ]; then
    echo "[clean] graph not empty after the sweeps; trying once more"
    total=$((total + $(sweep "$ANY_ROS")))
    sleep 3
    ros2 daemon stop >/dev/null 2>&1
    sleep 3
fi

if [ "$VERIFY_ONLY" -eq 0 ]; then
    echo "[clean] killed $total process(es) in total, inside the container"

    # Killing a process does not reclaim its shared memory, and enough leftover segments
    # break discovery: topics appear but subscribers never match, which reads as "the bag
    # is not playing".
    shm=$(ls /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    echo "[clean] cleared $shm stale DDS shared-memory segment(s)"
fi

nodes=$(graph_nodes)
topics=$(timeout 25 ros2 topic list 2>/dev/null | grep -vE '^/parameter_events$|^/rosout$')

echo "--- ros2 node list (want: empty) ---"
if [ -z "$nodes" ]; then echo "  (empty)"; else echo "$nodes" | sed 's/^/  LEFTOVER: /'; fi
echo "--- ros2 topic list (want: only /parameter_events and /rosout) ---"
if [ -z "$topics" ]; then echo "  (clean)"; else echo "$topics" | sed 's/^/  LEFTOVER: /'; fi

[ -z "$nodes" ] && [ -z "$topics" ]
