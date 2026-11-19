#!/bin/bash
# Runs the cbf_benchmark batches of one experiment of README_EXPERIMENTS.md.
#
#   run_experiment.sh h1h2|h3|h4|ablation|psf [extra cbf_benchmark options, e.g. --map maps/robot/map.yaml]
#
# Environment (all optional):
#   BENCH    benchmark command (default: ros2 run cbf_rrt_planner cbf_benchmark)
#   OUT_DIR  results directory (default: src/cbf_rrt_planner/benchmark/experiments)
#   GOALS    goals per batch (default 20)
#   RUNS     seeds per goal and configuration (default 10)
#   TAG      label of the machine, h4 only (default laptop)
#   PSF      PSF source, h4 only (default constant, the one H4 is about)
# The extra options go first, so the settings of the experiment itself always win.
set -e

exp="$1"
shift || true
BENCH="${BENCH:-ros2 run cbf_rrt_planner cbf_benchmark}"
OUT_DIR="${OUT_DIR:-src/cbf_rrt_planner/benchmark/experiments}"
GOALS="${GOALS:-20}"
RUNS="${RUNS:-10}"

# h1h2, h3 and ablation compare batches with each other, so a file must not mix two executions
fresh() {
    if [ -e "$1" ]; then
        echo "error: $1 already exists - move it away or set OUT_DIR to start a new execution" >&2
        exit 1
    fi
}

case "$exp" in
    h1h2)
        # one batch per PSF source: a sweep of c plus plain RRT* on the same goals and seeds
        fresh "$OUT_DIR/h1h2.csv"
        for src in constant centroid; do
            $BENCH "$@" --set psf_source=$src --c 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9 --baseline \
                --random-goals "$GOALS" --runs "$RUNS" --psf-repeats 5 \
                --out "$OUT_DIR/h1h2.csv" --tag h1h2
        done
        ;;
    h3)
        # match: open-space goals, reachable even with the largest radius, so every radius is
        #        compared on the same goals and only the clearance differs
        # near:  goals 0.25-0.40 m from an obstacle, reachable with the minimal radius 0.22 m
        fresh "$OUT_DIR/h3.csv"
        for set in match near; do
            if [ "$set" = match ]; then
                goal_args=(--goal-inflation 0.8)
            else
                goal_args=(--goal-inflation 0.22 --goal-clearance 0.25,0.40)
            fi
            common=("$@" --set psf_source=constant --set inflation=true "${goal_args[@]}"
                    --random-goals "$GOALS" --runs "$RUNS" --out "$OUT_DIR/h3.csv" --tag "$set")
            $BENCH "${common[@]}" --set inflation_robot_size=0.22 --c 0.8 --baseline
            for r in 0.3 0.4 0.5 0.6 0.7 0.8; do
                $BENCH "${common[@]}" --set inflation_robot_size=$r --baseline-only
            done
        done
        ;;
    h4)
        # timing only: few plans, but the PSF is generated 10 times
        $BENCH "$@" --set psf_source="${PSF:-constant}" --c 0.8 --random-goals 10 --runs 3 --psf-repeats 10 \
            --out "$OUT_DIR/h4.csv" --tag "${TAG:-laptop}"
        ;;
    ablation)
        # four variants on the same goals and seeds, on open goals and on goals next to walls:
        #   first batch:  full (c = 0.8), cbf_only (c = 1.0: cost = length) and plain RRT*
        #   second batch: cost_only (c = 0.8, kappa = 1e6: the CBF condition rejects no edge)
        fresh "$OUT_DIR/ablation.csv"
        for set in open near; do
            if [ "$set" = open ]; then
                goal_args=()
            else
                goal_args=(--goal-clearance 0.25,0.40)
            fi
            common=("$@" --set psf_source=constant --set inflation=true --set inflation_robot_size=0.22
                    "${goal_args[@]}" --random-goals "$GOALS" --runs "$RUNS"
                    --out "$OUT_DIR/ablation.csv" --tag "$set")
            $BENCH "${common[@]}" --c 0.8,1.0 --baseline
            $BENCH "${common[@]}" --c 0.8 --set kappa=1e6
        done
        ;;
    psf)
        # PSF fields of both sources on the bundled map and on the map with U-shaped obstacles,
        # nothing is planned
        for map in maps/map.yaml maps/map_u.yaml; do
            stem=$(basename "$map" .yaml)
            for src in centroid constant; do
                $BENCH "$@" --map "$map" --set psf_source=$src --random-goals 0 \
                    --export-psf "$OUT_DIR/psf/${stem}_${src}"
            done
        done
        ;;
    *)
        echo "usage: $0 h1h2|h3|h4|ablation|psf [extra cbf_benchmark options]" >&2
        exit 1
        ;;
esac
echo "experiment $exp done, results in $OUT_DIR"
