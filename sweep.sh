#!/usr/bin/env bash
# Build and run every arithmetic configuration of the trajectory planner and
# collect the results this thesis reports.
#
#   ./sweep.sh              run the sweep
#   ./sweep.sh --dry-run    list the configurations, build nothing
#
# The sweep varies three factors: the arithmetic format, nondimensionalization
# off then on, and the trigonometric path. Every run is named after all three,
# so no run overwrites another.
#
# Everything a new user needs to change is in the SETTINGS block below.
set -u
cd "$(dirname "$0")"
REPO=$PWD

# ===========================================================================
# SETTINGS
# ===========================================================================

# --- 1. SCENARIO -----------------------------------------------------------
# One of the scenarios defined in include/scenarios.h:
#   intersection   eight vehicles meeting at a crossroads (the reference)
#   crossing       four vehicles meeting at an unsignalled junction
#   merge          one vehicle joining a through lane from a ramp
#
# The name goes into every output filename, so two scenarios do not collide.
SCENARIO="intersection"

# --- 2. TRIGONOMETRIC PATH -------------------------------------------------
# How sin, cos and tan are evaluated (see include/trig.h):
#   poly      evaluated natively in the format under study, from the fdlibm
#             minimax polynomial. This is the path the reported results use.
#   upscale   the argument is converted to fp64, the C library is called, and
#             the result is rounded back to the format.
#
# List one or both. Listing both runs the whole sweep once per path, which
# doubles the number of runs and lets the two paths be compared directly.
#   TRIG_PATHS=(poly upscale)
TRIG_PATHS=(poly)

# --- 3. OUTPUT LOCATION ----------------------------------------------------
# Where this sweep's trajectory and lane CSVs are collected. The directory is
# created if it does not exist. A relative path is taken relative to this
# script, so the default works from any checkout without editing.
#
# Plots go to media/<basename of this directory>/, keeping each scenario's
# figures together. Set OUTDIR="" to leave the CSVs in the build directory and
# the plots in media/, which is the upstream behaviour.
OUTDIR="saved/Intersection"

# ===========================================================================
# END OF SETTINGS - nothing below needs changing for a normal run
# ===========================================================================

BUILD=sweep_build
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

if [ -n "$OUTDIR" ]; then
    case $OUTDIR in
        /*) ;;                      # absolute, use as given
        *)  OUTDIR=$REPO/$OUTDIR ;; # relative to the repo, not to $BUILD
    esac
    # A sweep writes into an empty directory. Appending to one that already
    # holds results mixes runs from different code states in a single report,
    # so an existing directory is moved aside rather than added to.
    if [ "$DRY_RUN" -eq 0 ]; then
        if [ -d "$OUTDIR" ] && [ -n "$(ls -A "$OUTDIR" 2>/dev/null)" ]; then
            ARCHIVE="$OUTDIR.superseded-$(date +%Y%m%d-%H%M%S)"
            mv "$OUTDIR" "$ARCHIVE"
            echo "existing results moved to: $ARCHIVE"
        fi
        if ! mkdir -p "$OUTDIR"; then
            echo "cannot create output directory: $OUTDIR" >&2
            exit 1
        fi
    fi
    echo "saving CSVs to: $OUTDIR"

fi

# label            posit  bits  es      ("-" = not a posit run)
CONFIGS=(
"half16         OFF    16    -"
"posit16es0     ON     16    0"
"posit16es1     ON     16    1"
"posit16es2     ON     16    2"
"posit16es3     ON     16    3"
"posit16es4     ON     16    4"
"ieee32         OFF    32    -"
"posit32es0     ON     32    0"
"posit32es1     ON     32    1"
"posit32es2     ON     32    2"
"posit32es3     ON     32    3"
"fp64           OFF    64    -"
)

total=$(( ${#CONFIGS[@]} * 2 * ${#TRIG_PATHS[@]} ))
n=0

for TRIG in "${TRIG_PATHS[@]}"; do
for QUANT in OFF ON; do
    echo ""
    echo "================ $SCENARIO | trig: $TRIG | nondimensionalization: $QUANT ================"

    for cfg in "${CONFIGS[@]}"; do
        set -- $cfg
        LABEL=$1; POSIT=$2; BITS=$3; ES=$4
        n=$((n + 1))

        ES_ARG=""
        [ "$ES" != "-" ] && ES_ARG="-DPOSIT_ES=$ES"

        printf "[%2d/%2d] %-12s bits=%-2s es=%-2s quant=%-3s trig=%-7s " \
               "$n" "$total" "$LABEL" "$BITS" "$ES" "$QUANT" "$TRIG"

        if [ "$DRY_RUN" -eq 1 ]; then
            echo "(dry run)"
            continue
        fi

        if ! cmake -S . -B "$BUILD" \
                -DENABLE_POSIT="$POSIT" -DREAL_BITS="$BITS" \
                -DENABLE_QUANTIZATION="$QUANT" -DTRIG_PATH="$TRIG" \
                -DSCENARIO="$SCENARIO" \
                $ES_ARG > /dev/null 2>&1; then
            echo "CONFIGURE FAILED"
            continue
        fi

        if ! cmake --build "$BUILD" -j"$(nproc)" > /dev/null 2>&1; then
            echo "BUILD FAILED"
            continue
        fi

        # The binary writes its CSVs relative to its working directory and
        # invokes ../plot_run.py, so it is always run from $BUILD. Anything it
        # produced is moved afterwards, which keeps that contract intact.
        # stdout is kept: it carries the CV line, which is the constraint
        # violation split by family and cannot be recovered from a trajectory.
        RUNLOG="$BUILD/run_stdout.txt"
        ( cd "$BUILD" && ./dynamic_game_trajectory_planner > run_stdout.txt 2> run_stderr.txt )
        rc=$?

        CVLINE=$(grep -m1 '^CV overall' "$RUNLOG" 2>/dev/null)
        CONVLINE=$(grep -m1 '^CONV ' "$RUNLOG" 2>/dev/null)
        if [ -n "$OUTDIR" ] && [ -n "$CVLINE" ]; then
            QTAG=$([ "$QUANT" = "ON" ] && echo withQuanti || echo withoutQuanti)
            # One row per run: the constraint violation split by family, and the
            # convergence state. A run that reaches the iteration limit is the
            # planner's output after a fixed budget, not a solved equilibrium,
            # and that has to be visible in the record rather than inferred.
            printf '%s\t%s\t%s\t%s\t%s\t%s\trc=%s\n' \
                   "$SCENARIO" "$LABEL" "$QTAG" "$TRIG" "$CVLINE" "$CONVLINE" "$rc" \
                   >> "$OUTDIR/cv_report.tsv"
            # The raw streams are kept per run, so a reported number can be
            # traced back to the output it came from.
            cp "$RUNLOG" "$OUTDIR/log_${SCENARIO}_${LABEL}_${QTAG}_${TRIG}.stdout" 2>/dev/null
            cp "$BUILD/run_stderr.txt" "$OUTDIR/log_${SCENARIO}_${LABEL}_${QTAG}_${TRIG}.stderr" 2>/dev/null
        fi

        # Collect this run's CSVs into OUTDIR, and its plot into the media
        # folder named after the scenario. Only the file this run produced is
        # moved, so plots left by earlier runs are untouched.
        if [ -n "$OUTDIR" ]; then
            PLOTDIR="$REPO/media/$(basename "$OUTDIR")"
            mkdir -p "$PLOTDIR"
            for csv in "$BUILD"/trajectories_*.csv "$BUILD"/lanes_*.csv; do
                [ -e "$csv" ] || continue
                stem=$(basename "$csv" .csv)
                mv -f "$csv" "$OUTDIR/"
                [ -e "$REPO/media/$stem.png" ] && mv -f "$REPO/media/$stem.png" "$PLOTDIR/"
            done
        fi

        if [ $rc -eq 0 ]; then
            echo "done"
        else
            echo "RUN FAILED"
        fi
    done
done
done

echo ""
if [ -n "$OUTDIR" ]; then
    echo "sweep finished - CSVs in $OUTDIR/, plots in media/"
else
    echo "sweep finished - CSVs in $BUILD/, plots in media/"
fi
