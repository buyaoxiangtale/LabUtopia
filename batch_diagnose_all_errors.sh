#!/bin/bash
set -e

OUTPUT_DIR="/home/pjlab/fbh/LabUtopia/outputs/path_planning_batch_results_gemini_flash/run_2026-01-13_18-50-01"
SCENE_DIR="/home/pjlab/fbh/LabUtopia/roomlayout/layout_new"
ASSETS_JSON="/home/pjlab/fbh/LabUtopia/roomlayout/layout_new/assets_annotated.json"
OFFSET_RADIUS=0.3

DIAGNOSTIC_DIR="${OUTPUT_DIR}/collision_analysis"
mkdir -p "${DIAGNOSTIC_DIR}"

SUMMARY_REPORT="${DIAGNOSTIC_DIR}/diagnostic_summary.txt"

echo "========================================" | tee -a "${SUMMARY_REPORT}"
echo "Batch Path Planning Error Diagnosis" | tee -a "${SUMMARY_REPORT}"
echo "Start time: $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "${SUMMARY_REPORT}"
echo "========================================" | tee -a "${SUMMARY_REPORT}"
echo "" | tee -a "${SUMMARY_REPORT}"

TOTAL_SCENES=0
FAILED_SCENES=0
TOTAL_FAILED_TASKS=0
SUCCESS_DIAGNOSED=0

echo "Scanning for failed tasks..."
echo ""

FAILED_TASKS_FILES=$(find "${OUTPUT_DIR}" -maxdepth 2 -name "*_failed_tasks.json" | sort)

if [ -z "$FAILED_TASKS_FILES" ]; then
    echo "ERROR: No failed task files found!"
    exit 1
fi

FAILED_SCENES=$(echo "$FAILED_TASKS_FILES" | wc -l)
echo "Found ${FAILED_SCENES} scenes with failed tasks"
echo ""

for FAILED_FILE in $FAILED_TASKS_FILES; do
    DIR_NAME=$(basename "$(dirname "$FAILED_FILE")")
    SCENE_BASE_NAME=$(echo "$DIR_NAME" | sed 's/_20[0-9][0-9][0-9][0-9][0-9][0-9]_[0-9][0-9][0-9][0-9][0-9][0-9]$//')

    echo "========================================" | tee -a "${SUMMARY_REPORT}"
    echo "Scene: ${SCENE_BASE_NAME}" | tee -a "${SUMMARY_REPORT}"
    echo "========================================" | tee -a "${SUMMARY_REPORT}"

    # Use ls instead of find for better symlink handling
    SCENE_SUBDIR=$(ls -d "${SCENE_DIR}"/*"${SCENE_BASE_NAME}"* 2>/dev/null | head -1)

    if [ -z "$SCENE_SUBDIR" ] || [ ! -d "$SCENE_SUBDIR" ]; then
        echo "WARNING: Scene directory not found for ${SCENE_BASE_NAME}" | tee -a "${SUMMARY_REPORT}"
        echo "" | tee -a "${SUMMARY_REPORT}"
        continue
    fi

    # Find room file
    SCENE_ROOM_FILE=$(ls "${SCENE_SUBDIR}"/*room_isaacsim.json 2>/dev/null | head -1)

    if [ -z "$SCENE_ROOM_FILE" ] || [ ! -f "$SCENE_ROOM_FILE" ]; then
        echo "WARNING: Room file not found" | tee -a "${SUMMARY_REPORT}"
        echo "" | tee -a "${SUMMARY_REPORT}"
        continue
    fi

    if [ ! -s "$FAILED_FILE" ]; then
        echo "WARNING: Failed tasks file is empty" | tee -a "${SUMMARY_REPORT}"
        echo "" | tee -a "${SUMMARY_REPORT}"
        continue
    fi

    FAILED_COUNT=$(python3 -c "import json; data=json.load(open('$FAILED_FILE')); print(len(data))" 2>/dev/null || echo "0")
    TOTAL_FAILED_TASKS=$((TOTAL_FAILED_TASKS + FAILED_COUNT))

    echo "Scene dir: $(basename "${SCENE_SUBDIR}")" | tee -a "${SUMMARY_REPORT}"
    echo "Room file: $(basename "${SCENE_ROOM_FILE}")" | tee -a "${SUMMARY_REPORT}"
    echo "Failed tasks: ${FAILED_COUNT}" | tee -a "${SUMMARY_REPORT}"

    SCENE_DIAG_DIR="${DIAGNOSTIC_DIR}/${DIR_NAME}"
    mkdir -p "${SCENE_DIAG_DIR}"

    TEXT_OUTPUT="${SCENE_DIAG_DIR}/diagnostic_report.txt"
    VISUALIZE_OUTPUT="${SCENE_DIAG_DIR}/collision_analysis.png"

    echo "Running diagnosis..."
    set +e
    PYTHONPATH=/home/pjlab/fbh/LabUtopia python3 /home/pjlab/fbh/LabUtopia/utils/collision_analyzer.py \
        "${SCENE_ROOM_FILE}" \
        "${ASSETS_JSON}" \
        "${FAILED_FILE}" \
        --offset-radius ${OFFSET_RADIUS} \
        --visualize "${VISUALIZE_OUTPUT}" \
        2>&1 | tee "${TEXT_OUTPUT}"
    EXIT_CODE=${?}
    set -e

    if [ $EXIT_CODE -eq 0 ]; then
        echo "SUCCESS" | tee -a "${SUMMARY_REPORT}"
        SUCCESS_DIAGNOSED=$((SUCCESS_DIAGNOSED + 1))
    else
        echo "ERROR (exit code: ${EXIT_CODE})" | tee -a "${SUMMARY_REPORT}"
    fi

    echo "" | tee -a "${SUMMARY_REPORT}"
    TOTAL_SCENES=$((TOTAL_SCENES + 1))
done

echo "========================================" | tee -a "${SUMMARY_REPORT}"
echo "Diagnosis Complete" | tee -a "${SUMMARY_REPORT}"
echo "Total: ${TOTAL_SCENES} | Successful: ${SUCCESS_DIAGNOSED} | Failed tasks: ${TOTAL_FAILED_TASKS}" | tee -a "${SUMMARY_REPORT}"
echo "Output: ${DIAGNOSTIC_DIR}" | tee -a "${SUMMARY_REPORT}"
echo "End: $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "${SUMMARY_REPORT}"

echo ""
echo "Diagnosis complete!"
echo "Summary: cat ${SUMMARY_REPORT}"
