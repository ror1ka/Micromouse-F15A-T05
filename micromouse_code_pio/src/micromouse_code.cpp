// MTRN3100 Micromouse Task 4.3 firmware
//
// Generative-AI assistance notice: the bounded hardware setup and retrying
// Task-4.3 mission supervisor marked "AI-assisted" were written with OpenAI
// Codex and reviewed by the team.

#include <Arduino.h>
#include <Wire.h>

#include "Micromouse.hpp"
#include "task4_code/task4AutonomousMapping.hpp"

constexpr uint32_t I2C_TIMEOUT_US = 5000;

Micromouse mouse(PIDController(2.0, 1.0, 2.0));
MazeMap maze;
MazeAutonomousPlanner planner;

// Task 4.3 permits only these assessment inputs to be hard-coded. Confirm them
// immediately before marking if the demonstrator changes the start or goal.
const Pose startPose = {4, 7, EAST};
constexpr int TARGET_ROW = 2;
constexpr int TARGET_COL = 5;

Pose pose = startPose;
bool missionComplete = false;
bool persistentFaultStop = false;
uint8_t lastVisitedCount = 0;
uint8_t lastMissionPhase = 0;
uint16_t lastEvidenceProgress = 0;
uint8_t bestPhaseDistance = INFINITE;
uint8_t missionRetriesWithoutProgress = 0;

void setup() {
    Wire.begin();

    // AI-assisted boot liveness: install the bus timeout before the first OLED,
    // IMU, or LiDAR transaction, and never leave a device routine with motors on.
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
    mouse.stop();

    while (!mouse.setupIMU()) {
        mouse.stop();
        delay(250);
    }

    while (!mouse.setupLidar()) {
        mouse.stop();
        delay(250);
    }
    // Initialise the display after another responding device has demonstrated
    // that the shared bus is usable; this also retries any transient boot glitch.
    while (!mouse.setupOled()) {
        mouse.stop();
        delay(250);
    }
    while (!mouse.initialiseGlobalHeading()) {
        mouse.stop();
        delay(250);
    }
    delay(1000);
}

void loop() {
    if (missionComplete || persistentFaultStop) {
        mouse.stop();
        delay(250);
        return;
    }

    if (task43LocalisationLost) {
        mouse.stop();
        delay(250);
        return;
    }

    // AI-assisted mission liveness: a transient sensor/controller failure no
    // longer sets a one-shot latch and parks forever. The map and verified pose
    // remain in RAM, both sensor subsystems recover while stopped, and the
    // incomplete phase is attempted again.
    if (runTask4_3(mouse, maze, planner, pose, startPose,
                   TARGET_ROW, TARGET_COL)) {
        missionComplete = true;
        mouse.stop();
        return;
    }

    mouse.stop();
    if (task43LocalisationLost) {
        delay(250);
        return;
    }
    uint8_t phaseDistance = INFINITE;
    if (task43MissionPhase == 2 &&
        planner.floodFill(maze, startPose.row, startPose.col, KNOWN_ONLY)) {
        phaseDistance = planner.getDistance(pose.row, pose.col);
    } else if (task43MissionPhase >= 4 &&
               planner.floodFill(maze, TARGET_ROW, TARGET_COL, KNOWN_ONLY)) {
        phaseDistance = planner.getDistance(pose.row, pose.col);
    }
    const bool phaseAdvanced = task43MissionPhase > lastMissionPhase;
    if (maze.getNumVisited() > lastVisitedCount || phaseAdvanced ||
        task43EvidenceProgress != lastEvidenceProgress ||
        phaseDistance < bestPhaseDistance) {
        lastVisitedCount = maze.getNumVisited();
        lastMissionPhase = task43MissionPhase;
        lastEvidenceProgress = task43EvidenceProgress;
        bestPhaseDistance = phaseDistance;
        missionRetriesWithoutProgress = 0;
    } else if (missionRetriesWithoutProgress < 10) {
        missionRetriesWithoutProgress++;
    }

    // Count only monotonic mission progress: new cells, a completed phase, or a
    // smaller known-path distance during return/scored navigation. Arbitrary
    // revisit shuttles cannot keep the retry budget alive forever.
    if (missionRetriesWithoutProgress >= 10) {
        persistentFaultStop = true;
        delay(250);
        return;
    }

    mouse.recoverIMU();
    mouse.recoverLidar();
    if (!mouse.oledHealthy()) mouse.setupOled();
    delay(250);
}
