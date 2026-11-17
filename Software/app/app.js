"use strict";

const OPERATING_MODE_VALUES = new Set(["offline", "online"]);
const PLANNING_POLICY_VERSION = "continuous-cartesian-approach-v4";

const translations = {
  en: {
    productName: "UR Print Workstation",
    localControl: "Local robot control",
    currentRunbook: "Current runbook:",
    operatingMode: "MODE",
    offlineMode: "Offline",
    onlineMode: "Online",
    offlineModeHint: "Work with local files, the installed print setup, and cached trajectories. New live hardware actions are locked.",
    onlineModeHint: "Allow live UR, Klipper, ROS, and MoveIt operations. Selecting Online does not start or move anything.",
    offlineModeEnabled: "Offline mode enabled. Existing ROS services were left running.",
    onlineModeEnabled: "Online mode enabled. No service was started and no hardware moved.",
    modeChangeBusy: "Finish or cancel the active operation before changing mode.",
    onlineOnly: "ONLINE ONLY",
    onlineModeRequired: "Online mode required",
    onlineModeRequiredDesc: "This runbook reads or controls live UR, Klipper, ROS, or MoveIt state. Switching Online only unlocks the controls; it does not start services or move hardware.",
    offlineModeGateKicker: "OFFLINE PREPARATION",
    offlineModeSafeWork: "You can continue importing G-code, preparing control channels, and reviewing the automatically centered Cartesian path offline.",
    backgroundServicesRunning: "ROS services are still running in the background. Offline mode blocks new live operations but does not stop them automatically.",
    noBackgroundOperation: "No live operation will be started from Offline mode.",
    stopRunningServices: "Stop ROS services",
    switchOnline: "Switch to Online",
    openOfflinePreparation: "Open G-code Preparation",
    offlineModeStatus: "Offline · local preparation",
    onlineReadyStatus: "Online · system ready",
    onlineNeedsConnectionStatus: "Online · connection required",
    offlineTool0Title: "Online mode is required for live tool0 capture",
    offlineTool0Desc: "Enter measured values manually, or switch Online to read the live base_link → tool0 transform. MoveIt is not required for capture.",
    offlineMoveitTitle: "Offline 3D review",
    offlineMoveitDesc: "You can inspect and play the planned toolpath offline. The arm moves in the preview only when a six-axis trajectory is available; MoveIt collision and joint-limit validation requires Online.",
    offlineSimulationReady: "Offline robot simulation ready",
    offlineSimulationReadyDesc: "A read-only UR3e IK estimate was generated locally from this G-code and the installed print setup. ROS, hardware, and collision checking were not contacted.",
    simulationOnly: "SIMULATION ONLY",
    simulationPathReviewNext: "Open simulation-only 3D review",
    simulationCalibrationTitle: "Offline preview can continue",
    simulationCalibrationDesc: "The current draft can be used for a read-only robot + toolpath estimate. Incomplete physical calibration remains visibly failed and Print Execution stays locked.",
    simulationMoveitBlockedTitle: "Physical calibration is not complete",
    simulationMoveitBlockedDesc: "Physical calibration confirmations are incomplete. Saved parameter values alone do not complete calibration. In Tabletop, Print Z & nozzle setup, verify tool orientation, measure the nozzle TCP and extrusion ratio, then save the installed calibration and regenerate this review before MoveIt validation or Print Execution.",
    simulationDraftUnsavedTitle: "Calibration draft is not saved",
    simulationDraftUnsavedDesc: "This simulation uses a complete but unsaved draft. Return to Calibration and save this exact profile before using MoveIt or Print Execution.",
    cachedMoveitPassed: "Cached online MoveIt trajectory",
    cachedMoveitPassedDesc: "This exact G-code and installed print setup were validated online earlier. The cached six-axis trajectory is read-only and can be replayed offline.",
    settings: "Settings",
    switchLanguage: "Switch language",
    runbookAria: "Setup runbook",
    checkingStatus: "Checking status",
    runbook: "RUNBOOK",
    connection: "Connection",
    gcodeRunbook: "G-code Preparation",
    connectionRunbookDescription: "Connect and verify the UR3e, Klipper, ROS, and MoveIt stack.",
    workcellRunbook: "Tabletop, Print Z & nozzle setup",
    workcellRunbookDescription: "Set the physical tabletop collision plane first, then place G-code Z=0 and review the installed nozzle.",
    workcellStep: "Tabletop and Print Z",
    workcellStepShort: "Save the actual tabletop Z, then choose the print path's Z origin before planning.",
    printZeroTitle: "Print Z placement",
    printZeroCandidateTitle: "Print Z placement (preview candidate)",
    printZeroLead: "Choose the absolute base_link Z where G-code Z=0 will run. The route shifts only in Z; X/Y and the physical tabletop stay fixed. Saving this setting does not move UR or Klipper.",
    printZeroLabel: "Print G-code Z=0 (mm, base_link)",
    printZeroDelta: "Path offset above physical tabletop: {offset} mm",
    printZeroApply: "Save selected print Z",
    printZeroDiscard: "Reset print Z to tabletop",
    printZeroCandidate: "The saved print Z setting is unavailable. Re-enter the desired absolute Z before a new path review.",
    printZReverificationRequired: "The previous Print Z setting is missing or invalid. Re-enter the desired absolute Z, then rebuild the 3D and MoveIt reviews.",
    printZeroVerified: "Selected G-code Z=0: {height} mm in base_link. This is a path placement, not a measurement of the physical tabletop.",
    printZeroMotionReminder: "A saved Z is not a motion approval. Review the full path in 3D, validate it with MoveIt from the current robot posture, and keep the normal collision and start checks.",
    printZTrialTitle: "Verify proposed print Z with the UR",
    printZTrialLead: "For this separate, guarded trial, match the exact nozzle orientation of the reviewed print path, with the tip pointing down, and start 11–60 mm above the proposed print Z=0, well inside the tabletop. Plan first without motion. Only after reviewing the plan and safety checks may you explicitly command a slow UR move to 10 mm above print Z=0. The physical tabletop/collision plane does not move; Klipper does not extrude. If the offset seems large, re-measure the installed nozzle TCP before any motion.",
    printZPlan: "Plan print Z trial — no motion",
    printZExecute: "Move UR to print Z trial height",
    printZConfirm: "Confirm measured print Z",
    printZPlanSucceeded: "Print Z trial plan passed. Check the target and safety conditions before commanding UR.",
    printZExecuteSucceeded: "UR trial finished at the planned 10 mm standoff. Keep it stationary and measure the actual nozzle clearance to the real intended print surface.",
    printZMeasuredClearanceLabel: "Measured nozzle-to-real-print-surface clearance at this trial pose (mm)",
    printZMeasuredClearanceHint: "With the UR stationary, use calibrated gauge blocks or a caliper at the actual intended print surface. Enter the measured gap (9.5–10.5 mm); a visual estimate alone cannot authorize print Z.",
    printZMeasuredClearanceInvalid: "Measure the actual nozzle clearance at the stationary 10 mm trial pose and enter a value from 9.5 to 10.5 mm.",
    printZVisualConfirm: "I checked the stationary nozzle and real intended print surface are clear, and recorded the independent gap measurement",
    printZCommitWarning: "Confirmation compares the measured gap with the planned 10 mm standoff and corrects the candidate by up to 0.5 mm; the saved Print Z appears above. It is bound to the current measured TCP and physical tabletop. Rebuild the 3D and MoveIt reviews before preparing execution.",
    printZOfflineHint: "Offline mode permits path preview, but the guarded print Z trial requires Online mode. No hardware moves automatically.",
    printZOffsetTooLarge: "This print Z is more than 4 mm above the installed tabletop. It is preview-only: the guarded UR trial and physical confirmation are locked. Re-measure and save the installed nozzle TCP, then choose a print Z within 4 mm of the physical tabletop for a real trial.",
    printZeroFixedTable: "Saved physical tabletop is at {height} mm. Print Z moves the path without moving that collision plane.",
    printZeroReview: "The print route uses a proposed Z position. The physical tabletop is unchanged; this review cannot authorize real motion or printing.",
    printZBackendRestart: "This workstation is still running the previous control service. Once work is stopped and UR and Klipper are stationary, restart the workstation to activate direct Print Z selection. Do not use these controls until then.",
    physicalTableTitle: "1. Physical tabletop Z",
    physicalTableLead: "After adjusting the height-adjustable tabletop, enter its actual top surface in base_link. Saving it sets the collision plane used by path validation and execution checks; it sends no motion command.",
    physicalTablePhysicalWarning: "Use the tabletop's actual height after each adjustment. Collision checks use the height you enter, so verify the physical surface before commanding motion.",
    heightCurrent: "Saved physical tabletop Z",
    heightCandidate: "Tabletop Z (mm, base_link)",
    heightCandidatePlanning: "After changing the tabletop, validate the full path again. Next, set Print Z to the desired G-code Z=0 height above this surface.",
    heightOldProposalHint: "An old tabletop proposal of {height} mm is still pending. Save tabletop Z above to make it the active collision plane before choosing Print Z.",
    heightCandidateReview: "A tabletop height change requires a new 3D and MoveIt path review before execution.",
    heightSaveDraft: "Save tabletop Z",
    heightDiscard: "Discard old tabletop proposal",
    heightPlan: "Plan height trial — no motion",
    heightExecute: "Move UR to trial height",
    heightConfirm: "Confirm measured tabletop height",
    heightTrialLead: "For this optional check, position the nozzle in the exact reviewed print orientation, 11–60 mm above the saved surface and well inside the tabletop. The guarded trial moves only UR, at low speed, to 10 mm above the surface. Klipper does not move; cancellation may also send a stop command to it.",
    heightTrialNozzle: "CAD nozzle offset from tool0",
    heightTrialUnverified: "CAD estimate only. Verify actual mounting and nozzle tip before production printing.",
    heightMeasuredTcp: "Installed nozzle TCP was explicitly measured and confirmed. Recheck it after any mount change.",
    heightTrialTarget: "Trial nozzle tip Z",
    heightTrialDistance: "Planned vertical move",
    heightTrialPlanSucceeded: "Plan-only check passed. Review the path and safety conditions before commanding UR.",
    heightTrialExecuteSucceeded: "UR trial finished at the planned 10 mm standoff. Keep it stationary and measure the actual nozzle clearance to the real tabletop.",
    heightMeasuredClearanceLabel: "Measured nozzle-to-real-tabletop clearance at this trial pose (mm)",
    heightMeasuredClearanceHint: "With the UR stationary, use calibrated gauge blocks or a caliper at the actual tabletop. Enter the measured gap (9.5–10.5 mm); a visual estimate alone cannot establish the physical collision plane.",
    heightMeasuredClearanceInvalid: "Measure the actual nozzle clearance at the stationary 10 mm trial pose and enter a value from 9.5 to 10.5 mm.",
    heightSafetyEstop: "Emergency stop is available",
    heightSafetyClear: "Robot and tabletop workspace are clear",
    heightSafetySpeed: "Low-speed motion has been reviewed",
    heightSafetyWired: "Wired network is stable",
    heightSafetyExternal: "UR External Control is playing",
    heightSafetyNozzle: "I measured the installed nozzle tip and verified its mount matches the CAD preview",
    heightVisualConfirm: "I checked that the stationary nozzle and real tabletop are clear, and recorded the independent gap measurement",
    heightOfflineHint: "Switch Online to use the optional UR trial. Saving tabletop Z does not command hardware.",
    heightCommitWarning: "Optional trial confirmation compares the measured gap with the planned 10 mm standoff and may correct the saved tabletop Z by up to 0.5 mm. Rebuild path reviews and execution packages if it changes.",
    heightLegacyProfile: "The saved setup still targets bare tool0. With the new printhead installed, live Tool0-on-table motion is locked. The CAD nozzle position is a starting estimate, not an approved production TCP.",
    nozzleTcpTitle: "Installed nozzle and extrusion calibration",
    nozzleTcpLead: "Enter the measured tool0-to-nozzle offsets for the installed head before authorizing a physical trial or print path.",
    nozzleTcpCadLead: "The inputs start from the revised head's CAD candidate. Replace them with measured tool0-to-nozzle offsets before confirming the installed TCP.",
    previousToolTcp: "Previous tool's saved TCP (comparison only)",
    nozzleMeasured: "I physically measured the actual nozzle tip offsets",
    nozzleOrientation: "I verified the mounted nozzle points vertically down in the reviewed print pose",
    nozzleSave: "Save installed calibration",
    nozzleSaved: "Installed calibration saved. Rebuild the robot path review before any motion.",
    extrusionSetupLead: "Native Klipper E uses filament millimetres. Use a ratio of 1 after calibrating rotation_distance in printer.cfg; verify the measured feed direction. Saving these values sends no motion command.",
    extrusionUnitsLabel: "Filament mm per 1 mm of G-code E",
    extrusionMeasuredInstalled: "I measured and verified the extrusion ratio and feed direction on this installed head",
    extrusionRatioInvalid: "Enter a measured extrusion ratio from 0.0001 to 100000 and select the feed direction.",
    installedCalibrationStatus: "Saved confirmations: TCP {tcp}, orientation {orientation}, extrusion {extrusion}.",
    calibrationConfirmed: "confirmed",
    calibrationRequired: "required",
    geometryRecalibrationRequired: "The printhead geometry changed. Measure and explicitly confirm the installed TCP, nozzle orientation and extrusion ratio for this head before preparing a print.",
    gcodeRunbookDescription: "Import, validate, and review the source print file before robot planning.",
    motionRunbook: "Motion Planning & Simulation",
    motionRunbookDescription: "Build the UR and Klipper channels, generate a six-axis plan, and review it in 3D.",
    executionRunbook: "Motion Test / Print Execution",
    executionRunbookDescription: "Run the reviewed Tool0 motion test or production print and monitor it in one place.",
    moduleCurrent: "Current section",
    moduleReady: "Ready to review",
    chooseRunbook: "Choose a runbook",
    runbookBusy: "Finish or cancel the active hardware operation before changing runbooks.",
    localOnly: "Localhost only",
    estopWorkspace: "Emergency stop ready · workspace clear",
    motionLocked: "Real motion locked",
    motionUnlocked: "One synchronized test authorized",
    continue: "Continue",
    configuration: "CONFIGURATION",
    workstationSettings: "Workstation settings",
    robotIp: "Robot IP",
    rosPcIp: "ROS PC IP (optional)",
    moonrakerUrl: "Moonraker URL",
    manualStepper: "Extruder name",
    updateRate: "UR update rate (Hz)",
    axes: "Cartesian axes",
    distance: "Distance per axis (m)",
    jointSpeed: "Max joint speed (rad/s)",
    stepperMove: "Extrusion distance (filament mm)",
    stepperSpeed: "Extrusion speed (mm/s)",
    stepperAccel: "Extrusion acceleration (mm/s²)",
    stopRos: "Stop ROS system",
    cancel: "Cancel",
    saveSettings: "Save settings",
    confirmation: "CONFIRMATION",
    confirm: "Confirm",
    demoMode: "DEMO MODE",
    systemReady: "System ready",
    systemNotReady: "Not ready",
    systemAttention: "Connection warning",
    settingsSaved: "Settings saved.",
    operationSucceeded: "Operation completed successfully.",
    operationFailed: "Operation failed. Review the command output.",
    operationCanceled: "Operation canceled.",
    operationRecordLost: "The previous service operation is no longer tracked. Live ROS status has been refreshed.",
    operationMotionRecordLost: "The local service lost this motion operation's status. Its stop state is unconfirmed; use Verify stop and recover.",
    requestFailed: "Request failed",
    requestTimedOut: "The request timed out. Please retry.",
    close: "Close",
    pass: "Passed",
    fail: "Failed",
    warn: "Warning",
    pending: "Pending",
    ready: "Ready",
    waiting: "Waiting",
    running: "Running",
    succeeded: "Succeeded",
    failed: "Failed",
    canceled: "Canceled",
    commandOutput: "Command output",
    cancelOperation: "Cancel operation",
    noOutput: "Waiting for output…",
    latestRviz: "Last RViz snapshot",
    referenceOnly: "Reference image · live state is shown in the checks",
    connectionProfile: "Connection profile",
    stepOf: "STEP {current} OF {total}",
    currentStep: "Current step",
    complete: "Complete",
    locked: "Locked",
    clickToReview: "Click to review",
    systemOverview: "System overview",
    urPreview: "UR3e connection preview",
    illustrativeModel: "Illustrative model · status comes from live checks",
    online: "Online",
    awaitingCheck: "Awaiting check",
    offline: "Offline",
    visualConnection: "Connection path",
    macWorkstation: "Workstation computer",
    urController: "UR controller",
    moonrakerService: "Moonraker",
    klipperController: "Klipper controller",
    workstationApp: "Workstation app",
    rosCoordinator: "ROS coordinator",
    klipperBridge: "ROS Klipper bridge",
    moveitUrDriver: "MoveIt / UR driver",
    rosPipeline: "ROS control pipeline",
    planOnly: "PLAN ONLY",
    guardedMotion: "GUARDED MOTION",
    readOnly: "READ ONLY",
    controlPorts: "Control ports",
    offlineAnalysis: "Offline analysis",
    fileReady: "File parsed",
    fileAttention: "File needs attention",
    noHardwareCommands: "Offline file analysis · no hardware commands",
    readOnlyWorkspace: "Read-only workspace",
    nextModule: "Open Path Planning",
    calibrationNext: "Open Calibration",
    pathReviewNext: "Open 3D Robot Path Review",
    collisionNext: "Collision checking · next module",
    draftPlanReady: "Draft plan ready",
    calibrationReady: "Calibration validated",
    calibrationSavedStatus: "Calibration saved",
    pathReviewReadyStatus: "3D path ready",
    moveitValidatedStatus: "MoveIt path validated",

    gcodeStepImport: "Import G-code",
    gcodeStepImportShort: "Choose a local .gcode, .gc, .ngc, or .nc file.",
    gcodeStepParse: "Parse & validate",
    gcodeStepParseShort: "Inspect commands, motion modes, units, and safety-sensitive instructions.",
    gcodeStepReview: "Review toolpath",
    gcodeStepReviewShort: "Review the XY preview, bounds, timing estimate, and issues.",
    gcodeStepPlan: "Path planning",
    gcodeStepPlanShort: "Split the parsed file into UR Robot and Klipper control channels.",
    gcodeStepCalibration: "Calibration",
    gcodeStepCalibrationShort: "Map G-code XYZ to the UR base frame and E to native extruder filament millimetres.",
    gcodeStepPathReview: "3D robot path review",
    gcodeStepPathReviewShort: "Play the calibrated path and inspect MoveIt plan-only validation.",
    gcodeStageWaiting: "Waiting for file",
    gcodeStageParsing: "Parsing",
    gcodeStageComplete: "Complete",
    gcodeStageReview: "Current step",
    gcodeStagePlanning: "Generating plan",
    gcodeStagePlanReady: "Draft ready",
    gcodeStageCalibrationLoading: "Loading profile",
    gcodeStageCalibrationDraft: "Needs validation",
    gcodeStageCalibrationReady: "Ready to save",
    gcodeStageCalibrationSaved: "Profile saved",
    gcodeStageReview3dLoading: "Building 3D path",
    gcodeStageReview3dOffline: "Offline preview ready",
    gcodeStageReview3dValidating: "MoveIt plan-only",
    gcodeStageReview3dPassed: "MoveIt validated",
    gcodeStageReview3dFailed: "MoveIt needs attention",

    motionStepChannels: "Channel planning",
    motionStepChannelsShort: "Separate robot motion from Klipper process commands and inspect the shared timeline.",
    motionStepRobot: "Robot trajectory & 3D review",
    motionStepRobotShort: "Generate the MoveIt six-axis trajectory and play the robot together with the TCP path.",
    executionStepReadiness: "Prepare reviewed job",
    executionStepReadinessShort: "Checksum-lock the G-code, installed print setup, MoveIt trajectory, and Klipper mapping.",
    executionStepRun: "Move to start & run",
    executionStepRunShort: "Authorize the reviewed approach, then start the synchronized Tool0 test or production print.",
    executionStepControl: "Live control & result",
    executionStepControlShort: "Monitor continuous synchronized progress or stop both control channels.",

    moduleMapKicker: "WORKSTATION MODULE",
    dependencyTitle: "Required before this module",
    generateChannelPlan: "Generate channel plan",
    buildRobotTrajectory: "Build 3D robot trajectory",
    moduleDependencyLead: "This section stays locked until its source data is available. No hardware command will be sent by the preparation action.",
    openRequiredModule: "Open {module}",
    motionNeedsGcode: "Import and validate a G-code file before generating robot motion.",
    executionKicker: "GUARDED UR + KLIPPER EXECUTION",
    executionTitle: "Execute the reviewed synchronized job",
    executionLead: "Lock the exact G-code, fixed table, measured TCP, Klipper mapping, and full MoveIt trajectories before authorizing motion.",
    executionNeedsTrajectory: "This exact G-code and installed print setup must first pass the online MoveIt trajectory and collision review.",
    prepareExecution: "Prepare cold test",
    preparingExecution: "Preparing execution package…",
    executionPackageReady: "Execution package ready",
    executionPackageReadyDesc: "Source, installed print setup, approach, print trajectory, TCP, table, and Klipper mapping are checksum-locked together.",
    executionPackageFailed: "The execution package could not be prepared.",
    executionGateSource: "Exact G-code locked",
    executionGatePath: "Full path geometry",
    executionGateTcp: "Measured nozzle TCP",
    executionGateBed: "Fixed table geometry",
    executionGateTool0: "Tool0 temporary reference",
    executionGateMoveit: "Collision-aware MoveIt plan",
    executionGateTrajectory: "Six-axis trajectory integrity",
    synchronizedBlocks: "Timeline segments",
    extrusionBlocks: "Extrusion segments",
    coldMotorBlocks: "Mapped motor segments",
    plannedTime: "Planned time",
    motionTestRunTime: "This test run",
    motionTestCoverage: "Full-path coverage",
    executionExtent: "Test range",
    executionExtentFull: "Full model",
    executionExtentFullDesc: "All reviewed layers",
    executionExtentSample: "Quick sample",
    executionExtentSampleDesc: "Connection check only",
    fullColdBoundary: "This execution path never sends heater or fan commands. Prepare and verify the hotend separately before any material print.",
    preparingFullExecution: "Preparing the continuous motion timeline…",
    preparingFullHint: "Large files may take a few minutes. No motion is sent.",
    preparationProgress: "Preparation progress",
    preparationEta: "About {time} remaining",
    preparationEtaPending: "Estimating remaining time…",
    preparationPhaseQueued: "Waiting to start",
    preparationPhaseValidatingInputs: "Checking the locked source and review",
    preparationPhaseParsingGcode: "Parsing the complete G-code",
    preparationPhaseRebuildingReview: "Rebuilding the reviewed path",
    preparationPhaseLoadingMoveitReview: "Loading the MoveIt result",
    preparationPhaseValidatingReview: "Verifying review integrity",
    preparationPhaseLoadingTrajectories: "Reading full-resolution trajectories",
    preparationPhaseMappingTimeline: "Mapping UR and Klipper events",
    preparationPhaseCertifyingPrintCurve: "Certifying the continuous print curve",
    preparationPhaseCertifyingApproachCurve: "Certifying the approach curve",
    preparationPhaseWritingPackage: "Locking the execution package",
    preparationPhaseComplete: "Preparation complete",
    preparationPhaseFailed: "Preparation failed",
    fullColdPackageReady: "Full cold-test package ready",
    fullColdPackageDesc: "The complete continuous model timeline is locked. This is a cold test, not authorization for material printing.",
    fullColdPackageShort: "Full cold test",
    fullColdPackageShortDesc: "Complete reviewed model path · heat off",
    startFullColdTest: "Start full model-path cold test",
    minimumWallTime: "Minimum total time",
    motionTime: "Motion time",
    nominalGcodeTime: "Nominal G-code time",
    reviewedMoveitTime: "MoveIt-reviewed UR time",
    processOnlyHoldTime: "Stepper-only holds",
    scheduleFloorExtension: "G-code / stepper schedule extension",
    certifiedRetimeOverhead: "Smooth-limit retiming",
    certifiedApproachTime: "Reviewed approach",
    finalExecutableTime: "Final executable time",
    executionTimingBreakdown: "Time comparison and final composition",
    executionTimingHint: "The imported G-code estimate mainly uses commanded feed rates and explicit dwell commands; it does not predict robot motion. Executable time also includes the reviewed robot path, motor-only holds, certified trajectory retiming and the approach. Speed and collision limits are unchanged.",
    fullColdFinishedTitle: "Full cold test complete",
    fullColdFinishedDesc: "The complete reviewed model timeline finished. No heat or material printing was performed.",
    executionSourceChanged: "The source or review changed during preparation. Prepare this job again.",
    printBedLimits: "Allowed physical print area",
    nozzleTcp: "Nozzle TCP",
    tool0Reference: "Tool0 process point",
    motionTestGuardPlane: "Fixed table plane",
    moveToStart: "Move to reviewed print start",
    moveToStartTitle: "Move the UR to the reviewed print start?",
    moveToStartDesc: "The UR will execute only the separately reviewed collision-free approach. Klipper will not move. Keep the physical emergency stop in hand.",
    approachRevalidationTitle: "Review the approach from the current robot position",
    approachRevalidationDesc: "The move to print start failed or was canceled. Return to the path review, click Validate with MoveIt again from the current position, then Prepare a new execution package. The previous failure record is preserved.",
    reviewApproachAgain: "Return to MoveIt review",
    atPrintStart: "UR is at the reviewed print start",
    atPrintStartDesc: "The executor will still compare the live six joints with the locked first print point immediately before starting.",
    productionPrintNoticeBeforeStart: "Move the UR to the reviewed start first. This job sends UR and extruder motion but no heater or fan commands; prepare the hotend separately before material printing. Physical simultaneous start/stop has not been certified.",
    productionPrintNoticeAtStart: "The move to print start is complete; 100% is not print progress. After live checks, you can explicitly start the reviewed UR/Klipper path. This job does not heat the hotend; confirm its preparation separately. Physical simultaneous start/stop has not been certified.",
    startPrint: "Start synchronized print",
    startPrintTitle: "Start the reviewed UR/Klipper path?",
    startPrintDesc: "ROS will software-coordinate one continuous UR trajectory and one queued Klipper extruder program. It will not heat the hotend or run slicer preparation commands; confirm those separately before material printing. Physical simultaneous start/stop has not been certified. Stay ready to stop both devices.",
    startMotionTest: "Start coordinated Tool0 test",
    startMotionTestTitle: "Start the real cold Tool0 motion test?",
    startMotionTestDesc: "Software-coordinated UR + Klipper test. Native E motion requires a separately prepared hotend at extrusion-ready temperature. Physical simultaneous start/stop is not verified. No heater, fan, homing, or slicer preparation command will be sent. Keep the physical emergency stop accessible.",
    tool0ExecutionPackageReady: "Cold Tool0 motion-test package ready",
    tool0ExecutionPackageReadyDesc: "The full path passed MoveIt. This runs one continuous cold trajectory with one software-coordinated start and verified action completion. Physical synchronization is not certified; production printing requires its own reviewed package.",
    motionTestPackageShort: "Tool0 motion test",
    motionTestPackageShortDesc: "Cold software-coordinated sample",
    productionPackageShort: "Production print",
    productionPackageShortDesc: "Reviewed software-coordinated job",
    heatingOff: "HEAT OFF",
    executionHeatExternal: "Prepare the hotend separately. E motion starts only after live temperature and extrusion readiness checks pass.",
    externalControlPlayingExecution: "External Control is playing on the teach pendant",
    externalControlRequired: "The backend also verifies a fresh joint state and the live External Control connection.",
    executionSafetyTitle: "Live motion authorization",
    executionSafetyDesc: "These confirmations are session-only and are checked again by the backend.",
    executionSpeedTitle: "Reviewed low programmed speed",
    executionSpeedDesc: "Source-timed motion · joint peak ≤ 0.2 rad/s. Pendant and safety settings are unchanged.",
    executionProgress: "Progress",
    executionWaiting: "Waiting for execution",
    executionStateStarting: "Starting",
    executionStatePreflight: "ROS preflight",
    executionStateRunning: "Running test",
    executionMovingToStart: "Moving to print start",
    executionTimeRemaining: "About {seconds} s left",
    executionAwaitingMotion: "Waiting for UR motion",
    executionRunningDesc: "Executing the reviewed test.",
    executionStateAtStart: "At print start",
    executionStateCompleted: "Completed",
    executionStateFailed: "Failed",
    executionStateStopUnconfirmed: "Stop unconfirmed — new motion locked",
    executionStopLockedDesc: "Stop evidence is incomplete. Keep both devices stopped and use Verify stop and recover before creating a fresh plan.",
    recoveryKicker: "SAFE RECOVERY",
    recoveryTitle: "Verify stop and restore the workstation",
    recoveryDesc: "One click stops local ROS and execution processes, confirms the UR program is stopped and its fresh joint feedback is stationary, checks Klipper, then archives the incident and removes the lock only when all checks pass.",
    recoveryPreservesEvidence: "The failed execution is preserved as evidence and will never resume. A fresh MoveIt review and Prepare Test are required afterward.",
    recoverPrintStop: "Verify stop and recover",
    recoveringPrintStop: "Stopping and verifying…",
    recoverySucceeded: "Recovery complete. ROS remains stopped; run a fresh MoveIt review before the next test.",
    recoveryUnavailable: "This incident cannot be recovered automatically. Review the detailed message and keep both devices stopped.",
    executionFailureDesc: "Execution failed. Review the cause and current stop status below before any further action.",
    executionErrorDetails: "Failure and stop details",
    executionRootCause: "Initial failure",
    executionCurrentStop: "Current stop checks",
    executionBlockProgressHint: "Continuous trajectory progress; timeline markers do not stop motion.",
    executionLiveJointProgressHint: "Based on live UR feedback.",
    executionPreflightSnapshot: "Start checks · snapshot only",
    executionStateCanceled: "Stopped",
    executionSignalExternal: "External Control",
    executionSignalJoints: "Live joint state",
    executionSignalKlipper: "Klipper action",
    executionSignalTrajectory: "Trajectory action",
    executionFooterSafety: "Physical emergency stop remains primary",
    executionFooterLocked: "Motion requires live checks and the reviewed trajectory",
    executionFooterActive: "UR and Klipper are under software coordination",
    executionFooterComplete: "Execution trace complete · inspect the robot and table",
    stopPrint: "Stop UR + Klipper",
    stopPrintTitle: "Stop Print Execution now?",
    stopPrintDesc: "ROS will cancel the UR trajectory and issue the Klipper emergency-stop path. Use the physical emergency stop immediately if motion does not stop.",
    stopSemantics: "Stop cancels both control channels; Klipper may require a firmware restart afterward.",
    executionFinishedTitle: "Synchronized execution complete",
    executionFinishedDesc: "The locked UR trajectory and Klipper program both reported successful completion. Inspect the robot, tool, and table before moving again.",
    motionTestFinishedTitle: "Tool0 motion-test sample complete",
    motionTestFinishedDesc: "Cold sample complete. Full G-code was not run.",
    sourceLine: "G-code line",
    currentBlock: "Timeline segment",
    blockKind: "Process",
    systemHealthTitle: "Live control stack",
    healthExternalShort: "External",
    planningOnly: "Planning and simulation only · no trajectory execution",

    gcodeKicker: "OFFLINE JOB PREPARATION",
    gcodeTitle: "Import and parse G-code",
    gcodeLead: "Inspect a local G-code file before it enters ROS planning. The file is parsed in memory, is not uploaded to a printer, and cannot move hardware.",
    importGcode: "Import G-code file",
    dropGcode: "Drop a G-code file here",
    dropGcodeLead: "or choose a file from this computer",
    chooseFile: "Choose G-code file",
    supportedGcode: ".gcode · .gc · .ngc · .nc · maximum 20 MB",
    parserChecks: "What this parser checks",
    parserCheck1Title: "Motion and extrusion",
    parserCheck1Desc: "G0/G1/G2/G3 moves, coordinate modes, feed rates, extrusion, and layer markers.",
    parserCheck2Title: "Geometry summary",
    parserCheck2Desc: "XY preview, XYZ bounds, print/travel distance, estimated duration, and temperatures.",
    parserCheck3Title: "Safety-sensitive commands",
    parserCheck3Desc: "Emergency, shell, firmware restart, configuration-save, and forced-motion commands are flagged.",
    offlineByDesign: "Offline by design",
    offlineByDesignDesc: "Parsing does not contact ROS, Moonraker, Klipper, or the UR robot. The source file is not saved by the workstation.",
    parsingGcode: "Parsing G-code…",
    parsingGcodeDesc: "Reading commands and building a bounded preview. No hardware connection is used.",
    replaceFile: "Import another file",
    parsedFile: "Parsed file",
    parsedReady: "Parsed · ready for review",
    parsedAttention: "Parsed · attention required",
    fileSummary: "File summary",
    toolpathPreview: "XY toolpath preview",
    printMove: "Print move",
    travelMove: "Travel move",
    previewSampled: "Preview optimized · all move statistics are complete",
    previewComplete: "All XY segments shown",
    analysisResults: "Analysis results",
    commands: "Commands",
    moves: "Moves",
    printMoves: "Print moves",
    travelMoves: "Travel moves",
    layers: "Layers",
    estimatedTime: "G-code time estimate",
    pathLength: "Path length",
    extrusion: "Extrusion",
    retractions: "Retractions",
    tools: "Tools",
    xyzBounds: "XYZ bounds",
    min: "Min",
    max: "Max",
    size: "Size",
    topCommands: "Top commands",
    fileDetails: "File details",
    fileName: "File name",
    fileSize: "File size",
    lines: "Lines",
    lineUnit: "lines",
    durationHour: "h",
    durationMinute: "m",
    durationSecond: "s",
    generator: "Generator",
    checksum: "SHA-256",
    preparationCommands: "Preparation commands",
    preparationCommandNote: "Detected for the later preparation step; nothing is executed during import.",
    notReported: "Not reported",
    parserIssues: "Parser findings",
    noParserIssues: "No blocking parser findings. The file is ready for review before path planning.",
    issueError: "BLOCKING",
    issueWarning: "CHECK",
    issueInfo: "INFO",
    lineLabel: "Line {line}",
    occurrences: "{count} occurrences",
    gcodeImportFailed: "The G-code file could not be parsed.",
    importModeTitle: "Choose how to import this job",
    importModeGcode: "Original G-code",
    importModeGcodeDesc: "Use the existing parsing, path planning, MoveIt review, and execution workflow.",
    importModePaired: "UR trajectory + extrusion G-code",
    importModePairedDesc: "Import two files produced for the same timeline. This first version supports offline format review only; real motion stays locked.",
    pairedTitle: "Import a shared-timeline pair",
    pairedLead: "Choose one six-joint UR trajectory CSV and its matching timed extrusion G-code. Nothing is sent to ROS, UR, or Klipper.",
    pairedTrajectory: "UR trajectory (.csv)",
    pairedExtrusion: "Timed extrusion (.gcode)",
    pairedChoose: "Choose file",
    pairedNoFile: "No file selected",
    pairedCheck: "Check paired timeline",
    pairedChecking: "Checking both files…",
    pairedMissing: "Choose both files before checking.",
    pairedTooLarge: "Each file must be at most 20 MB.",
    pairedFileType: "The trajectory must be .csv and the extrusion file .gcode.",
    pairedReady: "Shared timeline parsed for offline review",
    pairedLocked: "Import is not MoveIt validation or permission to print. The existing execution workflow cannot use this pair yet.",
    pairedFormat: "Extrusion v1: G21, M83, then relative G1 E… or G4 P… lines with ;@sync start=<seconds> end=<seconds>. Include zero-extrusion intervals. The final end time must equal the UR CSV end time.",
    pairedPoints: "UR joint points",
    pairedSegments: "Extrusion intervals",
    pairedDuration: "Shared duration",
    pairedRetract: "Retraction",
    pairedOriginalFlow: "Continue with original G-code",
    gcodeClientFileType: "Choose a .gcode, .gc, .ngc, or .nc file.",
    gcodeClientFileSize: "The selected file exceeds the 20 MB limit.",
    issue_dangerous_command: "{command} can immediately stop or force machine state and must be removed before planning.",
    issue_configuration_command: "{command} changes or restarts controller configuration and is blocked from the print workflow.",
    issue_homing_command: "This is not a parse error. G28 asks Klipper to home its configured axes. Nothing runs during import; before execution, decide whether homing belongs in a separate preparation step.",
    issue_inch_units: "G20 switches to inches. Values were converted to millimetres for this analysis.",
    issue_invalid_arc_geometry: "This arc is incomplete or geometrically inconsistent, so no robot path was created from it.",
    issue_unsupported_gcode_command: "{command} can change motion or coordinates but is not safely modelled by this workflow.",
    issue_unmodelled_process_command: "{command} is retained for process review but does not change the converted robot geometry.",
    issue_extended_command_not_modelled: "{command} is a printer-specific command. Its macro body is not expanded during offline path conversion.",
    issue_external_macro_not_expanded: "{command} is treated as a separate preparation or ending macro; any motion inside the macro is not part of this robot path.",
    issue_firmware_retraction_not_modelled: "Firmware retraction has no explicit E distance in this file. Convert it to explicit extrusion moves before synchronized planning.",
    issue_unsupported_extruder_axis: "This file uses an A, B, or C extrusion axis. Convert it to standard E-axis G-code before planning.",
    issue_non_finite_parameter: "A command contains a numeric value that cannot be safely interpreted.",
    issue_no_motion_commands: "No supported motion commands were found in this file.",
    issue_no_extrusion_moves: "Motion was found, but no positive extrusion moves were detected.",
    issue_missing_feed_rate: "Some moves occur before a valid feed rate is defined, so their time cannot be estimated.",
    issue_preview_sampled: "Only the drawing was optimized for responsiveness. Every parsed move is still included in the statistics.",

    planKicker: "DUAL-CHANNEL PATH PLANNING",
    planTitle: "UR Robot and Klipper plan",
    planLead: "Separate G-code XYZ motion from Klipper process commands, then review where both channels must stay synchronized.",
    backToReview: "Back to toolpath review",
    generatingPlan: "Generating the dual-channel plan…",
    generatingPlanDesc: "Classifying every motion and process command. No command is sent to ROS, Klipper, or the robot.",
    planFailed: "The path plan could not be generated.",
    planSourceUnavailable: "Re-import the G-code file before generating a plan.",
    regeneratePlan: "Regenerate plan",
    draftPlan: "Draft plan · execution locked",
    draftPlanDesc: "This is a read-only plan. The installed transform and fixed-table placement are applied automatically in 3D Review; collision checking and operator approval are still required before execution.",
    controlResponsibility: "Control responsibility",
    urRobotChannel: "UR ROBOT",
    urChannelBadge: "XYZ MOTION",
    urChannelRole: "Moves the tool through the G-code X, Y, and Z path.",
    urChannelController: "ROS 2 → MoveIt → UR driver",
    klipperChannel: "KLIPPER",
    klipperChannelBadge: "PROCESS CONTROL",
    klipperChannelRole: "Controls extrusion E, retraction, heaters, fans, and preparation commands.",
    klipperChannelController: "ROS 2 → Klipper Bridge → Moonraker",
    waypoints: "UR waypoints",
    totalPath: "Total path",
    extrusionMoves: "Extrusion moves",
    heaterCommands: "Heater commands",
    fanCommands: "Fan commands",
    machineSettings: "Machine settings",
    synchronizationOverview: "Synchronization overview",
    combinedMoves: "UR + Klipper together",
    urOnlyMoves: "UR Robot only",
    klipperOnlyMoves: "Klipper only",
    channelTimeline: "Two-channel timeline",
    timelineLead: "Blue is UR Robot motion. Orange and purple are Klipper extrusion and retraction.",
    timelineSampled: "Timeline display optimized · all events remain in plan statistics",
    timelineComplete: "All synchronized events shown",
    sourceLine: "G-code line",
    urAction: "UR Robot action",
    klipperAction: "Klipper action",
    eventDuration: "Duration",
    action_print_path: "Follow print path",
    action_travel: "Travel",
    action_hold: "Hold position",
    action_extrude: "Extrude",
    action_retract: "Retract",
    action_idle: "No process action",
    representativeEvents: "Representative synchronized events",
    coordinateFrame: "Coordinate frame",
    sourceFrame: "Source",
    targetFrame: "Target",
    scale: "Scale",
    notCalibrated: "Not calibrated",
    planningGates: "Required before execution",
    gatePassed: "Passed",
    gateRequired: "Required",
    gate_source_parsed_title: "Source file parsed",
    gate_source_parsed_desc: "The G-code passed blocking parser checks.",
    gate_coordinate_transform_title: "Apply the installed G-code transform",
    gate_coordinate_transform_desc: "Use the installed orientation and scale, then center the printed model on the fixed table in base_link.",
    gate_collision_check_title: "Run collision checking",
    gate_collision_check_desc: "Validate the full UR path against the robot, tool, table, and workpiece.",
    gate_extrusion_calibration_title: "Apply the installed Klipper extrusion mapping",
    gate_extrusion_calibration_desc: "Map G-code E millimetres with the installed Klipper extrusion ratio and direction.",
    gate_tcp_geometry_title: "Measure the nozzle TCP and tool envelope",
    gate_tcp_geometry_desc: "A non-placeholder tool0-to-tip offset and collision radius are required for planning and bed checks.",
    gate_tool0_reference_title: "Use tool0 as the temporary process point",
    gate_tool0_reference_desc: "The test path controls tool0 directly; no print-head TCP or added tool collision model is assumed.",
    gate_preparation_review_title: "Review preparation commands",
    gate_preparation_review_desc: "Keep G28 and machine setup outside the synchronized print trajectory.",
    gate_print_bed_title: "Keep the complete path on the fixed table",
    gate_print_bed_desc: "The automatically centered path must fit inside the confirmed table edges and safety margin, and G-code Z=0 must align with the tabletop.",
    gate_full_resolution_geometry_title: "Use the complete unsampled path",
    gate_full_resolution_geometry_desc: "Execution requires every parsed motion and process event; a display-sampled preview can never unlock printing.",
    gate_moveit_plan_title: "Validate with MoveIt",
    gate_moveit_plan_desc: "Check the complete path for IK, joint limits, joint continuity, and collisions in the active planning scene without executing motion.",
    gateFailed: "Failed",

    calibrationKicker: "OFFLINE PRINT SETUP",
    calibrationTitle: "Calibrate UR coordinates and Klipper extrusion",
    calibrationLead: "Rotate and scale this G-code; the software then centers its printed bounds on the confirmed fixed table. Validation and saving do not connect to or move hardware.",
    backToPlanning: "Back to path planning",
    calibrationLoading: "Loading the calibration profile…",
    calibrationLoadingDesc: "Reading the local profile and preparing this file's bounds check. No hardware connection is used.",
    calibrationLoadFailed: "The calibration profile could not be loaded.",
    calibrationDraft: "Calibration draft · execution locked",
    calibrationDraftDesc: "Verify XY direction, tool orientation, TCP, and extrusion, then validate the automatically centered path.",
    calibrationValidated: "Validated · ready to save",
    calibrationValidatedDesc: "The transform and extrusion mapping are complete for this file. Collision checking is still required before execution.",
    tool0CalibrationValidatedDesc: "The Tool0 transform, fixed table plane, and Klipper mapping are ready to save. MoveIt collision validation is still required before motion.",
    calibrationSaved: "Saved calibration · execution remains locked",
    calibrationSavedDesc: "This local profile was saved for the current G-code checksum. It will be revalidated when another file is loaded.",
    tool0CalibrationSavedDesc: "The cold Tool0 motion-test profile is saved for this exact G-code. Heating and production printing remain disabled.",
    calibrationNeedsAttention: "Calibration needs attention",
    calibrationNeedsAttentionDesc: "Complete the tool and process confirmations and keep the centered path inside the fixed table.",
    tool0CalibrationNeedsAttentionDesc: "Confirm the Tool0 reference and orientation, and keep the centered path inside and on or above the fixed table.",
    calibrationSaveLockedTitle: "Save is still locked",
    calibrationSaveLockedDesc: "Resolve every item below except collision checking, then run Validate calibration again.",
    onlineCalibrationSaveFirst: "Online 3D Review requires a saved calibration for this exact G-code. Validate until Save becomes available, then save the profile.",
    calibrationModeKicker: "TEST PURPOSE",
    calibrationModeTitle: "What are you validating?",
    calibrationModeHelp: "Choose the physical assumptions used by calibration, collision review, and execution.",
    productionPrintMode: "Production print",
    productionPrintModeDesc: "Uses the measured print-head TCP, tool collision envelope, and confirmed fixed table boundaries.",
    productionModeBadge: "MEASURED TOOL",
    productionModeRequirement: "Measured TCP · tool envelope · physical bed",
    productionModeSummary: "Requires the final print-head geometry; the confirmed fixed table is applied automatically.",
    tool0MotionTestMode: "Tool0 motion test",
    tool0MotionTestModeDesc: "Uses tool0 itself as the G-code point for a cold UR + Klipper coordination test. No heater or preparation command is sent.",
    tool0ModeBadge: "COLD TEST",
    tool0ModeRequirement: "No heat · TCP = 0 · fixed table plane",
    tool0ModeSummary: "Authorizes only a guarded cold coordination test. Heating and material printing stay locked.",
    tool0MotionTestBanner: "Cold motion-test mode",
    tool0MotionTestBannerDesc: "TCP is treated as exactly zero. The path may touch the fixed table plane but cannot authorize heating or material printing.",
    savedProfileLoaded: "Saved profile loaded",
    savedProfileSameFile: "Last saved for this G-code file.",
    savedProfileDifferentFile: "Saved for another G-code file; this file is being revalidated against the same installed print settings.",
    transformFlow: "Coordinate transform",
    gcodeCoordinates: "G-code XYZ",
    rotationAndScale: "Rotate · scale · offset",
    urBaseCoordinates: "UR base frame",
    urCoordinateCalibrationDesc: "Set the G-code Z-axis rotation and unit scale. The task origin is calculated automatically from the print bounds and fixed table.",
    automaticPlacement: "Automatic print placement",
    automaticPlacementDesc: "After rotation and scaling, the center of the extruded print path is aligned to the fixed tabletop center for every job. Travel-only moves do not shift the print.",
    printCenter: "Fixed print center",
    generatedTaskOrigin: "Generated task origin",
    confirmedGeometry: "CONFIRMED · READ ONLY",
    tableCenter: "Table center",
    tableDimensions: "Tabletop X × Y",
    tableYaw: "Table yaw",
    tableBounds: "Table bounds",
    tableThickness: "Table thickness",
    notMeasured: "Not measured",
    gcodeZDatum: "Vertical datum",
    rotationZ: "Z rotation",
    scaleMPerMm: "Scale",
    metresPerMillimetre: "m per G-code mm",
    orientationVerified: "I verified the G-code X/Y orientation",
    orientationVerifiedDesc: "Confirm the sign and Z rotation using known fixture points.",
    toolOrientation: "Tool orientation",
    toolOrientationDesc: "Set the fixed tool roll, pitch, and yaw that MoveIt should maintain along the print path.",
    toolRoll: "Tool roll",
    toolPitch: "Tool pitch",
    toolYaw: "Tool yaw",
    toolOrientationVerified: "I measured and verified the tool orientation",
    toolOrientationVerifiedDesc: "Confirm the real tool direction at the flange; the 3D review will use this orientation for every waypoint.",
    tcpTitle: "Nozzle TCP & collision envelope",
    tcpDesc: "Measure the vector from tool0 to the material/nozzle tip in tool0 coordinates. This converts the physical deposition path into the tool0 path that MoveIt controls.",
    tcpOffsetX: "TCP offset X",
    tcpOffsetY: "TCP offset Y",
    tcpOffsetZ: "TCP offset Z",
    tcpRadius: "Tool collision radius",
    tcpMeasured: "I measured the tool0-to-nozzle TCP and tool radius",
    tcpMeasuredDesc: "Use the mounted production tool. Zero or nominal placeholder geometry keeps Print Execution locked.",
    tool0ReferenceVerified: "I confirm tool0 is the temporary G-code point",
    tool0ReferenceVerifiedDesc: "Use only for a cold motion test before the final print-head model and TCP are installed.",
    tool0TcpNotRequired: "Print-head TCP is not used in this test",
    tool0TcpNotRequiredDesc: "The effective offset is fixed at X 0, Y 0, Z 0. Stored production TCP values are preserved but ignored.",
    klipperExtrusionCalibration: "Klipper extrusion calibration",
    klipperExtrusionCalibrationDesc: "Map each G-code E millimetre to the configured ROS Klipper bridge output.",
    extrusionTarget: "ROS bridge target",
    unitsPerEMm: "Filament mm per E mm",
    extrusionDirection: "Direction",
    directionPositive: "Positive",
    directionNegative: "Negative",
    extrusionFormula: "Filament mm = E mm × ratio × direction",
    extrusionRatioMeasured: "I measured and verified the extrusion ratio",
    extrusionRatioMeasuredDesc: "Confirm the result at the actual stepper and tool, not from a nominal value alone.",
    printBed: "Fixed tabletop",
    printBedDesc: "Confirmed, read-only tabletop geometry in the UR base frame.",
    printBedPlaneZ: "Bed surface Z",
    printBedMinimumX: "Bed minimum X",
    printBedMaximumX: "Bed maximum X",
    printBedMinimumY: "Bed minimum Y",
    printBedMaximumY: "Bed maximum Y",
    printBedEdgeMargin: "Edge margin",
    printBedMeasured: "I measured all four bed edges and the bed surface height",
    printBedMeasuredDesc: "Measure in base_link coordinates. Do not infer the bed size from the G-code preview or 3D picture.",
    guardPlaneMeasured: "I measured the table surface height",
    guardPlaneMeasuredDesc: "Only the table Z height in base_link is required for this Tool0 test; X/Y bed edges are intentionally ignored.",
    printBedSafeArea: "Usable safe area",
    printBedSurfaceGap: "Lowest path above bed",
    printBedNotMeasured: "The fixed table geometry could not be loaded.",
    printBedSurfaceMismatch: "The nozzle path goes below the physical tabletop. Review Print Z, G-code Z, and the TCP.",
    airRunGuardPlane: "Fixed Tool0 table plane",
    airRunGuardPlaneDesc: "Tool0 may touch the confirmed tabletop, and the same fixed X/Y bounds remain active in Motion Test.",
    minimumAxis: "Minimum {axis}",
    maximumAxis: "Maximum {axis}",
    sourceAndTransformedBounds: "Source and transformed bounds",
    sourceGcodeBounds: "Source G-code bounds",
    transformedUrBounds: "Transformed UR bounds",
    axis: "Axis",
    allowed: "Allowed",
    result: "Result",
    inside: "Inside",
    outside: "Outside",
    awaitingValidation: "Validate the draft to calculate transformed UR bounds.",
    calibrationGates: "Calibration gates",
    validateCalibration: "Validate calibration",
    validatingCalibration: "Validating…",
    saveCalibration: "Save calibration",
    savingCalibration: "Saving…",
    calibrationValidationFailed: "The calibration could not be validated.",
    calibrationSaveFailed: "The calibration could not be saved.",
    calibrationSavedToast: "Calibration saved locally. Hardware remains disconnected.",
    noHardwareCalibration: "Offline calibration · no ROS, Moonraker, or robot commands",
    collisionStillRequired: "Collision checking still required",

    pathReviewKicker: "3D ROBOT PATH REVIEW",
    pathReviewTitle: "Review the centered UR path",
    pathReviewLead: "Review the automatically centered nozzle route offline. The UR3e arm follows only when a six-axis trajectory is available; MoveIt validation remains a separate online gate.",
    backToCalibration: "Back to calibration",
    pathReviewLoading: "Building the 3D path…",
    pathReviewQueued: "3D review queued…",
    pathReviewBuilding: "Calculating the robot path…",
    pathReviewGenerationProgress: "Local review generation: {progress}%",
    retryPathReview: "Retry 3D path review",
    pathReviewLoadingDesc: "Transforming the bounded G-code preview and estimating UR3e joint motion locally. ROS and hardware are not contacted.",
    pathReviewLoadFailed: "The 3D path review could not be created.",
    pathReviewOffline: "Offline toolpath ready",
    pathReviewOfflineDesc: "Inspect and play the planned nozzle route without UR, ROS, or Klipper. The arm stays still until a six-axis trajectory is available. This is not collision validation.",
    pathReviewMoveitPassed: "MoveIt plan-only validation passed",
    pathReviewMoveitPassedDesc: "MoveIt returned a collision-aware six-axis trajectory for the complete parsed path. Execution still requires the immutable package and operator authorization.",
    pathReviewMoveitFailed: "MoveIt could not validate this path",
    pathReviewMoveitFailedDesc: "Inspect the failing fraction and adjust the installed tool setup, planning scene, or path geometry.",
    pathReviewDemoPassed: "Demo MoveIt result · simulated",
    pathReviewDemoPassedDesc: "The interaction and robot animation are simulated. No ROS service, robot, or controller was contacted.",
    pathViewport: "Interactive path viewport",
    pathViewportHelp: "Drag to orbit · scroll to zoom · right-drag to pan · click a segment to inspect its G-code line",
    cameraIso: "Isometric",
    cameraTop: "Top",
    cameraFront: "Front",
    cameraSide: "Side",
    showLayersThrough: "Show layers through",
    allLayers: "All layers",
    pathLegend: "Path legend",
    urRobotModel: "Official UR3e visual model",
    printTableModel: "Print table · top {height} mm",
    urPrintPath: "UR print path",
    urTravelPath: "UR travel path",
    klipperExtrusionEvents: "Klipper extrusion",
    klipperRetractionEvents: "Klipper retraction",
    playPreview: "Play toolpath",
    playRobotMotion: "Play robot + toolpath",
    pausePreview: "Pause",
    robotMotionReady: "Robot motion ready",
    robotMotionReadyDesc: "MoveIt joint trajectory loaded · the arm and toolpath will play together.",
    robotMotionEstimated: "Offline robot simulation ready",
    robotMotionEstimatedDesc: "Estimated six-axis motion loaded · press Play to preview the arm and toolpath together. Simulation only; not collision validated.",
    robotMotionWaiting: "Toolpath-only preview",
    robotMotionWaitingDesc: "Play or scrub the planned nozzle route offline. The arm remains still until a matching six-axis trajectory is available.",
    restartPreview: "Restart",
    playbackSpeed: "Playback speed",
    currentPathPoint: "Current path point",
    segmentNumber: "Segment",
    layerNumber: "Layer",
    pathType: "Path type",
    urPosition: "UR TCP position",
    klipperDelta: "Klipper delta",
    moveitValidation: "MoveIt validation",
    moveitValidationRequired: "Six-axis trajectory required",
    moveitValidationRequiredDesc: "Start ROS and MoveIt so this review can automatically generate the collision-aware UR joint trajectory. External Control does not need to be playing.",
    moveitUnavailableTitle: "ROS and MoveIt are not running",
    moveitUnavailableDesc: "The offline 3D toolpath can play independently. MoveIt collision and joint-limit validation requires Connection → Start ROS / MoveIt.",
    moveitValidationErrorTitle: "MoveIt validation could not start",
    openConnectionModule: "Open Connection",
    tool0UnavailableTitle: "Live tool0 pose is unavailable",
    tool0UnavailableDesc: "Start the UR driver and wait for fresh joint states in Connection before capturing tool0. This button never moves the robot.",
    validateWithMoveit: "Validate with MoveIt",
    validatingWithMoveit: "MoveIt is planning…",
    moveitValidationFailed: "MoveIt validation could not be completed.",
    moveitValidationLog: "Validation progress",
    cartesianFraction: "Cartesian fraction",
    plannedJointPoints: "Joint points",
    maxJointStep: "Maximum joint step",
    approachToPathStart: "Approach to path start",
    approachDistance: "Distance to path start",
    approachPassed: "Planned",
    approachFailed: "No plan",
    reviewCalibration: "Review calibration",
    planningScene: "Planning scene",
    activeMoveitScene: "Active MoveIt scene",
    simulatedScene: "Demo simulation",
    pathReviewSummary: "Review summary",
    displayedSegments: "Displayed segments",
    sampledPreview: "This review is sampled for responsiveness. It is useful for visual inspection, but it cannot unlock execution.",
    completePreview: "Every returned preview segment is shown. Execution remains locked.",
    exactCollisionRequired: "Exact collision checking remains a separate required module",
    collisionValidatedByMoveit: "MoveIt collision checking passed for the complete parsed path",
    pathReviewNoMotion: "Plan-only review · no trajectory execution and no Klipper command",
    robotModelSource: "Official UR3e visual meshes from ur_description · Offline uses a local IK estimate; Online can replace it with a matching MoveIt trajectory.",
    modelLoadWarning: "The UR3e mesh could not be loaded; path review remains available.",

    stepSafety: "Safety preparation",
    stepSafetyShort: "Confirm the work cell before any connection or motion command.",
    stepUr: "Connect UR3e",
    stepUrShort: "Verify the wired network, Dashboard, and RTDE interfaces.",
    stepKlipper: "Connect Klipper",
    stepKlipperShort: "Read Moonraker, Klipper, native extruder configuration, and hotend temperature.",
    stepRos: "Start ROS / MoveIt",
    stepRosShort: "Launch the driver, connect External Control, start planning, and verify the ROS Klipper bridge.",

    safetyKicker: "SAFE START",
    safetyTitle: "Prepare the work cell",
    safetyLead: "These confirmations reset every application session. They are required again before the backend can start ROS or authorize motion.",
    safetyChecks: "Operator confirmations",
    estopTitle: "Emergency stop is within reach",
    estopDesc: "Use the physical emergency stop if motion becomes unsafe. This application is not an emergency stop.",
    clearTitle: "Robot workspace is clear",
    clearDesc: "No people, loose tools, cables, or unsecured objects are inside the motion envelope.",
    speedTitle: "Teach pendant speed is below 20%",
    speedDesc: "Keep the speed slider low for commissioning and the first synchronized test.",
    wiredTitle: "The robot uses wired Ethernet",
    wiredDesc: "Do not use Wi-Fi for the UR real-time control path.",
    confirmSafety: "Confirm safety and continue",
    safetyPrinciples: "Safety boundaries",
    safetyBoundary1: "The physical emergency stop always has priority.",
    safetyBoundary2: "Real motion requires live safety checks and a reviewed trajectory.",
    safetyBoundary3: "The backend repeats readiness checks before motion.",
    safetyIncomplete: "Complete all four safety confirmations.",
    activeControlsTitle: "Already-running controls",
    activeControlsLead: "Use these stop controls if ROS or MoveIt was left running from an earlier session. Confirm that the robot is stationary first.",
    stopMoveitEarly: "Stop MoveIt",
    stopRosDriverEarly: "Stop ROS and UR driver",
    stopMoveitEarlyTitle: "Stop MoveIt now?",
    stopMoveitEarlyDescription: "The robot must be stationary. This stops only the MoveIt planning service and keeps the UR driver, ROS container, Docker runtime, and Klipper available.",
    stopRosDriverEarlyTitle: "Stop ROS and the UR driver now?",
    stopRosDriverEarlyDescription: "The robot must be stationary. This closes External Control and stops MoveIt plus the UR driver, while keeping the ROS container, Docker runtime, and Klipper available.",

    urKicker: "DEVICE CONNECTION",
    urTitle: "Connect the UR3e",
    urLead: "This step checks the workstation-to-robot wired path. It does not move the robot or start ROS.",
    operatorPrep: "Before the check",
    urPrep1: "Connect this workstation and the UR control box with Ethernet.",
    urPrep2: "Power on the robot and release the brakes.",
    urPrep3: "Keep the External Control program stopped until ROS asks for Play.",
    urPrepCombined: "Power on the robot, release the brakes, and keep External Control stopped until ROS asks for Play.",
    runUrCheck: "Run connection check",
    connectionResults: "Connection results",
    notCheckedYet: "No check has run in this session.",
    routeInterface: "Route interface",
    localAddress: "Local address",
    connectionGuide: "Connection guidance",
    urGuide: "The required path is wired Ethernet → Dashboard (29999) → RTDE (30004). Dashboard may be limited in Local Control, but RTDE and Ping must pass.",
    urCheckPassed: "UR3e checks passed. You can continue to Klipper.",
    urCheckFailed: "Fix the failed UR3e checks before continuing.",

    klipperKicker: "PRINTER CONNECTION",
    klipperTitle: "Connect Klipper",
    klipperLead: "The workstation reads the native extruder configuration and live temperature through Moonraker. A cold hotend can connect; E motion requires extrusion-ready temperature. This check sends no G-code.",
    klipperPrep: "Required Klipper configuration",
    klipperPrep1: "Moonraker must be reachable from this workstation for this read-only preflight.",
    klipperPrep2: "Klipper must report the ready state.",
    klipperPrep3: "printer.cfg must contain the selected [extruder] (or [extruder1], etc.).",
    runKlipperCheck: "Run Klipper check",
    klipperCheckPassed: "Klipper checks passed. You can start ROS.",
    klipperCheckFailed: "Fix the failed Klipper checks before continuing.",
    endpoint: "Endpoint",
    stepper: "Extruder",
    noGcode: "Workstation read-only preflight · no motion command or G-code is sent",

    rosKicker: "CONTROL STACK",
    rosTitle: "Start ROS and MoveIt",
    rosLead: "The application starts the tested ROS stack and shows its real output. ROS owns all later Klipper motion through the ROS Klipper bridge.",
    startDriver: "Start ROS and UR driver",
    startDriverDesc: "Start Docker, apply the 125 Hz stability configuration, and launch the detached UR driver.",
    waitExternal: "Connect External Control",
    waitExternalDesc: "Open the saved program on the teach pendant and press Play. The app waits for joint states and the reverse interface.",
    startMoveit: "Start MoveIt",
    startMoveitDesc: "Load the calibrated robot model and bring up the motion planning server.",
    runAction: "Run",
    rosStatus: "ROS readiness",
    docker: "Docker container",
    urDriver: "UR driver",
    externalControl: "External Control",
    moveit: "MoveIt",
    rosReadyCallout: "ROS, External Control, MoveIt, and the ROS Klipper bridge are ready. Continue to the dry run.",
    rtdeConflict: "RTDE is occupied. Disable robot Fieldbus or close the other RTDE client, then start ROS again.",
    hostPortConflict: "UR ports 50001–50004 are not reachable on this workstation. Use Restart ROS to repair the container network before pressing Play.",
    rosNeedsAttention: "Complete the driver, External Control, MoveIt, and ROS Klipper bridge readiness stages before continuing.",
    klipperBridgeNeedsAttention: "The ROS Klipper bridge is not ready. Review the latest ROS status or restart the ROS stack before continuing.",
    pressPlay: "After the driver starts, press Play on the teach pendant program containing External Control.",
    restartRos: "Restart ROS",
    restartRosHelpTitle: "Restart ROS",
    restartRosHelp: "Stops the current driver and MoveIt, clears stale processes, then starts a fresh driver and verifies readiness.",
    restartRosTitle: "Restart ROS now?",
    restartRosDescription: "First stop the External Control program on the teach pendant. This disconnects ROS and MoveIt; it does not replace the physical emergency stop.",
    restartRosAccepted: "ROS restart started. Wait for the readiness check to finish.",


    stopTitle: "Stop the ROS system?",
    stopDescription: "This stops the ROS container. Do not do this while the robot is moving. Use the physical emergency stop for unsafe motion.",
    stopAccepted: "ROS stop requested.",
  },

};

const connectionSteps = [
  { id: "safety", title: "stepSafety", subtitle: "stepSafetyShort" },
  { id: "ur", title: "stepUr", subtitle: "stepUrShort" },
  { id: "klipper", title: "stepKlipper", subtitle: "stepKlipperShort" },
  { id: "ros", title: "stepRos", subtitle: "stepRosShort" },
];

const steps = connectionSteps;

const gcodeSteps = [
  { id: "gcode-import", title: "gcodeStepImport", subtitle: "gcodeStepImportShort" },
  { id: "gcode-parse", title: "gcodeStepParse", subtitle: "gcodeStepParseShort" },
  { id: "gcode-review", title: "gcodeStepReview", subtitle: "gcodeStepReviewShort" },
];

const motionSteps = [
  { id: "motion-channels", title: "motionStepChannels", subtitle: "motionStepChannelsShort" },
  { id: "motion-robot", title: "motionStepRobot", subtitle: "motionStepRobotShort" },
];

const executionSteps = [
  { id: "execution-readiness", title: "executionStepReadiness", subtitle: "executionStepReadinessShort" },
  { id: "execution-run", title: "executionStepRun", subtitle: "executionStepRunShort" },
  { id: "execution-control", title: "executionStepControl", subtitle: "executionStepControlShort" },
];

const runbooks = {
  connection: {
    title: "connection",
    description: "connectionRunbookDescription",
    icon: "cable",
    onlineOnly: true,
  },
  workcell: {
    title: "workcellRunbook",
    description: "workcellRunbookDescription",
    icon: "straighten",
    steps: [{ id: "workcell-height", title: "workcellStep", subtitle: "workcellStepShort" }],
  },
  gcode: {
    title: "gcodeRunbook",
    description: "gcodeRunbookDescription",
    icon: "description",
    steps: gcodeSteps,
  },
  motion: {
    title: "motionRunbook",
    description: "motionRunbookDescription",
    icon: "view_in_ar",
    steps: motionSteps,
  },
  execution: {
    title: "executionRunbook",
    description: "executionRunbookDescription",
    icon: "play_circle",
    onlineOnly: true,
    steps: executionSteps,
  },
};

runbooks.connection.steps = connectionSteps;

const storedRunbook = localStorage.getItem("urStationRunbook");
const migratedRunbook = storedRunbook;

const state = {
  language: "en",
  activeRunbook: Object.prototype.hasOwnProperty.call(runbooks, migratedRunbook)
    ? migratedRunbook
    : "connection",
  config: null,
  workcell: null,
  printZMeasuredClearanceMm: "",
  heightMeasuredClearanceMm: "",
  nozzleVerification: { measured: false, orientation: false, extrusion: false },
  workcellSafety: {
    emergency_stop: false,
    workspace_clear: false,
    reviewed_low_speed: false,
    wired_network: false,
    external_control_playing: false,
    installed_nozzle_verified: false,
    visual_clearance_confirmed: false,
  },
  demo: false,
  operatingMode: "offline",
  modeChanging: false,
  currentStep: 0,
  moduleSteps: {
    motion: 0,
    execution: 0,
  },
  completed: new Set(),
  safety: {
    emergency_stop: false,
    workspace_clear: false,
    speed_under_20: false,
    wired_network: false,
  },
  results: { ur: null, klipper: null },
  rosStages: { driver: false, external: false, moveit: false, bridge: false },
  rosStatus: null,
  printInterlock: null,
  recoveringPrintStop: false,
  recoveryError: "",
  activeJob: null,
  jobSubmitting: false,
  jobCanceling: null,
  jobPollTimer: null,
  jobPollGeneration: 0,
  jobSuccessHandler: null,
  execution: {
    extent: "full",
    preparing: false,
    preparation: null,
    package: null,
    error: "",
    runtime: null,
    requiresRevalidation: false,
    job: null,
    externalControlPlaying: false,
    reviewedLowSpeed: false,
    pollTimer: null,
  },
  gcode: {
    importMode: null,
    importRequestId: 0,
    file: null,
    sourceFile: null,
    parsing: false,
    result: null,
    error: "",
    view: "review",
    planning: false,
    plan: null,
    planError: "",
    pathReviewLoading: false,
    pathReviewRequestId: 0,
    pathReviewJob: null,
    pathReview: null,
    pathReviewError: "",
    pathReviewMoveitError: "",
    pathReviewMoveitRunning: false,
    pathReviewAutoValidationAttempted: false,
    pathReviewRequestedSimulation: false,
    pathReviewMoveitJob: null,
    pathReviewMoveitResult: null,
    pathReviewSimulationResult: null,
    pathReviewProgress: 0,
    pathReviewPlaying: false,
    pathReviewPlaybackSpeed: 1,
    pathReviewLayer: "all",
    pathReviewViewer: null,
    pathReviewAnimationFrame: null,
    pathReviewLastFrame: 0,
    pathReviewModelWarningShown: false,
  },
  pairedImport: {
    trajectoryFile: null,
    extrusionFile: null,
    checking: false,
    result: null,
    error: "",
    requestId: 0,
  },
};

const elements = {};

function resetPrintExecution() {
  window.clearTimeout(state.execution.pollTimer);
  state.execution = {
    extent: "full",
    preparing: false,
    preparation: null,
    package: null,
    error: "",
    runtime: null,
    requiresRevalidation: false,
    job: null,
    externalControlPlaying: false,
    reviewedLowSpeed: false,
    pollTimer: null,
  };
}

function invalidatePlanningForWorkcellChange() {
  stopPathReviewPlayback();
  destroyPathReviewViewer();
  state.gcode.pathReviewRequestId += 1;
  state.gcode.pathReviewJob = null;
  state.gcode.pathReview = null;
  state.gcode.pathReviewError = "";
  state.gcode.pathReviewMoveitJob = null;
  state.gcode.pathReviewMoveitResult = null;
  state.gcode.pathReviewMoveitError = "";
  state.gcode.pathReviewMoveitRunning = false;
  state.gcode.pathReviewSimulationResult = null;
  state.gcode.pathReviewAutoValidationAttempted = false;
  resetPrintExecution();
}

function t(key, values = {}) {
  let text = translations.en[key] ?? key;
  for (const [name, value] of Object.entries(values)) {
    text = text.replaceAll(`{${name}}`, String(value));
  }
  return text;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function materialIcon(name, className = "") {
  return `<span class="material-symbols ${escapeHtml(className)}" aria-hidden="true">${escapeHtml(name)}</span>`;
}

function isOnlineMode() {
  return state.operatingMode === "online";
}

function runbookRequiresOnline(runbookId = state.activeRunbook) {
  return Boolean(runbooks[runbookId]?.onlineOnly);
}

async function fetchWithTimeout(path, options = {}, timeoutMs = 30000) {
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
  try {
    return await fetch(path, { ...options, signal: controller.signal });
  } catch (error) {
    if (error?.name === "AbortError") throw new Error(t("requestTimedOut"));
    throw error;
  } finally {
    window.clearTimeout(timeout);
  }
}

async function api(path, options = {}) {
  const { timeoutMs = 30000, ...fetchOptions } = options;
  const request = { ...fetchOptions, headers: { ...(fetchOptions.headers || {}) } };
  if (request.body && typeof request.body !== "string") {
    request.headers["Content-Type"] = "application/json";
    request.body = JSON.stringify(request.body);
  }
  const response = await fetchWithTimeout(path, request, timeoutMs);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    const error = new Error(payload.error || `${t("requestFailed")} (${response.status})`);
    error.status = response.status;
    error.code = payload.code || "";
    error.details = payload.details || null;
    throw error;
  }
  return payload;
}

function safetyPayload(extra = {}) {
  return { ...state.safety, ...extra };
}

function allSafetyConfirmed() {
  return Object.values(state.safety).every(Boolean);
}

function notify(message, type = "") {
  const duplicate = [...elements.toastRegion.children].find(
    (item) => item.textContent === String(message) && item.className === `toast ${type}`.trim(),
  );
  if (duplicate) return;
  const toast = document.createElement("div");
  toast.className = `toast ${type}`.trim();
  toast.textContent = message;
  elements.toastRegion.append(toast);
  window.setTimeout(() => toast.remove(), 5200);
}

function renderOperatingModeControl() {
  if (!elements.operatingModeSelector) return;
  const online = isOnlineMode();
  elements.operatingModeSelector.dataset.mode = state.operatingMode;
  elements.operatingModeSelector.setAttribute(
    "aria-label",
    `${t("operatingMode")}: ${t(online ? "onlineMode" : "offlineMode")}`,
  );
  [
    [elements.offlineModeButton, "offline", "offlineModeHint"],
    [elements.onlineModeButton, "online", "onlineModeHint"],
  ].forEach(([button, mode, hint]) => {
    if (!button) return;
    const selected = state.operatingMode === mode;
    button.classList.toggle("selected", selected);
    button.setAttribute("aria-pressed", String(selected));
    button.setAttribute("title", t(hint));
    button.disabled = state.modeChanging;
  });
}

async function changeOperatingMode(mode) {
  if (!OPERATING_MODE_VALUES.has(mode) || mode === state.operatingMode) return;
  if (jobIsRunning() || state.gcode.pathReviewMoveitRunning) {
    notify(t("modeChangeBusy"), "error");
    return;
  }
  state.modeChanging = true;
  renderOperatingModeControl();
  try {
    const payload = await api("/api/mode", {
      method: "PUT",
      body: { mode },
    });
    state.operatingMode = payload.mode;
    if (!isOnlineMode()) {
      stopPathReviewPlayback();
      notify(t("offlineModeEnabled"), "success");
    } else {
      notify(t("onlineModeEnabled"), "success");
      await refreshStatus(false);
    }
  } catch (error) {
    notify(error.message || t("requestFailed"), "error");
  } finally {
    state.modeChanging = false;
    render();
  }
}

function applyStaticTranslations() {
  document.documentElement.lang = "en";
  document.title = t("productName");
  document.querySelectorAll("[data-i18n]").forEach((node) => {
    node.textContent = t(node.dataset.i18n);
  });
  elements.demoBadge.textContent = t("demoMode");
  elements.settingsButton.setAttribute("aria-label", t("settings"));
  renderOperatingModeControl();
  document.querySelector(".runbook")?.setAttribute("aria-label", t("runbookAria"));
  const runbook = runbooks[state.activeRunbook];
  elements.currentRunbookName.textContent = t(runbook.title);
  elements.runbookSidebarName.textContent = t(runbook.title);
  elements.runbookButton.setAttribute(
    "aria-label",
    `${t("chooseRunbook")}: ${t(runbook.title)}`,
  );
  renderRunbookMenu();
  updateClock();
}

function renderRunbookMenu() {
  elements.runbookMenu.innerHTML = Object.entries(runbooks)
    .map(([id, runbook]) => {
      const selected = id === state.activeRunbook;
      const modeLocked = Boolean(runbook.onlineOnly && !isOnlineMode());
      return `
        <button class="runbook-option ${selected ? "selected" : ""} ${modeLocked ? "mode-locked" : ""}" type="button" role="menuitemradio" aria-checked="${selected}" data-runbook="${escapeHtml(id)}">
          <span class="runbook-option-icon">${materialIcon(runbook.icon)}</span>
          <span class="runbook-option-copy">
            <span class="runbook-option-title"><strong>${escapeHtml(t(runbook.title))}</strong></span>
            <small>${escapeHtml(t(runbook.description))}</small>
            ${runbook.onlineOnly ? `<span class="runbook-mode-requirement">${materialIcon("lock", "small")} ${escapeHtml(t("onlineOnly"))}</span>` : ""}
          </span>
          ${materialIcon(selected ? "check_circle" : modeLocked ? "lock" : "arrow_forward", selected ? "selected-icon" : "muted-icon")}
        </button>`;
    })
    .join("");
  elements.runbookMenu.querySelectorAll("[data-runbook]").forEach((button) => {
    button.addEventListener("click", () => selectRunbook(button.dataset.runbook));
  });
}

function setRunbookMenuOpen(open) {
  elements.runbookMenu.hidden = !open;
  elements.runbookButton.setAttribute("aria-expanded", String(open));
  elements.runbookButton.classList.toggle("open", open);
}

function selectRunbook(runbookId) {
  if (!runbooks[runbookId]) return;
  if (
    runbookId !== state.activeRunbook
    && jobIsRunning()
  ) {
    notify(t("runbookBusy"), "error");
    return;
  }
  if (runbookId !== "motion") {
    destroyPathReviewViewer();
    stopPathReviewPlayback();
  }
  state.activeRunbook = runbookId;
  if (runbookId === "gcode") state.gcode.view = "review";
  if (runbookId === "motion") {
    state.gcode.view = state.moduleSteps.motion === 1 ? "path-review" : "planning";
  }
  localStorage.setItem("urStationRunbook", runbookId);
  setRunbookMenuOpen(false);
  render();
  if (runbookId === "execution" && state.execution.package) {
    window.setTimeout(() => refreshPrintExecutionStatus(true), 0);
  }
  elements.runbookButton.focus();
}

function updateClock() {
  if (!elements.currentDateTime) return;
  const locale = "en-US";
  elements.currentDateTime.textContent = new Intl.DateTimeFormat(locale, {
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  }).format(new Date());
}

function maxUnlockedStep() {
  let unlocked = 0;
  for (let index = 0; index < steps.length - 1; index += 1) {
    if (state.completed.has(steps[index].id)) unlocked = index + 1;
    else break;
  }
  return unlocked;
}

function renderNavigation() {
  if (state.activeRunbook === "gcode") {
    renderGcodeNavigation();
    return;
  }
  if (state.activeRunbook !== "connection") {
    renderModuleNavigation(state.activeRunbook);
    return;
  }
  const unlocked = maxUnlockedStep();
  elements.stepNavigation.innerHTML = steps
    .map((step, index) => {
      const complete = state.completed.has(step.id);
      const active = index === state.currentStep;
      const locked = index > unlocked;
      const className = ["step-button", complete ? "complete" : "", active ? "active" : "", locked ? "locked" : ""]
        .filter(Boolean)
        .join(" ");
      const stateText = active
        ? t("currentStep")
        : complete
          ? t("complete")
          : locked
            ? t("locked")
            : t("clickToReview");
      return `
        <button class="${className}" type="button" data-step="${index}" ${locked ? "disabled" : ""}>
          <span class="step-number">${complete && !active ? materialIcon("check", "small") : index + 1}</span>
          <span class="step-title">${escapeHtml(t(step.title))}</span>
          <span class="step-state">${escapeHtml(stateText)}</span>
        </button>`;
    })
    .join("");

  elements.stepNavigation.querySelectorAll("[data-step]").forEach((button) => {
    button.addEventListener("click", () => {
      state.currentStep = Number(button.dataset.step);
      render();
      elements.mainContent.focus();
    });
  });

  const activeButton = elements.stepNavigation.querySelector(".step-button.active");
  window.requestAnimationFrame(() => {
    const runbook = document.querySelector(".runbook");
    if (!runbook) return;
    if (state.currentStep === 0) runbook.scrollTop = 0;
    else activeButton?.scrollIntoView({ block: "nearest" });
  });
}

function renderGcodeNavigation() {
  const pairedMode = state.gcode.importMode === "paired";
  const hasResult = pairedMode ? Boolean(state.pairedImport.result) : Boolean(state.gcode.result);
  const parsing = pairedMode ? state.pairedImport.checking : state.gcode.parsing;
  const hasFile = pairedMode
    ? Boolean(state.pairedImport.trajectoryFile && state.pairedImport.extrusionFile)
    : Boolean(state.gcode.file);
  const currentIndex = parsing
    ? 1
    : hasResult
      ? 2
      : 0;
  elements.stepNavigation.innerHTML = gcodeSteps
    .map((step, index) => {
      const complete = index === 0
        ? hasFile
        : hasResult;
      const active = index === currentIndex;
      const locked = index > currentIndex && !complete;
      const stateText = active
        ? parsing
          ? t("gcodeStageParsing")
          : t("gcodeStageReview")
        : complete
          ? t("gcodeStageComplete")
          : t("gcodeStageWaiting");
      const className = [
        "step-button",
        complete ? "complete" : "",
        active ? "active" : "",
        locked ? "locked" : "",
      ].filter(Boolean).join(" ");
      return `
        <button class="${className}" type="button" data-gcode-step="${index}" ${locked ? "disabled" : ""}>
          <span class="step-number">${complete && !active ? materialIcon("check", "small") : index + 1}</span>
          <span class="step-title">${escapeHtml(t(step.title))}</span>
          <span class="step-state">${escapeHtml(stateText)}</span>
        </button>`;
    })
    .join("");

  elements.stepNavigation.querySelectorAll("[data-gcode-step]").forEach((button) => {
    button.addEventListener("click", () => {
      const index = Number(button.dataset.gcodeStep);
      if (pairedMode) {
        render();
        document.querySelector(index === 0 ? ".import-mode-card" : ".paired-import-card")
          ?.scrollIntoView({ behavior: "smooth", block: "start" });
        return;
      }
      state.gcode.view = "review";
      render();
      if (index === 0 && state.gcode.result) {
        document.getElementById("replaceGcodeButton")?.focus();
      } else {
        const target = index === 2
          ? document.getElementById("gcodeFindings")
          : document.getElementById("gcodeImportArea");
        target?.scrollIntoView({ behavior: "smooth", block: "start" });
      }
    });
  });
}

function moduleStepComplete(runbookId, index) {
  if (runbookId === "motion") {
    if (index === 0) return Boolean(state.gcode.plan);
    if (index === 1) return hasValidatedMoveItTrajectory();
  }
  if (runbookId === "execution") {
    if (index === 0) return Boolean(state.execution.package);
    if (index === 1) return ["running", "completed"].includes(state.execution.runtime?.state);
    if (index === 2) return state.execution.runtime?.state === "completed";
  }
  return false;
}

function moduleStepStatus(runbookId, index, active, complete) {
  if (active) return t("moduleCurrent");
  if (complete) return t("complete");
  if (moduleStepLocked(runbookId, index)) return t("locked");
  return t("moduleReady");
}

function moduleStepLocked(runbookId, index) {
  if (runbookId !== "execution" || index === 0) return false;
  if (!state.execution.package) return true;
  const runtimeState = state.execution.runtime?.state;
  if (index === 1) return false;
  if (index === 2) {
    return !["starting", "preflight", "running", "canceling", "completed", "failed", "canceled", "stop-unconfirmed"].includes(runtimeState);
  }
  return runtimeState !== "completed";
}

function renderModuleNavigation(runbookId) {
  const runbook = runbooks[runbookId];
  const currentIndex = state.moduleSteps[runbookId] || 0;
  elements.stepNavigation.innerHTML = runbook.steps.map((step, index) => {
    const complete = moduleStepComplete(runbookId, index);
    const active = index === currentIndex;
    const locked = moduleStepLocked(runbookId, index) && !active;
    const className = [
      "step-button",
      complete ? "complete" : "",
      active ? "active" : "",
      locked ? "locked" : "",
    ].filter(Boolean).join(" ");
    return `
      <button class="${className}" type="button" data-module-step="${index}" ${locked ? "disabled" : ""}>
        <span class="step-number">${complete && !active ? materialIcon("check", "small") : index + 1}</span>
        <span class="step-title">${escapeHtml(t(step.title))}</span>
        <span class="step-state">${escapeHtml(moduleStepStatus(runbookId, index, active, complete))}</span>
      </button>`;
  }).join("");

  elements.stepNavigation.querySelectorAll("[data-module-step]").forEach((button) => {
    button.addEventListener("click", async () => {
      const index = Number(button.dataset.moduleStep);
      state.moduleSteps[runbookId] = index;
      if (runbookId === "motion") {
        state.gcode.view = index === 1 ? "path-review" : "planning";
        if (index === 1) {
          await openGcodePathReview();
          return;
        }
      }
      if (runbookId === "execution" && state.execution.package) {
        await refreshPrintExecutionStatus(false);
      }
      render();
      elements.mainContent.focus();
    });
  });
}

function stepHeader(kicker, title, lead) {
  return `
    <header class="step-header">
      <div class="step-kicker">${escapeHtml(kicker)}</div>
      <h1>${escapeHtml(title)}</h1>
      <p class="lead">${escapeHtml(lead)}</p>
    </header>`;
}

function checklistRow(id, title, description, checked) {
  return `
    <label class="check-row" for="${id}">
      <input id="${id}" type="checkbox" ${checked ? "checked" : ""}>
      <span><strong>${escapeHtml(title)}</strong><span>${escapeHtml(description)}</span></span>
    </label>`;
}

function sideSnapshot() {
  return `
    <section class="info-panel rviz-panel">
      <div class="info-panel-header">
        <strong class="panel-title">${materialIcon("view_in_ar")} ${escapeHtml(t("latestRviz"))}</strong>
        <span class="inline-status"><span class="status-dot neutral"></span>${escapeHtml(t("referenceOnly"))}</span>
      </div>
      <img class="snapshot" src="/api/rviz-snapshot" alt="UR3e in RViz">
    </section>`;
}

function profilePanel() {
  const route = state.results.ur?.route || {};
  return `
    <section class="info-panel">
      <div class="info-panel-header"><strong class="panel-title">${materialIcon("lan")} ${escapeHtml(t("connectionProfile"))}</strong></div>
      <div class="info-panel-body fact-list">
        <div class="fact-row"><span>${escapeHtml(t("robotIp"))}</span><code>${escapeHtml(state.config.ur_robot_ip)}</code><span></span></div>
        <div class="fact-row"><span>${escapeHtml(t("routeInterface"))}</span><code>${escapeHtml(route.interface || "—")}</code><span></span></div>
        <div class="fact-row"><span>${escapeHtml(t("localAddress"))}</span><code>${escapeHtml(route.address || state.config.ros_pc_ip || "Auto")}</code><span></span></div>
      </div>
    </section>`;
}

function checkById(result, id) {
  return result?.checks?.find((item) => item.id === id) || null;
}

function robotPreviewCard(modeLabel = "") {
  const checked = Boolean(state.results.ur);
  const online = Boolean(state.results.ur?.ok);
  const route = state.results.ur?.route || {};
  const dashboard = checkById(state.results.ur, "dashboard");
  const rtde = checkById(state.results.ur, "rtde");
  const badgeClass = online ? "pass" : checked ? "fail" : "neutral";
  const badgeText = online ? t("online") : checked ? t("offline") : t("awaitingCheck");
  return `
    <section class="visual-card robot-preview-card">
      <div class="visual-card-header">
        <strong>${escapeHtml(t("urPreview"))}</strong>
        <span class="status-badge ${badgeClass}"><span class="status-dot ${online ? "pass" : checked ? "fail" : "neutral"}"></span>${escapeHtml(modeLabel || badgeText)}</span>
      </div>
      <div class="robot-image-stage">
        <img src="/assets/ur3e-preview.png" alt="UR3e collaborative robot illustration">
      </div>
      <div class="preview-metrics">
        <div><span>${escapeHtml(t("routeInterface"))}</span><strong>${escapeHtml(route.interface || "—")}</strong></div>
        <div><span>Dashboard</span><strong class="${dashboard?.ok ? "pass-text" : ""}">${escapeHtml(dashboard ? (dashboard.ok ? t("ready") : t("waiting")) : "—")}</strong></div>
        <div><span>RTDE</span><strong class="${rtde?.ok ? "pass-text" : ""}">${escapeHtml(rtde ? (rtde.ok ? t("ready") : t("waiting")) : "—")}</strong></div>
      </div>
      <div class="visual-card-caption">${materialIcon("info", "small")} ${escapeHtml(t("illustrativeModel"))}</div>
    </section>`;
}

function systemOverviewPanel() {
  return `
    <section class="visual-card system-overview-card">
      <div class="visual-card-header"><strong>${escapeHtml(t("systemOverview"))}</strong><span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span></div>
      <div class="sync-control-flow">
        <div class="flow-node wide-node">${materialIcon("computer")}<span><strong>${escapeHtml(t("workstationApp"))}</strong><small>${escapeHtml(t("macWorkstation"))}</small></span></div>
        ${materialIcon("south", "flow-arrow")}
        <div class="flow-node wide-node">${materialIcon("hub")}<span><strong>${escapeHtml(t("rosCoordinator"))}</strong><small>ROS 2</small></span></div>
        <div class="sync-flow-branches">
          <div class="sync-flow-branch">
            ${materialIcon("south", "flow-arrow")}
            <div class="flow-node wide-node">${materialIcon("route")}<span><strong>${escapeHtml(t("moveitUrDriver"))}</strong><small>UR3e</small></span></div>
          </div>
          <div class="sync-flow-branch">
            ${materialIcon("south", "flow-arrow")}
            <div class="flow-node wide-node">${materialIcon("lan")}<span><strong>${escapeHtml(t("klipperBridge"))}</strong><small>Moonraker → Klipper</small></span></div>
          </div>
        </div>
      </div>
    </section>`;
}

function klipperVisualPanel() {
  const checked = Boolean(state.results.klipper);
  const online = Boolean(state.results.klipper?.ok);
  return `
    <section class="visual-card">
      <div class="visual-card-header"><strong>${escapeHtml(t("visualConnection"))}</strong><span class="status-badge ${online ? "pass" : checked ? "fail" : "neutral"}">${escapeHtml(online ? t("online") : checked ? t("offline") : t("awaitingCheck"))}</span></div>
      <div class="device-flow vertical">
        <div class="flow-node wide-node">${materialIcon("computer")}<span><strong>${escapeHtml(t("macWorkstation"))}</strong><small>${escapeHtml(state.config.klipper_url)}</small></span></div>
        ${materialIcon("south", "flow-arrow")}
        <div class="flow-node wide-node">${materialIcon("router")}<span><strong>${escapeHtml(t("moonrakerService"))}</strong><small>HTTP API</small></span></div>
        ${materialIcon("south", "flow-arrow")}
        <div class="flow-node wide-node">${materialIcon("memory")}<span><strong>${escapeHtml(t("klipperController"))}</strong><small>${escapeHtml(state.config.manual_stepper)}</small></span></div>
      </div>
      <div class="visual-card-caption">${materialIcon("visibility", "small")} ${escapeHtml(t("noGcode"))}</div>
    </section>`;
}

function rosPipelinePanel(driver, external, moveit, bridge) {
  const stages = [
    ["deployed_code", t("docker"), Boolean(state.rosStatus?.container_running)],
    ["precision_manufacturing", t("urDriver"), driver],
    ["cable", t("externalControl"), external],
    ["route", t("moveit"), moveit],
    ["lan", t("klipperBridge"), bridge],
  ];
  const ready = moveit && bridge;
  return `
    <section class="visual-card">
      <div class="visual-card-header"><strong>${escapeHtml(t("rosPipeline"))}</strong><span class="status-badge ${ready ? "pass" : "neutral"}">${escapeHtml(ready ? t("ready") : t("pending"))}</span></div>
      <div class="pipeline-stack">
        ${stages.map(([iconName, label, complete], index) => `
          <div class="pipeline-stage ${complete ? "complete" : ""}">
            <span class="stage-icon">${materialIcon(iconName)}</span>
            <span><strong>${escapeHtml(label)}</strong><small>${escapeHtml(complete ? t("ready") : t("waiting"))}</small></span>
            ${materialIcon(complete ? "check_circle" : "radio_button_unchecked", complete ? "pass-text" : "muted-icon")}
          </div>${index < stages.length - 1 ? `<div class="pipeline-line ${complete ? "complete" : ""}"></div>` : ""}`)
          .join("")}
      </div>
    </section>`;
}

function syncVisualPanel() {
  return `
    <section class="visual-card sync-visual-card">
      <div class="visual-card-header"><strong>${escapeHtml(t("visualConnection"))}</strong><span class="status-badge warning">${escapeHtml(t("guardedMotion"))}</span></div>
      <div class="sync-control-flow">
        <div class="flow-node wide-node">${materialIcon("computer")}<span><strong>${escapeHtml(t("workstationApp"))}</strong><small>${escapeHtml(t("macWorkstation"))}</small></span></div>
        ${materialIcon("south", "flow-arrow")}
        <div class="flow-node wide-node coordinator-node">${materialIcon("hub")}<span><strong>${escapeHtml(t("rosCoordinator"))}</strong><small>ROS 2</small></span></div>
        <div class="sync-flow-branches">
          <div class="sync-flow-branch">
            ${materialIcon("south", "flow-arrow")}
            <div class="flow-node wide-node">${materialIcon("route")}<span><strong>${escapeHtml(t("moveitUrDriver"))}</strong><small>UR3e</small></span></div>
          </div>
          <div class="sync-flow-branch">
            ${materialIcon("south", "flow-arrow")}
            <div class="flow-node wide-node">${materialIcon("lan")}<span><strong>${escapeHtml(t("klipperBridge"))}</strong><small>${escapeHtml(t("moonrakerService"))} → Klipper</small></span></div>
          </div>
        </div>
      </div>
    </section>`;
}

function resultMarkup(result, emptyText = t("notCheckedYet")) {
  if (!result) return `<div class="empty-result">${materialIcon("pending_actions")}<span>${escapeHtml(emptyText)}</span></div>`;
  const rows = result.checks
    .map((item) => {
      const visual = item.ok ? "pass" : item.severity === "warn" ? "warning" : "fail";
      const label = item.ok ? t("pass") : item.severity === "warn" ? t("warn") : t("fail");
      return `
        <div class="result-row">
          <strong>${escapeHtml(item.label)}</strong>
          <span class="result-icon ${visual}">${materialIcon(item.ok ? "check_circle" : item.severity === "warn" ? "warning" : "error")}</span>
          <span class="result-detail">${escapeHtml(item.detail)}</span>
          <span class="result-status">${escapeHtml(label)}</span>
        </div>`;
    })
    .join("");
  return `
    <div class="connection-results">${rows}</div>
    <div class="result-summary ${result.ok ? "pass" : "fail"}">${escapeHtml(result.summary)}</div>`;
}

function renderSafety() {
  elements.mainContent.innerHTML = `
    <div class="content-grid">
      <div class="content-column">
        ${stepHeader(t("safetyKicker"), t("safetyTitle"), t("safetyLead"))}
        <section class="section">
          <h2>${escapeHtml(t("safetyChecks"))}</h2>
          <div class="checklist">
            ${checklistRow("safetyEstop", t("estopTitle"), t("estopDesc"), state.safety.emergency_stop)}
            ${checklistRow("safetyClear", t("clearTitle"), t("clearDesc"), state.safety.workspace_clear)}
            ${checklistRow("safetySpeed", t("speedTitle"), t("speedDesc"), state.safety.speed_under_20)}
            ${checklistRow("safetyWired", t("wiredTitle"), t("wiredDesc"), state.safety.wired_network)}
          </div>
          <div class="button-row">
            <button id="confirmSafetyButton" class="button primary" type="button" ${allSafetyConfirmed() ? "" : "disabled"}>${materialIcon("verified_user")} ${escapeHtml(t("confirmSafety"))}</button>
          </div>
        </section>
        <section class="section">
          <h2>${escapeHtml(t("activeControlsTitle"))}</h2>
          <p class="muted">${escapeHtml(t("activeControlsLead"))}</p>
          <div class="button-row">
            <button id="stopMoveitEarlyButton" class="button danger-quiet" type="button" ${jobIsRunning() ? "disabled" : ""}>${materialIcon("stop_circle")} ${escapeHtml(t("stopMoveitEarly"))}</button>
            <button id="stopRosDriverEarlyButton" class="button danger" type="button" ${jobIsRunning() ? "disabled" : ""}>${materialIcon("power_settings_new")} ${escapeHtml(t("stopRosDriverEarly"))}</button>
          </div>
        </section>
        ${jobMarkup()}
      </div>
      <aside class="side-column">
        ${robotPreviewCard()}
        <section class="info-panel">
          <div class="info-panel-header"><strong class="panel-title">${materialIcon("health_and_safety")} ${escapeHtml(t("safetyPrinciples"))}</strong></div>
          <div class="info-panel-body">
            <div class="checklist">
              <div class="check-row principle-row">${materialIcon("warning", "principle-icon filled")}<span><strong>${escapeHtml(t("safetyBoundary1"))}</strong></span></div>
              <div class="check-row principle-row">${materialIcon("lock", "principle-icon")}<span><strong>${escapeHtml(t("safetyBoundary2"))}</strong></span></div>
              <div class="check-row principle-row">${materialIcon("fact_check", "principle-icon")}<span><strong>${escapeHtml(t("safetyBoundary3"))}</strong></span></div>
            </div>
          </div>
        </section>
      </aside>
    </div>`;

  const bindings = [
    ["safetyEstop", "emergency_stop"],
    ["safetyClear", "workspace_clear"],
    ["safetySpeed", "speed_under_20"],
    ["safetyWired", "wired_network"],
  ];
  bindings.forEach(([id, key]) => {
    document.getElementById(id).addEventListener("change", (event) => {
      state.safety[key] = event.target.checked;
      renderSafety();
      updateFooter();
    });
  });
  document.getElementById("confirmSafetyButton").addEventListener("click", () => {
    if (!allSafetyConfirmed()) {
      notify(t("safetyIncomplete"), "error");
      return;
    }
    state.completed.add("safety");
    state.currentStep = 1;
    render();
  });
  document.getElementById("stopMoveitEarlyButton")?.addEventListener("click", () => {
        startJob("/api/moveit/stop", { operator_action: "stop-moveit" }, () => {
          state.rosStages.moveit = false;
          state.rosStages.bridge = false;
          state.completed.delete("ros");
          state.completed.delete("dry");
          state.completed.delete("sync");
        });
  });
  document.getElementById("stopRosDriverEarlyButton")?.addEventListener("click", () => {
        startJob("/api/ros/driver/stop", { operator_action: "stop-ros-driver" }, () => {
          state.rosStages = { driver: false, external: false, moveit: false, bridge: false };
          state.completed.delete("ros");
          state.completed.delete("dry");
          state.completed.delete("sync");
        });
  });
  bindJobCancel();
}

function renderUr() {
  elements.mainContent.innerHTML = `
    <div class="ur-screen-grid">
      <div class="ur-primary">
        ${stepHeader(t("urKicker"), t("urTitle"), t("urLead"))}
        <section class="section">
          <h2>${escapeHtml(t("operatorPrep"))}</h2>
          <div class="checklist">
            <div class="check-row"><span class="workflow-index">1</span><span><strong>${escapeHtml(t("urPrep1"))}</strong></span></div>
            <div class="check-row"><span class="workflow-index">2</span><span><strong>${escapeHtml(t("urPrepCombined"))}</strong></span></div>
          </div>
        </section>
        <section class="section">
          <label class="ip-field"><span>${escapeHtml(t("robotIp"))}</span><input id="urIpInput" value="${escapeHtml(state.config.ur_robot_ip)}" inputmode="decimal"><small>${escapeHtml(t("controlPorts"))}: 29999 (Dashboard) · 30004 (RTDE)</small></label>
          <div class="button-row"><button id="runUrCheckButton" class="button primary" type="button">${materialIcon("play_arrow", "filled")} ${escapeHtml(t("runUrCheck"))}</button></div>
        </section>
      </div>
      <aside class="ur-preview">
        ${robotPreviewCard()}
      </aside>
      <section class="section ur-results">
        <h2>${escapeHtml(t("connectionResults"))}</h2>
        ${resultMarkup(state.results.ur)}
      </section>
    </div>`;

  document.getElementById("runUrCheckButton").addEventListener("click", runUrCheck);
}

async function runUrCheck() {
  const button = document.getElementById("runUrCheckButton");
  button.disabled = true;
  try {
    await saveConfig({ ur_robot_ip: document.getElementById("urIpInput").value.trim() }, false);
    state.results.ur = await api("/api/ur/check", { method: "POST", body: {} });
    if (state.results.ur.ok) {
      state.completed.add("ur");
      notify(t("urCheckPassed"), "success");
    } else {
      state.completed.delete("ur");
      notify(t("urCheckFailed"), "error");
    }
  } catch (error) {
    notify(error.message, "error");
  } finally {
    render();
  }
}

function renderKlipper() {
  elements.mainContent.innerHTML = `
    <div class="content-grid">
      <div class="content-column">
        ${stepHeader(t("klipperKicker"), t("klipperTitle"), t("klipperLead"))}
        <section class="section">
          <h2>${escapeHtml(t("klipperPrep"))}</h2>
          <div class="checklist">
            <div class="check-row"><span class="workflow-index">1</span><span><strong>${escapeHtml(t("klipperPrep1"))}</strong></span></div>
            <div class="check-row"><span class="workflow-index">2</span><span><strong>${escapeHtml(t("klipperPrep2"))}</strong></span></div>
            <div class="check-row"><span class="workflow-index">3</span><span><strong>${escapeHtml(t("klipperPrep3"))}</strong></span></div>
          </div>
        </section>
        <section class="section field-grid">
          <label><span>${escapeHtml(t("moonrakerUrl"))}</span><input id="klipperUrlInput" value="${escapeHtml(state.config.klipper_url)}" type="url"></label>
          <label><span>${escapeHtml(t("manualStepper"))}</span><input id="stepperInput" value="${escapeHtml(state.config.manual_stepper)}"></label>
        </section>
        <div class="button-row"><button id="runKlipperCheckButton" class="button primary" type="button">${materialIcon("play_arrow", "filled")} ${escapeHtml(t("runKlipperCheck"))}</button></div>
        <section class="section">
          <h2>${escapeHtml(t("connectionResults"))}</h2>
          ${resultMarkup(state.results.klipper)}
        </section>
      </div>
      <aside class="side-column">
        ${klipperVisualPanel()}
        <section class="info-panel">
          <div class="info-panel-header"><strong class="panel-title">${materialIcon("tune")} ${escapeHtml(t("connectionProfile"))}</strong><span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span></div>
          <div class="info-panel-body fact-list">
            <div class="fact-row"><span>${escapeHtml(t("endpoint"))}</span><code>${escapeHtml(state.config.klipper_url)}</code><span></span></div>
            <div class="fact-row"><span>${escapeHtml(t("stepper"))}</span><code>${escapeHtml(state.config.manual_stepper)}</code><span></span></div>
          </div>
        </section>
      </aside>
    </div>`;
  document.getElementById("runKlipperCheckButton").addEventListener("click", runKlipperCheck);
}

async function runKlipperCheck() {
  const button = document.getElementById("runKlipperCheckButton");
  button.disabled = true;
  try {
    await saveConfig(
      {
        klipper_url: document.getElementById("klipperUrlInput").value.trim(),
        manual_stepper: document.getElementById("stepperInput").value.trim(),
      },
      false,
    );
    state.results.klipper = await api("/api/klipper/check", { method: "POST", body: {} });
    if (state.results.klipper.ok) {
      state.completed.add("klipper");
      notify(t("klipperCheckPassed"), "success");
    } else {
      state.completed.delete("klipper");
      notify(t("klipperCheckFailed"), "error");
    }
  } catch (error) {
    notify(error.message, "error");
  } finally {
    render();
  }
}

function statusFact(label, ok, pending = false) {
  const visual = ok ? "pass" : pending ? "neutral" : "warning";
  const text = ok ? t("ready") : pending ? t("pending") : t("waiting");
  return `<div class="fact-row"><span>${escapeHtml(label)}</span><span></span><span class="result-status"><span class="status-dot ${visual}"></span>${escapeHtml(text)}</span></div>`;
}

function workflowAction(index, title, description, complete, enabled, actionId) {
  return `
    <div class="workflow-action ${complete ? "complete" : ""}">
      <span class="workflow-index">${complete ? materialIcon("check", "small") : index}</span>
      <span><strong>${escapeHtml(title)}</strong><span class="step-state">${escapeHtml(description)}</span></span>
      <button id="${actionId}" class="button ${complete ? "quiet" : "secondary"} compact" type="button" ${!enabled || complete ? "disabled" : ""}>${escapeHtml(complete ? t("complete") : t("runAction"))}</button>
    </div>`;
}

function jobIsRunning() {
  return Boolean(state.jobSubmitting || state.jobCanceling || (state.activeJob && ["queued", "running"].includes(state.activeJob.state)));
}

function renderRos() {
  const ros = state.rosStatus || {};
  // Demo mode must reflect the actions taken in this runbook, not the simulated
  // backend's always-ready hardware state.
  // Real mode must always follow live evidence so a previously completed UI
  // step cannot make a failed or restarted driver look ready.
  const driver = state.demo ? state.rosStages.driver : Boolean(ros.driver_started);
  const external = state.demo ? state.rosStages.external : Boolean(ros.external_control);
  const moveit = state.demo ? state.rosStages.moveit : Boolean(ros.moveit_ready);
  const bridge = state.demo ? state.rosStages.bridge : Boolean(ros.klipper_bridge_ready);
  const stackReady = moveit && bridge;
  const attention = ros.rtde_conflict
    ? t("rtdeConflict")
    : ros.host_ports_ready === false
      ? t("hostPortConflict")
      : moveit && !bridge
        ? t("klipperBridgeNeedsAttention")
      : t("rosNeedsAttention");
  elements.mainContent.innerHTML = `
    <div class="content-grid">
      <div class="content-column">
        ${stepHeader(t("rosKicker"), t("rosTitle"), t("rosLead"))}
        <div class="callout warning">${escapeHtml(t("pressPlay"))}</div>
        <section class="section workflow-actions">
          ${workflowAction(1, t("startDriver"), t("startDriverDesc"), driver, !jobIsRunning(), "startRosButton")}
          ${workflowAction(2, t("waitExternal"), t("waitExternalDesc"), external, driver && !jobIsRunning(), "waitExternalButton")}
          ${workflowAction(3, t("startMoveit"), t("startMoveitDesc"), moveit, external && !jobIsRunning(), "startMoveitButton")}
        </section>
        <section class="section ros-restart">
          <span class="restart-icon">${materialIcon("restart_alt")}</span>
          <span class="restart-copy"><strong>${escapeHtml(t("restartRosHelpTitle"))}</strong><span class="step-state">${escapeHtml(t("restartRosHelp"))}</span></span>
          <button id="restartRosButton" class="button secondary compact" type="button" ${jobIsRunning() ? "disabled" : ""}>${materialIcon("restart_alt")} ${escapeHtml(t("restartRos"))}</button>
        </section>
        ${jobMarkup()}
      </div>
      <aside class="side-column">
        ${rosPipelinePanel(driver, external, moveit, bridge)}
        ${sideSnapshot()}
        <div class="callout ${stackReady ? "" : "warning"}">${escapeHtml(stackReady ? t("rosReadyCallout") : attention)}</div>
      </aside>
    </div>`;

  document.getElementById("startRosButton")?.addEventListener("click", () => {
    startJob("/api/ros/start", safetyPayload(), () => {
      state.rosStages.driver = true;
    });
  });
  document.getElementById("waitExternalButton")?.addEventListener("click", () => {
    startJob("/api/ros/wait-external", safetyPayload(), () => {
      state.rosStages.external = true;
    });
  });
  document.getElementById("startMoveitButton")?.addEventListener("click", () => {
    startJob("/api/moveit/start", safetyPayload(), () => {
      state.rosStages.moveit = true;
      if (state.demo) {
        state.rosStages.bridge = true;
        state.completed.add("ros");
      }
    });
  });
  document.getElementById("restartRosButton")?.addEventListener("click", () => {
        startJob(
          "/api/ros/restart",
          { ...safetyPayload(), operator_action: "restart-ros" },
          () => {
            state.rosStages = { driver: true, external: false, moveit: false, bridge: false };
          },
          () => {
            state.rosStages = { driver: false, external: false, moveit: false, bridge: false };
            state.completed.delete("ros");
            state.completed.delete("dry");
            state.completed.delete("sync");
            notify(t("restartRosAccepted"));
          },
        );
  });
  bindJobCancel();
}

function jobMarkup() {
  if (!state.activeJob) return "";
  const job = state.activeJob;
  const active = ["queued", "running"].includes(job.state);
  const visual = job.state === "succeeded" ? "pass" : job.state === "failed" ? "fail" : job.state === "canceled" ? "warning" : "neutral";
  return `
    <section class="job-panel">
      <div class="job-panel-header">
        <span class="inline-status"><span class="status-dot ${visual}"></span>${escapeHtml(t(job.state))}</span>
        ${active ? `<button id="cancelJobButton" class="button quiet compact" type="button" ${state.jobCanceling ? "disabled" : ""}>${escapeHtml(t("cancelOperation"))}</button>` : ""}
      </div>
      <pre class="job-log">${escapeHtml(job.log || t("noOutput"))}</pre>
    </section>`;
}

function bindJobCancel() {
  document.getElementById("cancelJobButton")?.addEventListener("click", cancelActiveJob);
}

async function startJob(endpoint, payload, onSuccess, onAccepted) {
  if (state.jobSubmitting || state.jobCanceling) return;
  state.jobSubmitting = true;
  render();
  try {
    const response = await api(endpoint, { method: "POST", body: payload });
    state.activeJob = response.job;
    state.jobSuccessHandler = onSuccess;
    onAccepted?.();
    render();
    pollJob(response.job.id);
  } catch (error) {
    notify(error.message, "error");
  } finally {
    state.jobSubmitting = false;
    render();
  }
}

async function pollJob(jobId) {
  if (state.activeJob?.id !== jobId) return;
  window.clearTimeout(state.jobPollTimer);
  state.jobPollTimer = null;
  const generation = (state.jobPollGeneration || 0) + 1;
  state.jobPollGeneration = generation;
  const isCurrent = () => state.activeJob?.id === jobId && state.jobPollGeneration === generation;
  const schedule = delay => {
    if (!isCurrent()) return;
    state.jobPollTimer = window.setTimeout(() => {
      if (!isCurrent()) return;
      state.jobPollTimer = null;
      return pollJob(jobId);
    }, delay);
  };
  try {
    const response = await api(`/api/jobs/${jobId}`);
    if (!isCurrent()) return;
    state.activeJob = response.job;
    const active = ["queued", "running"].includes(response.job.state);
    if (active) {
      render();
      schedule(650);
      return;
    }
    const onSuccess = state.jobSuccessHandler;
    state.jobSuccessHandler = null;
    if (response.job.state === "succeeded") {
      onSuccess?.();
      notify(t("operationSucceeded"), "success");
    } else if (response.job.state === "canceled") {
      notify(t("operationCanceled"));
    } else {
      notify(t("operationFailed"), "error");
    }
    await refreshStatus(false);
    if (isCurrent()) render();
  } catch (error) {
    if (!isCurrent()) return;
    if (error.status === 404) {
      const missingJob = state.activeJob;
      state.jobSuccessHandler = null;
      // A restarted external backend has an empty in-memory job registry.
      // Fresh server status can release only a lost service operation's UI
      // lock; absent job records never prove that robot motion has stopped.
      const status = await refreshStatus(false);
      if (!isCurrent()) return;
      const serviceOperation = missingJob.motion === false && [
        "start-ros", "restart-ros", "external-control", "start-moveit",
        "stop-ros", "stop-moveit", "stop-ros-driver",
      ].includes(missingJob.kind);
      if (status && !status.active_job && state.activeJob === missingJob && serviceOperation) {
        state.activeJob = null;
        notify(t("operationRecordLost"));
        render();
        return;
      }
      if (missingJob.motion) {
        notify(t("operationMotionRecordLost"), "error");
        schedule(1500);
        return;
      }
    }
    notify(error.message, "error");
    // A single lost status response must not strand a completed/canceled
    // operation in the UI's running state and disable every future Start.
    schedule(1500);
  }
}

async function cancelActiveJob() {
  if (!state.activeJob || state.jobCanceling || !["queued", "running"].includes(state.activeJob.state)) return;
  const jobId = state.activeJob.id;
  state.jobCanceling = jobId;
  render();
  try {
    const response = await api(`/api/jobs/${jobId}/cancel`, { method: "POST", body: {}, timeoutMs: 65000 });
    if (state.activeJob?.id !== jobId) return;
    state.activeJob = response.job;
    pollJob(jobId);
  } catch (error) {
    if (state.activeJob?.id === jobId) notify(error.message, "error");
  } finally {
    if (state.jobCanceling === jobId) state.jobCanceling = null;
    render();
  }
}

function formatFileSize(bytes) {
  const value = Number(bytes || 0);
  if (value < 1024) return `${value} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KB`;
  return `${(value / (1024 * 1024)).toFixed(1)} MB`;
}

function formatDuration(seconds) {
  const total = Math.max(0, Math.round(Number(seconds || 0)));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const remainingSeconds = total % 60;
  if (hours) {
    return `${hours}${t("durationHour")} ${String(minutes).padStart(2, "0")}${t("durationMinute")}`;
  }
  if (minutes) {
    return `${minutes}${t("durationMinute")} ${String(remainingSeconds).padStart(2, "0")}${t("durationSecond")}`;
  }
  return `${remainingSeconds}${t("durationSecond")}`;
}

function formatNumber(value, maximumFractionDigits = 1) {
  const locale = "en-US";
  return new Intl.NumberFormat(locale, { maximumFractionDigits }).format(Number(value || 0));
}

function gcodeMetric(icon, label, value, unit = "") {
  return `
    <div class="gcode-metric">
      <span class="gcode-metric-icon">${materialIcon(icon)}</span>
      <span class="gcode-metric-copy">
        <small>${escapeHtml(label)}</small>
        <strong>${escapeHtml(value)}${unit ? `<span>${escapeHtml(unit)}</span>` : ""}</strong>
      </span>
    </div>`;
}

function importModeMarkup() {
  const mode = state.gcode.importMode;
  return `
    <section class="import-mode-card" aria-label="${escapeHtml(t("importModeTitle"))}">
      <h2>${escapeHtml(t("importModeTitle"))}</h2>
      <div class="import-mode-options">
        <button type="button" data-import-mode="gcode" class="import-mode-option ${mode === "gcode" ? "selected" : ""}" aria-pressed="${mode === "gcode"}">
          <span>${materialIcon("description")}</span><strong>${escapeHtml(t("importModeGcode"))}</strong><small>${escapeHtml(t("importModeGcodeDesc"))}</small>
        </button>
        <button type="button" data-import-mode="paired" class="import-mode-option ${mode === "paired" ? "selected" : ""}" aria-pressed="${mode === "paired"}">
          <span>${materialIcon("route")}</span><strong>${escapeHtml(t("importModePaired"))}</strong><small>${escapeHtml(t("importModePairedDesc"))}</small>
        </button>
      </div>
    </section>`;
}

function clearImportedJob() {
  stopPathReviewPlayback();
  destroyPathReviewViewer();
  state.gcode.importRequestId += 1;
  state.gcode.pathReviewRequestId += 1;
  Object.assign(state.gcode, {
    file: null, sourceFile: null, parsing: false, result: null, error: "", view: "review",
    planning: false, plan: null, planError: "", pathReviewLoading: false,
    pathReviewJob: null, pathReview: null, pathReviewError: "",
    pathReviewMoveitError: "", pathReviewMoveitRunning: false,
    pathReviewAutoValidationAttempted: false, pathReviewRequestedSimulation: false,
    pathReviewMoveitJob: null, pathReviewMoveitResult: null,
    pathReviewSimulationResult: null, pathReviewProgress: 0,
    pathReviewPlaying: false, pathReviewLayer: "all",
  });
  state.pairedImport.requestId += 1;
  Object.assign(state.pairedImport, {
    trajectoryFile: null, extrusionFile: null, checking: false, result: null, error: "",
  });
  state.moduleSteps.motion = 0;
  resetPrintExecution();
}

function bindImportMode() {
  document.querySelectorAll("[data-import-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      const next = button.dataset.importMode;
      if (next === state.gcode.importMode) return;
      if (jobIsRunning() || state.execution.preparing
          || ["queued", "running"].includes(state.execution.job?.state)
          || ["starting", "preflight", "running", "canceling"].includes(state.execution.runtime?.state)) {
        notify(t("modeChangeBusy"), "error");
        return;
      }
      clearImportedJob();
      state.gcode.importMode = next;
      render();
    });
  });
}

function gcodeImportMarkup() {
  const parsing = state.gcode.parsing;
  return `
    <section id="gcodeImportArea" class="gcode-import-card ${parsing ? "parsing" : ""}">
      <input id="gcodeFileInput" type="file" accept=".gcode,.gc,.ngc,.nc,text/x-gcode" hidden>
      <div id="gcodeDropZone" class="gcode-drop-zone" role="button" tabindex="${parsing ? "-1" : "0"}" aria-disabled="${parsing}">
        <span class="gcode-upload-icon">${materialIcon(parsing ? "progress_activity" : "upload_file")}</span>
        <h2>${escapeHtml(t(parsing ? "parsingGcode" : "dropGcode"))}</h2>
        <p>${escapeHtml(t(parsing ? "parsingGcodeDesc" : "dropGcodeLead"))}</p>
        ${parsing ? `
          <div class="parse-progress" aria-hidden="true"><span></span></div>
        ` : `
          <button id="chooseGcodeButton" class="button primary" type="button">${materialIcon("folder_open")} ${escapeHtml(t("chooseFile"))}</button>
          <small>${escapeHtml(t("supportedGcode"))}</small>
        `}
      </div>
    </section>`;
}

function renderPairedImport() {
  const paired = state.pairedImport;
  const result = paired.result;
  elements.mainContent.innerHTML = `
    <div class="gcode-empty-page">
      ${stepHeader(t("gcodeKicker"), t("pairedTitle"), t("pairedLead"))}
      ${importModeMarkup()}
      <section class="paired-import-card">
        <div class="paired-import-fields">
          <div class="paired-import-field">
            <strong>${escapeHtml(t("pairedTrajectory"))}</strong>
            <div class="paired-file-picker">
              <input id="pairedTrajectoryFile" type="file" accept=".csv,text/csv" hidden ${paired.checking ? "disabled" : ""}>
              <button id="choosePairedTrajectory" type="button" class="button secondary" aria-label="${escapeHtml(`${t("pairedChoose")} ${t("pairedTrajectory")}`)}" ${paired.checking ? "disabled" : ""}>${escapeHtml(t("pairedChoose"))}</button>
              <span class="paired-file-name" role="status">${escapeHtml(paired.trajectoryFile?.name || t("pairedNoFile"))}</span>
            </div>
          </div>
          <div class="paired-import-field">
            <strong>${escapeHtml(t("pairedExtrusion"))}</strong>
            <div class="paired-file-picker">
              <input id="pairedExtrusionFile" type="file" accept=".gcode,text/x-gcode" hidden ${paired.checking ? "disabled" : ""}>
              <button id="choosePairedExtrusion" type="button" class="button secondary" aria-label="${escapeHtml(`${t("pairedChoose")} ${t("pairedExtrusion")}`)}" ${paired.checking ? "disabled" : ""}>${escapeHtml(t("pairedChoose"))}</button>
              <span class="paired-file-name" role="status">${escapeHtml(paired.extrusionFile?.name || t("pairedNoFile"))}</span>
            </div>
          </div>
        </div>
        <p class="muted">${escapeHtml(t("pairedFormat"))}</p>
        <button id="checkPairedImport" type="button" class="button primary" ${paired.checking ? "disabled" : ""}>${materialIcon(paired.checking ? "progress_activity" : "fact_check")} ${escapeHtml(t(paired.checking ? "pairedChecking" : "pairedCheck"))}</button>
        ${paired.error ? `<div class="callout danger gcode-import-error">${materialIcon("error")}<span>${escapeHtml(paired.error)}</span></div>` : ""}
      </section>
      ${result ? `
        <section class="paired-import-result">
          <div class="gcode-file-status pass">${materialIcon("task_alt")}<span><strong>${escapeHtml(t("pairedReady"))}</strong><small>${escapeHtml(result.trajectory.name)} + ${escapeHtml(result.extrusion.name)}</small></span></div>
          <div class="gcode-metric-grid">
            ${gcodeMetric("route", t("pairedPoints"), formatNumber(result.trajectory.point_count, 0))}
            ${gcodeMetric("schedule", t("pairedDuration"), formatDuration(result.trajectory.duration_s))}
            ${gcodeMetric("water_drop", t("pairedSegments"), formatNumber(result.extrusion.segment_count, 0))}
            ${gcodeMetric("undo", t("pairedRetract"), formatNumber(result.extrusion.retraction_e_mm, 3), " mm")}
          </div>
          <div class="callout warning">${materialIcon("lock")}<span>${escapeHtml(t("pairedLocked"))}</span></div>
        </section>` : ""}
      <div class="callout gcode-offline-callout">${materialIcon("shield_lock")}<span><strong>${escapeHtml(t("offlineByDesign"))}</strong>${escapeHtml(t("offlineByDesignDesc"))}</span></div>
    </div>`;
  bindImportMode();
  document.getElementById("choosePairedTrajectory")?.addEventListener("click", () => {
    document.getElementById("pairedTrajectoryFile")?.click();
  });
  document.getElementById("choosePairedExtrusion")?.addEventListener("click", () => {
    document.getElementById("pairedExtrusionFile")?.click();
  });
  document.getElementById("pairedTrajectoryFile")?.addEventListener("change", (event) => {
    paired.requestId += 1;
    paired.trajectoryFile = event.target.files?.[0] || null;
    paired.result = null;
    paired.error = "";
    render();
  });
  document.getElementById("pairedExtrusionFile")?.addEventListener("change", (event) => {
    paired.requestId += 1;
    paired.extrusionFile = event.target.files?.[0] || null;
    paired.result = null;
    paired.error = "";
    render();
  });
  document.getElementById("checkPairedImport")?.addEventListener("click", checkPairedImport);
}

async function checkPairedImport() {
  const paired = state.pairedImport;
  const trajectory = paired.trajectoryFile;
  const extrusion = paired.extrusionFile;
  paired.error = "";
  if (!trajectory || !extrusion) paired.error = t("pairedMissing");
  else if (!/\.csv$/i.test(trajectory.name) || !/\.gcode$/i.test(extrusion.name)) paired.error = t("pairedFileType");
  else if (trajectory.size > 20 * 1024 * 1024 || extrusion.size > 20 * 1024 * 1024) paired.error = t("pairedTooLarge");
  if (paired.error) { render(); return; }
  const requestId = ++paired.requestId;
  paired.checking = true;
  paired.result = null;
  render();
  try {
    const [trajectoryText, extrusionText] = await Promise.all([trajectory.text(), extrusion.text()]);
    if (state.gcode.importMode !== "paired" || requestId !== paired.requestId) return;
    const response = await fetchWithTimeout("/api/paired-timeline/parse", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        trajectory: { name: trajectory.name, text: trajectoryText },
        extrusion: { name: extrusion.name, text: extrusionText },
      }),
    }, 60000);
    const payload = await response.json().catch(() => ({}));
    if (state.gcode.importMode !== "paired" || requestId !== paired.requestId) return;
    if (!response.ok) throw new Error(payload.error || t("requestFailed"));
    paired.result = payload.analysis;
  } catch (error) {
    if (state.gcode.importMode !== "paired" || requestId !== paired.requestId) return;
    paired.error = error.message || t("requestFailed");
  } finally {
    if (state.gcode.importMode === "paired" && requestId === paired.requestId) {
      paired.checking = false;
      render();
    }
  }
}

function gcodeParserChecksPanel() {
  const checks = [
    ["route", t("parserCheck1Title"), t("parserCheck1Desc")],
    ["analytics", t("parserCheck2Title"), t("parserCheck2Desc")],
    ["policy", t("parserCheck3Title"), t("parserCheck3Desc")],
  ];
  return `
    <section class="visual-card gcode-checks-card">
      <div class="visual-card-header">
        <strong>${escapeHtml(t("parserChecks"))}</strong>
        <span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span>
      </div>
      <div class="gcode-check-list">
        ${checks.map(([icon, title, description]) => `
          <div class="gcode-check-item">
            <span class="stage-icon">${materialIcon(icon)}</span>
            <span><strong>${escapeHtml(title)}</strong><small>${escapeHtml(description)}</small></span>
          </div>`).join("")}
      </div>
      <div class="visual-card-caption">${materialIcon("shield_lock", "small")} ${escapeHtml(t("offlineByDesignDesc"))}</div>
    </section>`;
}

function issueMarkup(issue) {
  const severityClass = issue.severity === "error"
    ? "error"
    : issue.severity === "warning"
      ? "warning"
      : "info";
  const severityLabel = issue.severity === "error"
    ? t("issueError")
    : issue.severity === "warning"
      ? t("issueWarning")
      : t("issueInfo");
  const description = t(`issue_${issue.code}`, {
    command: issue.command || "",
  });
  const metadata = [];
  if (issue.line) metadata.push(t("lineLabel", { line: issue.line }));
  if (issue.occurrences > 1) {
    metadata.push(t("occurrences", { count: issue.occurrences }));
  }
  return `
    <div class="gcode-issue ${severityClass}">
      <span class="gcode-issue-icon">${materialIcon(issue.severity === "error" ? "error" : issue.severity === "warning" ? "warning" : "info")}</span>
      <span class="gcode-issue-copy">
        <span><strong>${escapeHtml(severityLabel)}</strong>${issue.command ? `<code>${escapeHtml(issue.command)}</code>` : ""}</span>
        <p>${escapeHtml(description)}</p>
        ${metadata.length ? `<small>${escapeHtml(metadata.join(" · "))}</small>` : ""}
      </span>
    </div>`;
}

function boundsTableMarkup(bounds) {
  return `
    <div class="bounds-table">
      <div class="bounds-row bounds-header"><span></span><strong>${escapeHtml(t("min"))}</strong><strong>${escapeHtml(t("max"))}</strong><strong>${escapeHtml(t("size"))}</strong></div>
      ${["x", "y", "z"].map((axis) => `
        <div class="bounds-row">
          <strong>${axis.toUpperCase()}</strong>
          <span>${escapeHtml(formatNumber(bounds.minimum[axis], 3))}</span>
          <span>${escapeHtml(formatNumber(bounds.maximum[axis], 3))}</span>
          <span>${escapeHtml(formatNumber(bounds.dimensions[axis], 3))} mm</span>
        </div>`).join("")}
    </div>`;
}

function renderGcodeResult() {
  const result = state.gcode.result;
  const summary = result.summary;
  const preview = result.preview;
  const preparationCommands = result.preparation_commands || [];
  const preparationCommandText = preparationCommands
    .slice(0, 4)
    .map((item) => `${item.command} · ${t("lineLabel", { line: item.line })}`)
    .join(" / ");
  const preparationCommandTitle = preparationCommands
    .slice(0, 20)
    .map((item) => `${item.command} · ${t("lineLabel", { line: item.line })}`)
    .join(" / ");
  const statusText = result.safe_to_plan ? t("parsedReady") : t("parsedAttention");
  const issuesMarkup = result.issues.length
    ? result.issues.map(issueMarkup).join("")
    : `<div class="gcode-no-issues">${materialIcon("check_circle")}<span>${escapeHtml(t("noParserIssues"))}</span></div>`;
  elements.mainContent.innerHTML = `
    <div class="gcode-result-page">
      <header class="gcode-file-header">
        <div>
          <div class="step-kicker">${escapeHtml(t("gcodeKicker"))}</div>
          <h1>${escapeHtml(result.file.name)}</h1>
          <p class="lead">${escapeHtml(t("gcodeLead"))}</p>
        </div>
        <button id="replaceGcodeButton" class="button secondary compact" type="button">${materialIcon("upload_file")} ${escapeHtml(t("replaceFile"))}</button>
        <input id="gcodeFileInput" type="file" accept=".gcode,.gc,.ngc,.nc,text/x-gcode" hidden>
      </header>

      <div class="gcode-file-status ${result.safe_to_plan ? "pass" : "warning"}">
        <span>${materialIcon(result.safe_to_plan ? "task_alt" : "warning")}</span>
        <span><strong>${escapeHtml(statusText)}</strong><small>${escapeHtml(formatFileSize(result.file.size_bytes))} · ${escapeHtml(formatNumber(result.file.line_count, 0))} ${escapeHtml(t("lineUnit"))}</small></span>
      </div>

      <section class="gcode-metric-grid" aria-label="${escapeHtml(t("analysisResults"))}">
        ${gcodeMetric("data_object", t("commands"), formatNumber(summary.command_count, 0))}
        ${gcodeMetric("route", t("moves"), formatNumber(summary.move_count, 0))}
        ${gcodeMetric("layers", t("layers"), formatNumber(summary.layer_count, 0))}
        ${gcodeMetric("schedule", t("estimatedTime"), formatDuration(summary.estimated_duration_s))}
        ${gcodeMetric(
          "straighten",
          t("pathLength"),
          summary.total_path_mm >= 1000
            ? formatNumber(summary.total_path_mm / 1000, 2)
            : formatNumber(summary.total_path_mm, 1),
          summary.total_path_mm >= 1000 ? " m" : " mm",
        )}
        ${gcodeMetric("water_drop", t("extrusion"), formatNumber(summary.extrusion_mm, 1), " mm")}
      </section>

      <div class="gcode-analysis-grid">
        <div class="gcode-analysis-main">
          <section class="visual-card gcode-preview-card">
            <div class="visual-card-header">
              <strong>${escapeHtml(t("toolpathPreview"))}</strong>
              <div class="gcode-preview-legend">
                <span><i class="print"></i>${escapeHtml(t("printMove"))}</span>
                <span><i class="travel"></i>${escapeHtml(t("travelMove"))}</span>
              </div>
            </div>
            <div class="gcode-canvas-stage">
              <canvas id="gcodePreviewCanvas" role="img" aria-label="${escapeHtml(t("toolpathPreview"))}"></canvas>
            </div>
            <div class="visual-card-caption">${materialIcon(preview.sampled ? "filter_alt" : "check_circle", "small")} ${escapeHtml(preview.sampled ? t("previewSampled") : t("previewComplete"))}</div>
          </section>

          <section id="gcodeFindings" class="section gcode-findings">
            <div class="section-title-row">
              <h2>${escapeHtml(t("parserIssues"))}</h2>
              <span class="status-badge ${summary.error_count ? "fail" : summary.warning_count ? "warning" : "pass"}">${summary.error_count ? `${summary.error_count} ${escapeHtml(t("issueError"))}` : summary.warning_count ? `${summary.warning_count} ${escapeHtml(t("issueWarning"))}` : escapeHtml(t("ready"))}</span>
            </div>
            <div class="gcode-issues">${issuesMarkup}</div>
          </section>
        </div>

        <aside class="gcode-analysis-side">
          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("analytics")} ${escapeHtml(t("fileSummary"))}</strong></div>
            <div class="gcode-stat-list">
              <div><span>${escapeHtml(t("printMoves"))}</span><strong>${escapeHtml(formatNumber(summary.print_move_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("travelMoves"))}</span><strong>${escapeHtml(formatNumber(summary.travel_move_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("retractions"))}</span><strong>${escapeHtml(formatNumber(summary.retraction_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("tools"))}</span><strong>${escapeHtml(formatNumber(summary.tool_count, 0))}</strong></div>
            </div>
          </section>

          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("deployed_code")} ${escapeHtml(t("xyzBounds"))}</strong><span class="inline-status">mm</span></div>
            ${boundsTableMarkup(result.bounds)}
          </section>

          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("terminal")} ${escapeHtml(t("topCommands"))}</strong></div>
            <div class="command-chip-list">
              ${result.top_commands.map((item) => `<span><code>${escapeHtml(item.command)}</code><strong>${escapeHtml(formatNumber(item.count, 0))}</strong></span>`).join("")}
            </div>
          </section>

          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("description")} ${escapeHtml(t("fileDetails"))}</strong></div>
            <div class="info-panel-body fact-list gcode-file-facts">
              <div class="fact-row"><span>${escapeHtml(t("fileSize"))}</span><code>${escapeHtml(formatFileSize(result.file.size_bytes))}</code><span></span></div>
              <div class="fact-row"><span>${escapeHtml(t("generator"))}</span><code title="${escapeHtml(result.file.generator || t("notReported"))}">${escapeHtml(result.file.generator || t("notReported"))}</code><span></span></div>
              ${preparationCommands.length ? `<div class="fact-row"><span title="${escapeHtml(t("preparationCommandNote"))}">${escapeHtml(t("preparationCommands"))}</span><code title="${escapeHtml(`${t("preparationCommandNote")} ${preparationCommandTitle}`)}">${escapeHtml(preparationCommandText)}${preparationCommands.length > 4 ? ` +${escapeHtml(String(preparationCommands.length - 4))}` : ""}</code><span></span></div>` : ""}
              <div class="fact-row"><span>${escapeHtml(t("checksum"))}</span><code title="${escapeHtml(result.file.sha256)}">${escapeHtml(result.file.sha256.slice(0, 12))}…</code><span></span></div>
            </div>
          </section>
        </aside>
      </div>
    </div>`;
  bindGcodeImport();
  window.requestAnimationFrame(drawGcodePreview);
}

function representativeTimelineEvents(events, limit = 9) {
  if (events.length <= limit) return events;
  const indexes = new Set();
  for (let index = 0; index < limit; index += 1) {
    indexes.add(Math.round(index * (events.length - 1) / (limit - 1)));
  }
  return [...indexes].map((index) => events[index]);
}

function formatEventDuration(seconds) {
  const value = Number(seconds || 0);
  if (value < 0.001) return "<0.001 s";
  if (value < 1) return `${formatNumber(value, 3)} s`;
  return formatDuration(value);
}

function planningGateMarkup(gate) {
  const passed = gate.status === "passed";
  const failed = gate.status === "failed";
  return `
    <div class="planning-gate ${passed ? "passed" : failed ? "failed" : "required"}">
      <span class="planning-gate-icon">${materialIcon(passed ? "check_circle" : failed ? "error" : "pending_actions")}</span>
      <span>
        <strong>${escapeHtml(t(`gate_${gate.code}_title`))}</strong>
        <small>${escapeHtml(t(`gate_${gate.code}_desc`))}</small>
      </span>
      <span class="status-badge ${passed ? "pass" : failed ? "fail" : "warning"}">${escapeHtml(t(passed ? "gatePassed" : failed ? "gateFailed" : "gateRequired"))}</span>
    </div>`;
}

function renderGcodePlanning() {
  const result = state.gcode.result;
  const plan = state.gcode.plan;
  if (!result) {
    state.gcode.view = "review";
    renderGcode();
    return;
  }

  if (state.gcode.planning) {
    elements.mainContent.innerHTML = `
      <div class="gcode-plan-page">
        <header class="gcode-plan-header">
          <div>${stepHeader(t("planKicker"), t("planTitle"), t("planLead"))}</div>
          <button id="backToGcodeReview" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToReview"))}</button>
        </header>
        <section class="plan-loading-card" aria-live="polite">
          <span class="gcode-upload-icon">${materialIcon("progress_activity")}</span>
          <h2>${escapeHtml(t("generatingPlan"))}</h2>
          <p>${escapeHtml(t("generatingPlanDesc"))}</p>
          <div class="parse-progress" aria-hidden="true"><span></span></div>
        </section>
      </div>`;
    document.getElementById("backToGcodeReview")?.addEventListener("click", () => {
      selectRunbook("gcode");
    });
    return;
  }

  if (!plan) {
    elements.mainContent.innerHTML = `
      <div class="gcode-plan-page">
        <header class="gcode-plan-header">
          <div>${stepHeader(t("planKicker"), t("planTitle"), t("planLead"))}</div>
          <button id="backToGcodeReview" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToReview"))}</button>
        </header>
        <div class="callout danger">${materialIcon("error")}<span><strong>${escapeHtml(t("planFailed"))}</strong>${escapeHtml(state.gcode.planError || t("requestFailed"))}</span></div>
        <button id="retryGcodePlan" class="button primary" type="button">${materialIcon("refresh")} ${escapeHtml(t("regeneratePlan"))}</button>
      </div>`;
    document.getElementById("backToGcodeReview")?.addEventListener("click", () => {
      selectRunbook("gcode");
    });
    document.getElementById("retryGcodePlan")?.addEventListener("click", () => {
      openGcodePlanning(true);
    });
    return;
  }

  const ur = plan.ur_robot;
  const klipper = plan.klipper;
  const synchronization = plan.synchronization;
  const timeline = synchronization.timeline;
  const representativeEvents = representativeTimelineEvents(timeline.events);
  const pathValue = ur.path_mm >= 1000
    ? `${formatNumber(ur.path_mm / 1000, 2)} m`
    : `${formatNumber(ur.path_mm, 1)} mm`;
  const preparationMarkup = klipper.preparation_commands.length
    ? `
      <div class="channel-preparation">
        <span>${materialIcon("home", "small")} ${escapeHtml(t("preparationCommands"))}</span>
        <div>${klipper.preparation_commands.map((item) => `<code>${escapeHtml(item.command)} · ${escapeHtml(t("lineLabel", { line: item.line }))}</code>`).join("")}</div>
      </div>`
    : "";

  elements.mainContent.innerHTML = `
    <div class="gcode-plan-page">
      <header class="gcode-plan-header">
        <div>
          <div class="step-kicker">${escapeHtml(t("planKicker"))}</div>
          <h1>${escapeHtml(result.file.name)}</h1>
          <p class="lead">${escapeHtml(t("planLead"))}</p>
        </div>
        <div class="plan-header-actions">
          <button id="backToGcodeReview" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToReview"))}</button>
          <button id="regenerateGcodePlan" class="button secondary compact" type="button">${materialIcon("refresh")} ${escapeHtml(t("regeneratePlan"))}</button>
          <button id="openPathReviewFromPlan" class="button primary compact" type="button">${materialIcon("view_in_ar")} ${escapeHtml(t("pathReviewNext"))}</button>
        </div>
      </header>

      <div class="plan-lock-banner">
        <span>${materialIcon("lock")}</span>
        <span><strong>${escapeHtml(t("draftPlan"))}</strong><small>${escapeHtml(t("draftPlanDesc"))}</small></span>
      </div>

      <section class="gcode-metric-grid planning-metrics" aria-label="${escapeHtml(t("synchronizationOverview"))}">
        ${gcodeMetric("precision_manufacturing", t("waypoints"), formatNumber(ur.waypoint_count, 0))}
        ${gcodeMetric("route", t("totalPath"), pathValue)}
        ${gcodeMetric("water_drop", t("extrusionMoves"), formatNumber(klipper.extrusion_move_count, 0))}
        ${gcodeMetric("schedule", t("nominalGcodeTime"), formatDuration(ur.estimated_duration_s))}
      </section>

      <section class="section planning-channel-section">
        <div class="section-title-row">
          <h2>${escapeHtml(t("controlResponsibility"))}</h2>
          <span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span>
        </div>
        <div class="planning-channel-grid">
          <article class="planning-channel-card ur-channel">
            <header>
              <span class="planning-channel-icon">${materialIcon("precision_manufacturing")}</span>
              <span><strong>${escapeHtml(t("urRobotChannel"))}</strong><small>${escapeHtml(t("urChannelBadge"))}</small></span>
            </header>
            <p>${escapeHtml(t("urChannelRole"))}</p>
            <div class="channel-controller">${materialIcon("account_tree", "small")} ${escapeHtml(t("urChannelController"))}</div>
            <div class="channel-responsibility"><code>X</code><code>Y</code><code>Z</code></div>
            <div class="channel-stats">
              <div><span>${escapeHtml(t("waypoints"))}</span><strong>${escapeHtml(formatNumber(ur.waypoint_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("totalPath"))}</span><strong>${escapeHtml(pathValue)}</strong></div>
            </div>
          </article>

          <article class="planning-channel-card klipper-channel">
            <header>
              <span class="planning-channel-icon">${materialIcon("water_drop")}</span>
              <span><strong>${escapeHtml(t("klipperChannel"))}</strong><small>${escapeHtml(t("klipperChannelBadge"))}</small></span>
            </header>
            <p>${escapeHtml(t("klipperChannelRole"))}</p>
            <div class="channel-controller">${materialIcon("account_tree", "small")} ${escapeHtml(t("klipperChannelController"))}</div>
            <div class="channel-responsibility"><code>E</code><code>M104</code><code>M106</code><code>G28</code></div>
            <div class="channel-stats channel-stats-three">
              <div><span>${escapeHtml(t("extrusionMoves"))}</span><strong>${escapeHtml(formatNumber(klipper.extrusion_move_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("extrusion"))}</span><strong>${escapeHtml(formatNumber(klipper.extrusion_mm, 1))} mm</strong></div>
              <div><span>${escapeHtml(t("retractions"))}</span><strong>${escapeHtml(formatNumber(klipper.retraction_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("heaterCommands"))}</span><strong>${escapeHtml(formatNumber(klipper.heater_command_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("fanCommands"))}</span><strong>${escapeHtml(formatNumber(klipper.fan_command_count, 0))}</strong></div>
              <div><span>${escapeHtml(t("machineSettings"))}</span><strong>${escapeHtml(formatNumber(klipper.machine_setting_command_count, 0))}</strong></div>
            </div>
            ${preparationMarkup}
          </article>
        </div>
      </section>

      <div class="planning-detail-grid">
        <div class="planning-detail-main">
          <section class="visual-card planning-timeline-card">
            <div class="visual-card-header">
              <span><strong>${escapeHtml(t("channelTimeline"))}</strong><small>${escapeHtml(t("timelineLead"))}</small></span>
              <div class="planning-timeline-summary">
                <span><strong>${escapeHtml(formatNumber(synchronization.combined_motion_count, 0))}</strong>${escapeHtml(t("combinedMoves"))}</span>
                <span><strong>${escapeHtml(formatNumber(synchronization.ur_only_motion_count, 0))}</strong>${escapeHtml(t("urOnlyMoves"))}</span>
                <span><strong>${escapeHtml(formatNumber(synchronization.klipper_only_motion_count, 0))}</strong>${escapeHtml(t("klipperOnlyMoves"))}</span>
              </div>
            </div>
            <div class="planning-timeline-visual">
              <div class="planning-timeline-labels">
                <strong class="ur-label">${escapeHtml(t("urRobotChannel"))}</strong>
                <strong class="klipper-label">${escapeHtml(t("klipperChannel"))}</strong>
              </div>
              <canvas id="planningTimelineCanvas" role="img" aria-label="${escapeHtml(t("channelTimeline"))}"></canvas>
            </div>
            <div class="planning-timeline-legend">
              <span><i class="ur-print"></i>${escapeHtml(t("action_print_path"))}</span>
              <span><i class="ur-travel"></i>${escapeHtml(t("action_travel"))}</span>
              <span><i class="klipper-extrude"></i>${escapeHtml(t("action_extrude"))}</span>
              <span><i class="klipper-retract"></i>${escapeHtml(t("action_retract"))}</span>
            </div>
            <div class="visual-card-caption">${materialIcon(timeline.sampled ? "filter_alt" : "check_circle", "small")} ${escapeHtml(t(timeline.sampled ? "timelineSampled" : "timelineComplete"))}</div>
          </section>

          <section class="info-panel planning-events-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("format_list_numbered")} ${escapeHtml(t("representativeEvents"))}</strong></div>
            <div class="planning-event-table-wrap">
              <table class="planning-event-table">
                <thead><tr><th>${escapeHtml(t("sourceLine"))}</th><th>${escapeHtml(t("urAction"))}</th><th>${escapeHtml(t("klipperAction"))}</th><th>${escapeHtml(t("eventDuration"))}</th></tr></thead>
                <tbody>
                  ${representativeEvents.map((event) => `
                    <tr>
                      <td><code>${escapeHtml(t("lineLabel", { line: event.line }))}</code></td>
                      <td><span class="action-pill ur ${escapeHtml(event.ur_action)}">${escapeHtml(t(`action_${event.ur_action}`))}</span></td>
                      <td><span class="action-pill klipper ${escapeHtml(event.klipper_action)}">${escapeHtml(t(`action_${event.klipper_action}`))}</span></td>
                      <td>${escapeHtml(formatEventDuration(event.duration_s))}</td>
                    </tr>`).join("")}
                </tbody>
              </table>
            </div>
          </section>
        </div>

        <aside class="planning-detail-side">
          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("deployed_code")} ${escapeHtml(t("coordinateFrame"))}</strong><span class="status-badge neutral">${escapeHtml(t("automaticPlacement"))}</span></div>
            <div class="info-panel-body fact-list">
              <div class="fact-row"><span>${escapeHtml(t("sourceFrame"))}</span><code>${escapeHtml(plan.coordinate_frame.source)}</code><span></span></div>
              <div class="fact-row"><span>${escapeHtml(t("targetFrame"))}</span><code>${escapeHtml(plan.coordinate_frame.target)}</code><span></span></div>
              <div class="fact-row"><span>${escapeHtml(t("scale"))}</span><code>${escapeHtml(plan.coordinate_frame.scale)}</code><span></span></div>
            </div>
          </section>
          <section class="info-panel planning-gates-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("verified_user")} ${escapeHtml(t("planningGates"))}</strong></div>
            <div class="planning-gates">${plan.gates.map(planningGateMarkup).join("")}</div>
          </section>
        </aside>
      </div>
    </div>`;

  document.getElementById("backToGcodeReview")?.addEventListener("click", () => {
    selectRunbook("gcode");
  });
  document.getElementById("regenerateGcodePlan")?.addEventListener("click", () => {
    openGcodePlanning(true);
  });
  document.getElementById("openPathReviewFromPlan")?.addEventListener("click", () => {
    state.moduleSteps.motion = 1;
    state.gcode.view = "path-review";
    openGcodePathReview();
  });
  window.requestAnimationFrame(drawPlanningTimeline);
}

async function openGcodePlanning(force = false) {
  if (
    state.gcode.planning
    || !state.gcode.result
    || !state.gcode.result.safe_to_plan
  ) return;
  if (!state.gcode.sourceFile) {
    notify(t("planSourceUnavailable"), "error");
    return;
  }
  const sourceFile = state.gcode.sourceFile;
  const importRequestId = state.gcode.importRequestId;
  const requestIsCurrent = () => state.gcode.importMode === "gcode"
    && state.gcode.sourceFile === sourceFile
    && state.gcode.importRequestId === importRequestId;
  state.gcode.view = "planning";
  if (state.gcode.plan && !force) {
    render();
    return;
  }
  state.gcode.planning = true;
  state.gcode.planError = "";
  render();
  try {
    const response = await fetchWithTimeout("/api/gcode/plan", {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
        "X-Gcode-Filename": encodeURIComponent(sourceFile.name),
      },
      body: sourceFile,
    });
    const payload = await response.json().catch(() => ({}));
    if (!requestIsCurrent()) return;
    if (!response.ok) {
      throw new Error(payload.error || `${t("planFailed")} (${response.status})`);
    }
    state.gcode.plan = payload.plan;
    notify(t("draftPlanReady"), "success");
  } catch (error) {
    if (!requestIsCurrent()) return;
    state.gcode.plan = null;
    state.gcode.planError = error.message || t("planFailed");
    notify(state.gcode.planError, "error");
  } finally {
    if (requestIsCurrent()) {
      state.gcode.planning = false;
      render();
    }
  }
}

function destroyPathReviewViewer() {
  state.gcode.pathReviewViewer?.destroy?.();
  state.gcode.pathReviewViewer = null;
}

function stopPathReviewPlayback() {
  state.gcode.pathReviewPlaying = false;
  state.gcode.pathReviewLastFrame = 0;
  if (state.gcode.pathReviewAnimationFrame) {
    window.cancelAnimationFrame(state.gcode.pathReviewAnimationFrame);
    state.gcode.pathReviewAnimationFrame = null;
  }
}

function pathReviewGateMarkup() {
  const moveit = state.gcode.pathReviewMoveitResult;
  const gates = (state.gcode.pathReview?.gates || []).map((gate) => {
    if (gate.code === "collision_check") {
      return {
        ...gate,
        status: moveit?.collision_free === true
          ? "passed"
          : moveit?.status === "failed" || moveit?.collision_free === false
            ? "failed"
            : "required",
      };
    }
    if (gate.code !== "moveit_plan") return gate;
    return {
      ...gate,
      status: moveit?.status === "passed"
        ? "passed"
        : moveit?.status === "failed"
          ? "failed"
          : "required",
    };
  });
  return gates.map(planningGateMarkup).join("");
}

function pathReviewStatusMarkup() {
  const moveit = state.gcode.pathReviewMoveitResult;
  const simulation = state.gcode.pathReviewSimulationResult;
  const passed = moveit?.status === "passed";
  const failed = moveit?.status === "failed";
  const estimated = !passed && hasRobotMotionTrajectory(simulation);
  const routeOnly = !passed && !estimated && !failed && hasReviewRoute();
  const simulated = Boolean(moveit?.simulation);
  const cached = Boolean(moveit?.cached);
  const title = passed
    ? cached
      ? t("cachedMoveitPassed")
      : simulated
      ? t("pathReviewDemoPassed")
      : t("pathReviewMoveitPassed")
    : estimated
      ? t("offlineSimulationReady")
    : failed
      ? t("pathReviewMoveitFailed")
      : t("pathReviewOffline");
  const description = passed
    ? cached
      ? t("cachedMoveitPassedDesc")
      : simulated
      ? t("pathReviewDemoPassedDesc")
      : t("pathReviewMoveitPassedDesc")
    : estimated
      ? t("offlineSimulationReadyDesc")
    : failed
      ? t("pathReviewMoveitFailedDesc")
      : t("pathReviewOfflineDesc");
  return `
    <div class="path-review-status ${passed ? "passed" : estimated ? "offline" : failed ? "failed" : "offline"}">
      <span>${materialIcon(passed ? "verified" : estimated ? "motion_photos_on" : failed ? "error" : "visibility")}</span>
      <span><strong>${escapeHtml(title)}</strong><small>${escapeHtml(description)}</small></span>
      <span class="status-badge ${passed ? "pass" : estimated ? "warning" : failed ? "fail" : routeOnly ? "neutral" : "warning"}">${escapeHtml(passed ? t("gatePassed") : estimated ? t("simulationOnly") : failed ? t("gateFailed") : routeOnly ? t("readOnly") : t("gateRequired"))}</span>
    </div>`;
}

function pathReviewSegmentMarkup(segment, index = 0, count = 0) {
  if (!segment) {
    return `<div class="path-segment-empty">${escapeHtml(t("pathReviewLoading"))}</div>`;
  }
  const kindLabel = t(
    segment.kind === "print" ? "action_print_path" : "action_travel",
  );
  return `
    <div class="path-segment-facts">
      <div><span>${escapeHtml(t("segmentNumber"))}</span><strong>${escapeHtml(formatNumber(index + 1, 0))} / ${escapeHtml(formatNumber(count, 0))}</strong></div>
      <div><span>${escapeHtml(t("sourceLine"))}</span><strong>${escapeHtml(formatNumber(segment.line, 0))}</strong></div>
      <div><span>${escapeHtml(t("layerNumber"))}</span><strong>${escapeHtml(formatNumber(segment.layer + 1, 0))}</strong></div>
      <div><span>${escapeHtml(t("pathType"))}</span><strong class="${segment.kind === "print" ? "pass-text" : ""}">${escapeHtml(kindLabel)}</strong></div>
      <div class="wide"><span>${escapeHtml(t("urPosition"))}</span><code>X ${escapeHtml(formatNumber(segment.x1, 4))} · Y ${escapeHtml(formatNumber(segment.y1, 4))} · Z ${escapeHtml(formatNumber(segment.z1, 4))} m</code></div>
      <div class="wide"><span>${escapeHtml(t("klipperDelta"))}</span><code>${escapeHtml(formatNumber(segment.klipper_units, 5))} mm · E ${escapeHtml(formatNumber(segment.extrusion_delta_mm, 5))} mm</code></div>
    </div>`;
}

function updatePathReviewProgress(details) {
  if (!details) return;
  state.gcode.pathReviewProgress = Number(details.progress) || 0;
  const slider = document.getElementById("pathReviewProgress");
  if (slider) slider.value = String(Math.round(state.gcode.pathReviewProgress * 1000));
  const progressLabel = document.getElementById("pathReviewProgressLabel");
  if (progressLabel) {
    progressLabel.textContent = `${formatNumber(state.gcode.pathReviewProgress * 100, 1)}%`;
  }
  const detail = document.getElementById("pathReviewSegmentDetail");
  if (detail) {
    detail.innerHTML = pathReviewSegmentMarkup(
      details.segment,
      details.index,
      details.count,
    );
  }
}

function isRobotMotionTrajectory(result) {
  return Boolean(
    result?.status === "passed"
    && Array.isArray(result?.joint_names)
    && result.joint_names.length === 6
    && Array.isArray(result?.trajectory)
    && result.trajectory.length > 1
    && result.trajectory.every(
      (point) => Array.isArray(point?.positions) && point.positions.length === 6,
    ),
  );
}

function activeRobotMotionTrajectory() {
  if (isRobotMotionTrajectory(state.gcode.pathReviewMoveitResult)) {
    return state.gcode.pathReviewMoveitResult;
  }
  if (isRobotMotionTrajectory(state.gcode.pathReviewSimulationResult)) {
    return state.gcode.pathReviewSimulationResult;
  }
  return null;
}

function hasRobotMotionTrajectory(result = activeRobotMotionTrajectory()) {
  return isRobotMotionTrajectory(result);
}

function hasReviewRoute() {
  return Boolean(state.gcode.pathReview?.path?.segments?.length);
}

function hasValidatedMoveItTrajectory() {
  const result = state.gcode.pathReviewMoveitResult;
  return Boolean(
    isRobotMotionTrajectory(result) && !result?.offline_estimate
    && result?.planning_policy_version === PLANNING_POLICY_VERSION
    && result?.interpolation_validation === "passed",
  );
}

function moveitValidationAvailable() {
  return Boolean(isOnlineMode() && (state.demo || state.rosStatus?.moveit_ready));
}

function updatePathReviewPlaybackButton() {
  const button = document.getElementById("pathReviewPlayButton");
  if (!button) return;
  const routeReady = hasReviewRoute();
  const label = state.gcode.pathReviewPlaying
    ? "pausePreview"
    : hasRobotMotionTrajectory() ? "playRobotMotion" : "playPreview";
  button.disabled = !routeReady;
  button.innerHTML = `${materialIcon(state.gcode.pathReviewPlaying ? "pause" : "play_arrow")} ${escapeHtml(t(label))}`;
}

function pathReviewPlaybackFrame(timestamp) {
  if (!state.gcode.pathReviewPlaying || state.gcode.view !== "path-review") {
    stopPathReviewPlayback();
    updatePathReviewPlaybackButton();
    return;
  }
  if (!state.gcode.pathReviewLastFrame) state.gcode.pathReviewLastFrame = timestamp;
  const elapsed = Math.min(250, timestamp - state.gcode.pathReviewLastFrame);
  state.gcode.pathReviewLastFrame = timestamp;
  const previewDurationMs = 30000 / Math.max(0.25, state.gcode.pathReviewPlaybackSpeed);
  const next = Math.min(1, state.gcode.pathReviewProgress + elapsed / previewDurationMs);
  state.gcode.pathReviewViewer?.setProgress(next);
  if (next >= 1) {
    stopPathReviewPlayback();
    updatePathReviewPlaybackButton();
    return;
  }
  state.gcode.pathReviewAnimationFrame = window.requestAnimationFrame(
    pathReviewPlaybackFrame,
  );
}

function togglePathReviewPlayback() {
  if (!hasReviewRoute()) {
    stopPathReviewPlayback();
    updatePathReviewPlaybackButton();
    return;
  }
  if (state.gcode.pathReviewPlaying) {
    stopPathReviewPlayback();
    updatePathReviewPlaybackButton();
    return;
  }
  if (state.gcode.pathReviewProgress >= 1) {
    state.gcode.pathReviewProgress = 0;
    state.gcode.pathReviewViewer?.setProgress(0);
  }
  state.gcode.pathReviewPlaying = true;
  state.gcode.pathReviewLastFrame = 0;
  updatePathReviewPlaybackButton();
  state.gcode.pathReviewAnimationFrame = window.requestAnimationFrame(
    pathReviewPlaybackFrame,
  );
}

function mountPathReviewViewer() {
  const container = document.getElementById("pathReviewViewport");
  if (
    !container
    || !state.gcode.pathReview
    || state.gcode.view !== "path-review"
  ) return;
  const mount = () => {
    if (
      state.gcode.view !== "path-review"
      || !document.getElementById("pathReviewViewport")
      || !window.URPathReview3D
    ) return;
    destroyPathReviewViewer();
    state.gcode.pathReviewViewer = window.URPathReview3D.mount(
      document.getElementById("pathReviewViewport"),
      state.gcode.pathReview,
      {
        onProgress: updatePathReviewProgress,
        onSelectSegment: (segment) => {
          notify(t("lineLabel", { line: segment.line }));
        },
        onModelError: () => {
          if (!state.gcode.pathReviewModelWarningShown) {
            state.gcode.pathReviewModelWarningShown = true;
            notify(t("modelLoadWarning"), "error");
          }
        },
      },
    );
    state.gcode.pathReviewViewer.setMoveItResult(
      activeRobotMotionTrajectory(),
    );
    state.gcode.pathReviewViewer.setLayer(state.gcode.pathReviewLayer);
    state.gcode.pathReviewViewer.setProgress(state.gcode.pathReviewProgress);
  };
  if (window.URPathReview3D) {
    mount();
  } else {
    window.addEventListener("ur-path-review-3d-ready", mount, { once: true });
  }
}

function bindPathReviewControls() {
  document.getElementById("backToMotionPlanning")?.addEventListener("click", () => {
    stopPathReviewPlayback();
    destroyPathReviewViewer();
    state.moduleSteps.motion = 0;
    state.gcode.view = "planning";
    render();
  });
  document.getElementById("retryPathReview")?.addEventListener("click", () => {
    openGcodePathReview(true, state.gcode.pathReviewRequestedSimulation);
  });
  document.getElementById("pathReviewPlayButton")?.addEventListener(
    "click",
    togglePathReviewPlayback,
  );
  document.getElementById("pathReviewRestartButton")?.addEventListener("click", () => {
    if (!hasReviewRoute()) return;
    stopPathReviewPlayback();
    state.gcode.pathReviewProgress = 0;
    state.gcode.pathReviewViewer?.setProgress(0);
    updatePathReviewPlaybackButton();
  });
  document.getElementById("pathReviewProgress")?.addEventListener("input", (event) => {
    if (!hasReviewRoute()) return;
    stopPathReviewPlayback();
    state.gcode.pathReviewViewer?.setProgress(Number(event.target.value) / 1000);
    updatePathReviewPlaybackButton();
  });
  document.getElementById("pathReviewSpeed")?.addEventListener("change", (event) => {
    state.gcode.pathReviewPlaybackSpeed = Number(event.target.value) || 1;
  });
  document.getElementById("pathReviewLayer")?.addEventListener("change", (event) => {
    state.gcode.pathReviewLayer = event.target.value;
    state.gcode.pathReviewViewer?.setLayer(event.target.value);
    state.gcode.pathReviewViewer?.setProgress(state.gcode.pathReviewProgress);
  });
  document.querySelectorAll("[data-path-camera]").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll("[data-path-camera]").forEach((item) => {
        item.classList.toggle("selected", item === button);
      });
      state.gcode.pathReviewViewer?.setCameraPreset(button.dataset.pathCamera);
    });
  });
  document.getElementById("validatePathWithMoveit")?.addEventListener("click", () => {
    validatePathWithMoveIt();
  });
  document.getElementById("openConnectionFromMoveit")?.addEventListener("click", () => {
    stopPathReviewPlayback();
    selectRunbook("connection");
  });
  document.getElementById("switchOnlineFromMoveit")?.addEventListener("click", () => {
    changeOperatingMode("online");
  });
}

function moveitResultMetrics(result) {
  if (!result) return "";
  const approach = result.approach_status;
  return `
    <div class="moveit-result-metrics">
      ${approach && approach !== "not_run" ? `<div><span>${escapeHtml(t("approachToPathStart"))}</span><strong class="${approach === "passed" ? "pass-text" : "fail-text"}">${escapeHtml(t(approach === "passed" ? "approachPassed" : "approachFailed"))}</strong></div>` : ""}
      ${Number.isFinite(Number(result.approach_distance_m)) ? `<div><span>${escapeHtml(t("approachDistance"))}</span><strong>${escapeHtml(formatNumber(result.approach_distance_m, 3))} m</strong></div>` : ""}
      <div><span>${escapeHtml(t("cartesianFraction"))}</span><strong>${escapeHtml(formatNumber(Number(result.cartesian_fraction) * 100, 1))}%</strong></div>
      <div><span>${escapeHtml(t("plannedJointPoints"))}</span><strong>${escapeHtml(formatNumber(result.planned_point_count, 0))}</strong></div>
      <div><span>${escapeHtml(t("maxJointStep"))}</span><strong>${escapeHtml(formatNumber(result.max_joint_step_rad, 4))} rad</strong></div>
      <div><span>${escapeHtml(t("planningScene"))}</span><strong>${escapeHtml(t(result.simulation ? "simulatedScene" : "activeMoveitScene"))}</strong></div>
    </div>`;
}

function renderGcodePathReview() {
  destroyPathReviewViewer();
  const review = state.gcode.pathReview;
  const result = state.gcode.result;
  if (!result || !state.gcode.plan) {
    state.gcode.view = "review";
    renderGcode();
    return;
  }

  if (state.gcode.pathReviewLoading) {
    const reviewJob = state.gcode.pathReviewJob;
    const reviewProgress = Number(reviewJob?.progress);
    const hasReviewProgress = Number.isFinite(reviewProgress);
    const boundedReviewProgress = hasReviewProgress
      ? Math.max(0, Math.min(100, Math.round(reviewProgress)))
      : 0;
    const reviewLoadingTitle = reviewJob?.phase === "queued"
      ? t("pathReviewQueued")
      : reviewJob
        ? t("pathReviewBuilding")
        : t("pathReviewLoading");
    elements.mainContent.innerHTML = `
      <div class="gcode-plan-page path-review-page">
        <header class="gcode-plan-header">
          <div>${stepHeader(t("pathReviewKicker"), t("pathReviewTitle"), t("pathReviewLead"))}</div>
          <button id="backToMotionPlanning" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToPlanning"))}</button>
        </header>
        <section class="plan-loading-card" aria-live="polite">
          <span class="gcode-upload-icon">${materialIcon("view_in_ar")}</span>
          <h2>${escapeHtml(reviewLoadingTitle)}</h2>
          <p>${escapeHtml(t("pathReviewLoadingDesc"))}</p>
          ${hasReviewProgress
            ? `<progress class="execution-progress-track" value="${escapeHtml(String(boundedReviewProgress))}" max="100" aria-label="${escapeHtml(t("pathReviewGenerationProgress", { progress: boundedReviewProgress }))}"></progress><small>${escapeHtml(t("pathReviewGenerationProgress", { progress: boundedReviewProgress }))}</small>`
            : `<div class="parse-progress" aria-hidden="true"><span></span></div>`}
        </section>
      </div>`;
    bindPathReviewControls();
    return;
  }

  if (!review || state.gcode.pathReviewError) {
    elements.mainContent.innerHTML = `
      <div class="gcode-plan-page path-review-page">
        <header class="gcode-plan-header">
          <div>${stepHeader(t("pathReviewKicker"), t("pathReviewTitle"), t("pathReviewLead"))}</div>
          <button id="backToMotionPlanning" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToPlanning"))}</button>
        </header>
        <div class="callout danger">${materialIcon("error")}<span><strong>${escapeHtml(t("pathReviewLoadFailed"))}</strong>${escapeHtml(state.gcode.pathReviewError || t("requestFailed"))}</span></div>
        <button id="retryPathReview" class="button primary" type="button">${materialIcon("refresh")} ${escapeHtml(t("retryPathReview"))}</button>
      </div>`;
    bindPathReviewControls();
    return;
  }

  const tableSurfaceZ = Number(
    review.workcell_model?.tabletop?.surface_z_m
      ?? review.print_bed_m?.plane_z,
  );
  const tableHeightMm = Number.isFinite(tableSurfaceZ)
    ? formatNumber(tableSurfaceZ * 1000, 1)
    : "—";
  const moveit = state.gcode.pathReviewMoveitResult;
  const moveitDuration = Number(moveit?.duration_s);
  const reviewedDurationReady = moveit?.status === "passed"
    && Number.isFinite(moveitDuration);
  const simulationOnly = review.simulation_only === true;
  const simulationGateTitle = t(
    review.physical_calibration_complete
      ? "simulationDraftUnsavedTitle"
      : "simulationMoveitBlockedTitle",
  );
  const simulationGateDescription = t(
    review.physical_calibration_complete
      ? "simulationDraftUnsavedDesc"
      : "simulationMoveitBlockedDesc",
  );
  const moveitAvailable = moveitValidationAvailable() && !simulationOnly;
  const moveitError = state.gcode.pathReviewMoveitError;
  const layerOptions = review.layers.map((layer) => `
    <option value="${escapeHtml(String(layer.index))}" ${String(state.gcode.pathReviewLayer) === String(layer.index) ? "selected" : ""}>${escapeHtml(formatNumber(layer.index + 1, 0))} · Z ${escapeHtml(formatNumber(layer.z_mm, 3))} mm</option>
  `).join("");
  const initialSegment = review.path.segments[
    Math.min(
      review.path.segments.length - 1,
      Math.floor(state.gcode.pathReviewProgress * Math.max(0, review.path.segments.length - 1)),
    )
  ];
  const moveitButtonDisabled = state.gcode.pathReviewMoveitRunning
    || !state.gcode.sourceFile
    || !moveitAvailable;
  const playbackTrajectory = activeRobotMotionTrajectory();
  const routeReady = hasReviewRoute();
  const robotMotionReady = hasRobotMotionTrajectory(playbackTrajectory);
  const robotMotionEstimated = Boolean(playbackTrajectory?.offline_estimate);
  const moveitCardFailed = moveit?.status === "failed" || Boolean(moveitError);
  const moveitCardTitle = moveit?.status === "passed"
    ? moveit.cached
      ? t("cachedMoveitPassed")
      : moveit.simulation
      ? t("pathReviewDemoPassed")
      : t("pathReviewMoveitPassed")
    : moveit?.status === "failed"
      ? t("pathReviewMoveitFailed")
      : moveitError
        ? t("moveitValidationErrorTitle")
        : simulationOnly
          ? simulationGateTitle
        : !moveitAvailable
          ? t(isOnlineMode() ? "moveitUnavailableTitle" : "offlineMoveitTitle")
          : t("moveitValidationRequired");
  const moveitCardDescription = (moveit?.cached ? t("cachedMoveitPassedDesc") : moveit?.message)
    || moveitError
    || (simulationOnly
      ? simulationGateDescription
      : !moveitAvailable
      ? t(isOnlineMode() ? "moveitUnavailableDesc" : "offlineMoveitDesc")
      : t("moveitValidationRequiredDesc"));

  elements.mainContent.innerHTML = `
    <div class="gcode-plan-page path-review-page">
      <header class="gcode-plan-header">
        <div>
          <div class="step-kicker">${escapeHtml(t("pathReviewKicker"))}</div>
          <h1>${escapeHtml(result.file.name)}</h1>
          <p class="lead">${escapeHtml(t("pathReviewLead"))}</p>
        </div>
        <div class="plan-header-actions">
          <button id="backToMotionPlanning" class="button secondary compact" type="button">${materialIcon("arrow_back")} ${escapeHtml(t("backToPlanning"))}</button>
        </div>
      </header>

      ${pathReviewStatusMarkup()}

      ${simulationOnly ? `<div class="callout warning">${materialIcon("lock")}<span><strong>${escapeHtml(simulationGateTitle)}</strong>${escapeHtml(simulationGateDescription)}</span></div>` : ""}
      ${review.tabletop_height_candidate ? `<div class="callout warning">${materialIcon("height")}<span>${escapeHtml(t("heightCandidateReview"))}</span></div>` : ""}
      ${review.print_z_candidate ? `<div class="callout warning">${materialIcon("height")}<span>${escapeHtml(t("printZeroReview"))}</span></div>` : ""}


      <section class="path-review-metrics" aria-label="${escapeHtml(t("pathReviewSummary"))}">
        ${gcodeMetric("route", t("displayedSegments"), formatNumber(review.path.returned_segment_count, 0))}
        ${gcodeMetric("layers", t("layers"), formatNumber(review.layers.length, 0))}
        ${gcodeMetric("schedule", t(reviewedDurationReady ? "reviewedMoveitTime" : "nominalGcodeTime"), formatDuration(reviewedDurationReady ? moveitDuration : state.gcode.plan.ur_robot.estimated_duration_s))}
        ${gcodeMetric("water_drop", t("extrusionMoves"), formatNumber(state.gcode.plan.klipper.extrusion_move_count, 0))}
      </section>

      <div class="path-review-layout">
        <section class="path-viewport-card">
          <div class="path-viewport-header">
            <span><strong>${escapeHtml(t("pathViewport"))}</strong><small>${escapeHtml(t("pathViewportHelp"))}</small></span>
            <div class="path-camera-controls" role="group" aria-label="${escapeHtml(t("pathViewport"))}">
              <button class="icon-button selected" type="button" data-path-camera="iso" title="${escapeHtml(t("cameraIso"))}">${materialIcon("view_in_ar")}</button>
              <button class="icon-button" type="button" data-path-camera="top" title="${escapeHtml(t("cameraTop"))}">${materialIcon("vertical_align_top")}</button>
              <button class="icon-button" type="button" data-path-camera="front" title="${escapeHtml(t("cameraFront"))}">${materialIcon("crop_landscape")}</button>
              <button class="icon-button" type="button" data-path-camera="side" title="${escapeHtml(t("cameraSide"))}">${materialIcon("side_navigation")}</button>
            </div>
          </div>
          <div id="pathReviewViewport" class="path-review-viewport" aria-label="${escapeHtml(t("pathViewport"))}"></div>
          <div class="path-legend">
            <strong>${escapeHtml(t("pathLegend"))}</strong>
            <span class="robot-model-key">${materialIcon("precision_manufacturing")}${escapeHtml(t("urRobotModel"))}</span>
            <span class="table-model-key">${materialIcon("table_restaurant")}${escapeHtml(t("printTableModel", { height: tableHeightMm }))}</span>
            <span><i class="path-swatch print"></i>${escapeHtml(t("urPrintPath"))}</span>
            <span><i class="path-swatch travel"></i>${escapeHtml(t("urTravelPath"))}</span>
            <span><i class="path-swatch extrusion"></i>${escapeHtml(t("klipperExtrusionEvents"))}</span>
            <span><i class="path-swatch retraction"></i>${escapeHtml(t("klipperRetractionEvents"))}</span>
          </div>
          <div class="path-motion-state ${robotMotionReady ? "ready" : "waiting"}">
            ${materialIcon(robotMotionReady ? "motion_photos_on" : "route")}
            <span>
              <strong>${escapeHtml(t(robotMotionReady ? (robotMotionEstimated ? "robotMotionEstimated" : "robotMotionReady") : "robotMotionWaiting"))}</strong>
              <small>${escapeHtml(t(robotMotionReady ? (robotMotionEstimated ? "robotMotionEstimatedDesc" : "robotMotionReadyDesc") : "robotMotionWaitingDesc"))}</small>
            </span>
          </div>
          <div class="path-playback">
            <button id="pathReviewPlayButton" class="button primary compact" type="button" ${routeReady ? "" : "disabled"}>${materialIcon(state.gcode.pathReviewPlaying ? "pause" : "play_arrow")} ${escapeHtml(t(state.gcode.pathReviewPlaying ? "pausePreview" : robotMotionReady ? "playRobotMotion" : "playPreview"))}</button>
            <button id="pathReviewRestartButton" class="icon-button" type="button" title="${escapeHtml(t("restartPreview"))}" ${routeReady ? "" : "disabled"}>${materialIcon("replay")}</button>
            <input id="pathReviewProgress" type="range" min="0" max="1000" value="${escapeHtml(String(Math.round(state.gcode.pathReviewProgress * 1000)))}" aria-label="${escapeHtml(t(robotMotionReady ? "playRobotMotion" : "playPreview"))}" ${routeReady ? "" : "disabled"}>
            <strong id="pathReviewProgressLabel">${escapeHtml(formatNumber(state.gcode.pathReviewProgress * 100, 1))}%</strong>
            <label><span>${escapeHtml(t("playbackSpeed"))}</span><select id="pathReviewSpeed"><option value="0.5" ${state.gcode.pathReviewPlaybackSpeed === 0.5 ? "selected" : ""}>0.5×</option><option value="1" ${state.gcode.pathReviewPlaybackSpeed === 1 ? "selected" : ""}>1×</option><option value="2" ${state.gcode.pathReviewPlaybackSpeed === 2 ? "selected" : ""}>2×</option></select></label>
            <label><span>${escapeHtml(t("showLayersThrough"))}</span><select id="pathReviewLayer"><option value="all" ${state.gcode.pathReviewLayer === "all" ? "selected" : ""}>${escapeHtml(t("allLayers"))}</option>${layerOptions}</select></label>
          </div>
        </section>

        <aside class="path-review-side">
          <section class="info-panel moveit-validation-card">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("route")} ${escapeHtml(t("moveitValidation"))}</strong><span class="status-badge ${moveit?.status === "passed" ? "pass" : moveitCardFailed ? "fail" : "warning"}">${escapeHtml(moveit?.status === "passed" ? t("gatePassed") : moveitCardFailed ? t("gateFailed") : t("gateRequired"))}</span></div>
            <div class="moveit-validation-body">
              <strong>${escapeHtml(moveitCardTitle)}</strong>
              <p>${escapeHtml(moveitCardDescription)}</p>
              ${moveitResultMetrics(moveit)}
              ${simulationOnly || moveitAvailable ? "" : isOnlineMode()
                ? `<button id="openConnectionFromMoveit" class="button secondary" type="button">${materialIcon("cable")} ${escapeHtml(t("openConnectionModule"))}</button>`
                : `<button id="switchOnlineFromMoveit" class="button secondary" type="button">${materialIcon("sensors")} ${escapeHtml(t("switchOnline"))}</button>`}
              ${state.gcode.pathReviewMoveitRunning ? `<div class="moveit-progress">${materialIcon("progress_activity")}<span><strong>${escapeHtml(t("validatingWithMoveit"))}</strong><small id="moveitValidationLog">${escapeHtml(state.gcode.pathReviewMoveitJob?.log?.split("\n").filter(Boolean).at(-1) || t("pathReviewLoading"))}</small></span></div>` : ""}
              <button id="validatePathWithMoveit" class="button primary" type="button" ${moveitButtonDisabled ? "disabled" : ""}>${materialIcon(state.gcode.pathReviewMoveitRunning ? "progress_activity" : "fact_check")} ${escapeHtml(t(state.gcode.pathReviewMoveitRunning ? "validatingWithMoveit" : "validateWithMoveit"))}</button>
            </div>
          </section>

          <section class="info-panel current-segment-card">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("my_location")} ${escapeHtml(t("currentPathPoint"))}</strong></div>
            <div id="pathReviewSegmentDetail" class="info-panel-body">${pathReviewSegmentMarkup(initialSegment, Math.floor(state.gcode.pathReviewProgress * Math.max(0, review.path.segments.length - 1)), review.path.segments.length)}</div>
          </section>

          <section class="info-panel">
            <div class="info-panel-header"><strong class="panel-title">${materialIcon("verified_user")} ${escapeHtml(t("planningGates"))}</strong></div>
            <div class="planning-gates">${pathReviewGateMarkup()}</div>
          </section>
        </aside>
      </div>

      <div class="callout ${review.path.sampled ? "warning" : ""} path-review-sampling">${materialIcon(review.path.sampled ? "filter_alt" : "task_alt")}<span><strong>${escapeHtml(review.path.sampled ? t("sampledPreview") : t("completePreview"))}</strong>${escapeHtml(t(moveit?.collision_free === true ? "collisionValidatedByMoveit" : "exactCollisionRequired"))}</span></div>

      <div class="path-review-safety-bar">
        <span>${materialIcon("shield_lock")}<span><strong>${escapeHtml(t("pathReviewNoMotion"))}</strong><small>${escapeHtml(t("robotModelSource"))}</small></span></span>
        <span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span>
      </div>
    </div>`;

  bindPathReviewControls();
  window.requestAnimationFrame(mountPathReviewViewer);
}

async function openGcodePathReview(force = false, simulationOnly = false) {
  if (!state.gcode.sourceFile || state.gcode.pathReviewLoading) return;
  const gcodeState = state.gcode;
  const sourceFile = state.gcode.sourceFile;
  const requestSimulationOnly = Boolean(simulationOnly);
  const requestId = Number(state.gcode.pathReviewRequestId || 0) + 1;
  state.gcode.pathReviewRequestId = requestId;
  const requestIsCurrent = () => state.gcode === gcodeState
    && state.gcode.sourceFile === sourceFile
    && state.gcode.pathReviewRequestId === requestId;
  state.gcode.pathReviewRequestedSimulation = requestSimulationOnly;
  state.gcode.view = "path-review";
  if (
    state.gcode.pathReview
    && Boolean(state.gcode.pathReview.simulation_only) === simulationOnly
    && !force
  ) {
    render();
    if (
      !simulationOnly
      && !hasValidatedMoveItTrajectory()
      && moveitValidationAvailable()
      && !state.gcode.pathReviewAutoValidationAttempted
    ) {
      window.setTimeout(() => {
        if (
          !requestIsCurrent()
          || state.gcode.view !== "path-review"
          || state.gcode.pathReviewAutoValidationAttempted
        ) return;
        state.gcode.pathReviewAutoValidationAttempted = true;
        validatePathWithMoveIt();
      }, 0);
    }
    return;
  }
  stopPathReviewPlayback();
  state.gcode.pathReviewLoading = true;
  state.gcode.pathReviewJob = null;
  state.gcode.pathReviewError = "";
  state.gcode.pathReviewMoveitError = "";
  state.gcode.pathReview = null;
  state.gcode.pathReviewMoveitResult = null;
  state.gcode.pathReviewSimulationResult = null;
  state.gcode.pathReviewAutoValidationAttempted = false;
  state.gcode.pathReviewProgress = 0;
  render();
  try {
    const response = await fetchWithTimeout(
      simulationOnly ? "/api/gcode/review/simulation" : "/api/gcode/review",
      {
        method: "POST",
        headers: {
          "Content-Type": "application/octet-stream",
          "Prefer": "respond-async",
          "X-Gcode-Filename": encodeURIComponent(sourceFile.name),
        },
        body: sourceFile,
      },
      20000,
    );
    let payload = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(payload.error || `${t("pathReviewLoadFailed")} (${response.status})`);
    }
    if (!requestIsCurrent()) return;
    let review = payload.review;
    if (payload.review_job) {
      let reviewJob = payload.review_job;
      const reviewJobId = String(reviewJob.id || "");
      if (!/^[0-9a-f]{12}$/i.test(reviewJobId)) {
        throw new Error(t("pathReviewLoadFailed"));
      }
      const deadline = Date.now() + 15 * 60 * 1000;
      const terminalStates = new Set(["succeeded", "failed"]);
      while (
        !terminalStates.has(reviewJob.state)
        || (reviewJob.state === "succeeded" && !reviewJob.review)
      ) {
        if (!["queued", "running", "succeeded"].includes(reviewJob.state)) {
          throw new Error(reviewJob.error || t("pathReviewLoadFailed"));
        }
        if (!requestIsCurrent()) return;
        state.gcode.pathReviewJob = reviewJob;
        render();
        if (Date.now() >= deadline) throw new Error(t("requestTimedOut"));
        if (reviewJob.state !== "succeeded") {
          await new Promise((resolve) => window.setTimeout(resolve, 750));
        }
        if (!requestIsCurrent()) return;
        const statusResponse = await fetchWithTimeout(
          `/api/gcode/review/jobs/${encodeURIComponent(reviewJobId)}`,
          { headers: { "Accept": "application/json", "Cache-Control": "no-store" } },
          20000,
        );
        payload = await statusResponse.json().catch(() => ({}));
        if (!statusResponse.ok) {
          throw new Error(payload.error || `${t("pathReviewLoadFailed")} (${statusResponse.status})`);
        }
        if (!payload.review_job || payload.review_job.id !== reviewJobId) {
          throw new Error(t("pathReviewLoadFailed"));
        }
        reviewJob = payload.review_job;
      }
      if (reviewJob.state !== "succeeded" || !reviewJob.review) {
        throw new Error(reviewJob.error || t("pathReviewLoadFailed"));
      }
      review = reviewJob.review;
    }
    if (!review) throw new Error(t("pathReviewLoadFailed"));
    if (!requestIsCurrent()) return;
    state.gcode.pathReview = review;
    state.gcode.pathReviewRequestedSimulation = Boolean(
      review.simulation_only,
    );
    state.gcode.pathReviewSimulationResult = hasRobotMotionTrajectory(
      review.offline_simulation,
    )
      ? review.offline_simulation
      : null;
    state.gcode.pathReviewMoveitResult = hasRobotMotionTrajectory(
      review.moveit_validation,
    )
      ? review.moveit_validation
      : null;
    state.gcode.pathReviewAutoValidationAttempted = Boolean(
      state.gcode.pathReviewMoveitResult,
    );
    state.gcode.pathReviewError = "";
    state.gcode.pathReviewMoveitError = "";
  } catch (error) {
    if (!requestIsCurrent()) return;
    state.gcode.pathReviewError = error.message || t("pathReviewLoadFailed");
    notify(state.gcode.pathReviewError, "error");
  } finally {
    if (!requestIsCurrent()) return;
    state.gcode.pathReviewJob = null;
    state.gcode.pathReviewLoading = false;
    render();
  }
  if (
    requestIsCurrent()
    && state.gcode.view === "path-review"
    && state.gcode.pathReview
    && !state.gcode.pathReview.simulation_only
    && !hasValidatedMoveItTrajectory()
    && moveitValidationAvailable()
    && !state.gcode.pathReviewAutoValidationAttempted
  ) {
    window.setTimeout(() => {
      if (
        !requestIsCurrent()
        || state.gcode.view !== "path-review"
        || state.gcode.pathReviewAutoValidationAttempted
      ) return;
      state.gcode.pathReviewAutoValidationAttempted = true;
      validatePathWithMoveIt();
    }, 0);
  }
}

async function validatePathWithMoveIt() {
  if (state.gcode.pathReview?.simulation_only) {
    state.gcode.pathReviewMoveitError = t(
      state.gcode.pathReview.physical_calibration_complete
        ? "simulationDraftUnsavedDesc"
        : "simulationMoveitBlockedDesc",
    );
    render();
    return;
  }
  if (!moveitValidationAvailable()) {
    state.gcode.pathReviewMoveitError = t(isOnlineMode() ? "moveitUnavailableDesc" : "offlineMoveitDesc");
    render();
    return;
  }
  if (
    state.gcode.pathReviewMoveitRunning
    || !state.gcode.sourceFile
    || !state.gcode.pathReview
  ) return;
  const gcodeState = state.gcode;
  const sourceFile = state.gcode.sourceFile;
  const pathReview = state.gcode.pathReview;
  const pathReviewRequestId = state.gcode.pathReviewRequestId;
  const requestIsCurrent = () => state.gcode === gcodeState
    && state.gcode.sourceFile === sourceFile
    && state.gcode.pathReview === pathReview
    && state.gcode.pathReviewRequestId === pathReviewRequestId;
  state.gcode.pathReviewAutoValidationAttempted = true;
  resetPrintExecution();
  stopPathReviewPlayback();
  state.gcode.pathReviewMoveitRunning = true;
  state.gcode.pathReviewMoveitResult = null;
  state.gcode.pathReviewMoveitError = "";
  state.gcode.pathReviewMoveitJob = null;
  render();
  try {
    const response = await fetchWithTimeout("/api/gcode/review/moveit", {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
        "X-Gcode-Filename": encodeURIComponent(sourceFile.name),
      },
      body: sourceFile,
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok) {
      throw new Error(payload.error || `${t("moveitValidationFailed")} (${response.status})`);
    }
    if (!requestIsCurrent()) return;
    state.gcode.pathReviewMoveitJob = payload.job;
    let terminal = false;
    while (!terminal) {
      await new Promise((resolve) => window.setTimeout(resolve, 320));
      if (!requestIsCurrent()) return;
      const status = await api(`/api/gcode/review/moveit/${payload.job.id}`);
      if (!requestIsCurrent()) return;
      state.gcode.pathReviewMoveitJob = status.job;
      const log = document.getElementById("moveitValidationLog");
      if (log) {
        log.textContent = status.job.log.split("\n").filter(Boolean).at(-1) || t("validatingWithMoveit");
      }
      terminal = !["queued", "running"].includes(status.job.state);
      if (terminal) {
        state.gcode.pathReviewMoveitResult = status.result;
        state.gcode.pathReviewMoveitError = "";
        if (hasRobotMotionTrajectory(status.result)) {
          state.gcode.pathReviewProgress = 0;
        }
      }
    }
    notify(
      t(
        state.gcode.pathReviewMoveitResult?.status === "passed"
          ? "moveitValidatedStatus"
          : "pathReviewMoveitFailed",
      ),
      state.gcode.pathReviewMoveitResult?.status === "passed" ? "success" : "error",
    );
  } catch (error) {
    if (!requestIsCurrent()) return;
    state.gcode.pathReviewMoveitError = error.message || t("moveitValidationFailed");
    notify(state.gcode.pathReviewMoveitError, "error");
  } finally {
    if (!requestIsCurrent()) return;
    state.gcode.pathReviewMoveitRunning = false;
    render();
  }
}

function renderGcode() {
  if (state.gcode.importMode === "paired") {
    destroyPathReviewViewer();
    stopPathReviewPlayback();
    renderPairedImport();
    return;
  }
  if (!state.gcode.importMode) {
    destroyPathReviewViewer();
    stopPathReviewPlayback();
    elements.mainContent.innerHTML = `
      <div class="gcode-empty-page">
        ${stepHeader(t("gcodeKicker"), t("gcodeTitle"), t("gcodeLead"))}
        ${importModeMarkup()}
        <div class="callout gcode-offline-callout">${materialIcon("shield_lock")}<span><strong>${escapeHtml(t("offlineByDesign"))}</strong>${escapeHtml(t("offlineByDesignDesc"))}</span></div>
      </div>`;
    bindImportMode();
    return;
  }
  if (state.gcode.view === "path-review" && state.gcode.result) {
    renderGcodePathReview();
    return;
  }
  destroyPathReviewViewer();
  stopPathReviewPlayback();
  if (state.gcode.view === "planning" && state.gcode.result) {
    renderGcodePlanning();
    return;
  }
  if (state.gcode.result && !state.gcode.parsing) {
    renderGcodeResult();
    elements.mainContent.insertAdjacentHTML("afterbegin", importModeMarkup());
    bindImportMode();
    return;
  }
  elements.mainContent.innerHTML = `
    <div class="gcode-empty-page">
      ${stepHeader(t("gcodeKicker"), t("gcodeTitle"), t("gcodeLead"))}
      ${importModeMarkup()}
      <div class="gcode-empty-grid">
        <div>
          ${gcodeImportMarkup()}
          ${state.gcode.error ? `<div class="callout danger gcode-import-error">${escapeHtml(state.gcode.error)}</div>` : ""}
          <div class="callout gcode-offline-callout">${materialIcon("shield_lock")}<span><strong>${escapeHtml(t("offlineByDesign"))}</strong>${escapeHtml(t("offlineByDesignDesc"))}</span></div>
        </div>
        <aside>${gcodeParserChecksPanel()}</aside>
      </div>
    </div>`;
  bindGcodeImport();
  bindImportMode();
}

function bindGcodeImport() {
  const input = document.getElementById("gcodeFileInput");
  const chooseButton = document.getElementById("chooseGcodeButton");
  const replaceButton = document.getElementById("replaceGcodeButton");
  const dropZone = document.getElementById("gcodeDropZone");
  const openPicker = () => {
    if (state.gcode.parsing || !input) return;
    // Let the same file be selected again after an edit or for a repeated check.
    input.value = "";
    try {
      if (typeof input.showPicker === "function") {
        input.showPicker();
      } else {
        input.click();
      }
    } catch (_error) {
      // Older WKWebView versions still support a trusted input click.
      input.click();
    }
  };
  chooseButton?.addEventListener("click", (event) => {
    event.stopPropagation();
    openPicker();
  });
  replaceButton?.addEventListener("click", (event) => {
    event.preventDefault();
    openPicker();
  });
  input?.addEventListener("change", () => {
    if (input.files?.[0]) parseGcodeFile(input.files[0]);
  });
  if (!dropZone) return;
  dropZone.addEventListener("click", (event) => {
    if (!event.target.closest("button")) openPicker();
  });
  dropZone.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      openPicker();
    }
  });
  ["dragenter", "dragover"].forEach((eventName) => {
    dropZone.addEventListener(eventName, (event) => {
      event.preventDefault();
      if (!state.gcode.parsing) dropZone.classList.add("drag-active");
    });
  });
  ["dragleave", "drop"].forEach((eventName) => {
    dropZone.addEventListener(eventName, (event) => {
      event.preventDefault();
      dropZone.classList.remove("drag-active");
    });
  });
  dropZone.addEventListener("drop", (event) => {
    const file = event.dataTransfer?.files?.[0];
    if (file && !state.gcode.parsing) parseGcodeFile(file);
  });
}

async function parseGcodeFile(file) {
  if (state.gcode.importMode !== "gcode") return;
  const importRequestId = ++state.gcode.importRequestId;
  resetPrintExecution();
  state.gcode.pathReviewRequestId = Number(state.gcode.pathReviewRequestId || 0) + 1;
  state.gcode.pathReviewJob = null;
  state.gcode.pathReviewLoading = false;
  const allowed = /\.(gcode|gc|ngc|nc)$/i.test(file.name);
  if (!allowed) {
    state.gcode.error = t("gcodeClientFileType");
    state.gcode.file = null;
    state.gcode.sourceFile = null;
    state.gcode.result = null;
    state.gcode.plan = null;
    state.gcode.pathReview = null;
    state.gcode.pathReviewMoveitResult = null;
    state.gcode.pathReviewSimulationResult = null;
    state.gcode.pathReviewAutoValidationAttempted = false;
    state.gcode.pathReviewRequestedSimulation = false;
    render();
    return;
  }
  if (file.size > 20 * 1024 * 1024) {
    state.gcode.error = t("gcodeClientFileSize");
    state.gcode.file = null;
    state.gcode.sourceFile = null;
    state.gcode.result = null;
    state.gcode.plan = null;
    state.gcode.pathReview = null;
    state.gcode.pathReviewMoveitResult = null;
    state.gcode.pathReviewSimulationResult = null;
    state.gcode.pathReviewAutoValidationAttempted = false;
    render();
    return;
  }
  state.gcode.file = { name: file.name, size: file.size };
  state.gcode.sourceFile = file;
  state.gcode.parsing = true;
  state.gcode.result = null;
  state.gcode.error = "";
  state.gcode.view = "review";
  state.gcode.planning = false;
  state.gcode.plan = null;
  state.gcode.planError = "";
  state.gcode.pathReview = null;
  state.gcode.pathReviewError = "";
  state.gcode.pathReviewMoveitError = "";
  state.gcode.pathReviewMoveitRunning = false;
  state.gcode.pathReviewMoveitJob = null;
  state.gcode.pathReviewMoveitResult = null;
  state.gcode.pathReviewSimulationResult = null;
  state.gcode.pathReviewAutoValidationAttempted = false;
  state.gcode.pathReviewRequestedSimulation = false;
  state.gcode.pathReviewProgress = 0;
  state.gcode.pathReviewPlaying = false;
  state.gcode.pathReviewLayer = "all";
  render();
  try {
    const response = await fetchWithTimeout("/api/gcode/parse", {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
        "X-Gcode-Filename": encodeURIComponent(file.name),
      },
      body: file,
    });
    const payload = await response.json().catch(() => ({}));
    if (state.gcode.importMode !== "gcode" || importRequestId !== state.gcode.importRequestId) return;
    if (!response.ok) {
      throw new Error(payload.error || `${t("gcodeImportFailed")} (${response.status})`);
    }
    state.gcode.result = payload.analysis;
    notify(
      t(payload.analysis.safe_to_plan ? "parsedReady" : "parsedAttention"),
      payload.analysis.safe_to_plan ? "success" : "",
    );
  } catch (error) {
    if (state.gcode.importMode !== "gcode" || importRequestId !== state.gcode.importRequestId) return;
    state.gcode.error = error.message || t("gcodeImportFailed");
    notify(state.gcode.error, "error");
  } finally {
    if (state.gcode.importMode === "gcode" && importRequestId === state.gcode.importRequestId) {
      state.gcode.parsing = false;
      render();
    }
  }
}

function drawGcodePreview() {
  const canvas = document.getElementById("gcodePreviewCanvas");
  const result = state.gcode.result;
  if (!canvas || !result) return;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = rect.width;
  const height = rect.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#fbfcfe";
  context.fillRect(0, 0, width, height);

  context.strokeStyle = "#edf0f3";
  context.lineWidth = 1;
  for (let x = 24; x < width; x += 32) {
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
  }
  for (let y = 24; y < height; y += 32) {
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }

  const segments = result.preview.segments;
  if (!segments.length) return;
  const bounds = result.bounds;
  const minX = bounds.minimum.x;
  const maxX = bounds.maximum.x;
  const minY = bounds.minimum.y;
  const maxY = bounds.maximum.y;
  const spanX = Math.max(maxX - minX, 1);
  const spanY = Math.max(maxY - minY, 1);
  const padding = 34;
  const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
  const offsetX = (width - spanX * scale) / 2;
  const offsetY = (height - spanY * scale) / 2;
  const project = (x, y) => [
    offsetX + (x - minX) * scale,
    height - offsetY - (y - minY) * scale,
  ];

  const paths = { travel: new Path2D(), print: new Path2D() };
  segments.forEach((segment) => {
    const [x0, y0] = project(segment.x0, segment.y0);
    const [x1, y1] = project(segment.x1, segment.y1);
    const path = paths[segment.kind === "print" ? "print" : "travel"];
    path.moveTo(x0, y0);
    path.lineTo(x1, y1);
  });

  context.strokeStyle = "#98a2b3";
  context.lineWidth = 1.1;
  context.setLineDash([4, 4]);
  context.stroke(paths.travel);
  context.strokeStyle = "#2563d9";
  context.lineWidth = 1.7;
  context.setLineDash([]);
  context.stroke(paths.print);

  const first = segments[0];
  const last = segments[segments.length - 1];
  const [startX, startY] = project(first.x0, first.y0);
  const [endX, endY] = project(last.x1, last.y1);
  context.fillStyle = "#12a15b";
  context.beginPath();
  context.arc(startX, startY, 4, 0, Math.PI * 2);
  context.fill();
  context.fillStyle = "#d92d20";
  context.beginPath();
  context.arc(endX, endY, 4, 0, Math.PI * 2);
  context.fill();
}

function drawPlanningTimeline() {
  const canvas = document.getElementById("planningTimelineCanvas");
  const timeline = state.gcode.plan?.synchronization?.timeline;
  if (!canvas || !timeline?.events?.length) return;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = rect.width;
  const height = rect.height;
  const paddingX = 8;
  const trackWidth = Math.max(1, width - paddingX * 2);
  const urY = 16;
  const klipperY = Math.max(62, height - 42);
  const trackHeight = 24;
  const total = Math.max(1, timeline.source_event_count);
  const markWidth = Math.max(1.2, trackWidth / Math.max(1, timeline.events.length) * 1.3);

  context.clearRect(0, 0, width, height);
  context.fillStyle = "#fbfcfe";
  context.fillRect(0, 0, width, height);
  context.fillStyle = "#eef2f6";
  context.fillRect(paddingX, urY, trackWidth, trackHeight);
  context.fillRect(paddingX, klipperY, trackWidth, trackHeight);

  context.strokeStyle = "#dfe3e8";
  context.lineWidth = 1;
  for (let index = 0; index <= 10; index += 1) {
    const x = paddingX + trackWidth * index / 10;
    context.beginPath();
    context.moveTo(x, 4);
    context.lineTo(x, height - 4);
    context.stroke();
  }

  timeline.events.forEach((event) => {
    const x = paddingX + Math.min(1, event.sequence / total) * trackWidth;
    if (event.ur_action !== "hold") {
      context.fillStyle = event.ur_action === "print_path" ? "#2563d9" : "#98a2b3";
      context.fillRect(x, urY, markWidth, trackHeight);
    }
    if (event.klipper_action !== "idle") {
      context.fillStyle = event.klipper_action === "extrude" ? "#f79009" : "#7f56d9";
      context.fillRect(x, klipperY, markWidth, trackHeight);
    }
  });
}

function modulePageHeader(runbookId, stepIndex) {
  const runbook = runbooks[runbookId];
  const step = runbook.steps[stepIndex];
  return `
    <header class="module-page-header">
      <div>${stepHeader(t("moduleMapKicker"), t(step.title), t(step.subtitle))}</div>
    </header>`;
}

function renderModuleDependency(runbookId, stepIndex, message, buttonText, onOpen) {
  const flowMiddleRunbookId = runbookId === "execution" ? "motion" : runbookId;
  elements.mainContent.innerHTML = `
    <div class="module-page dependency-page">
      ${modulePageHeader(runbookId, stepIndex)}
      <section class="module-dependency-card">
        <span class="module-hero-icon">${materialIcon("lock_open")}</span>
        <div>
          <div class="step-kicker">${escapeHtml(t("dependencyTitle"))}</div>
          <h2>${escapeHtml(message)}</h2>
          <p>${escapeHtml(t("moduleDependencyLead"))}</p>
          <button id="openModuleDependency" class="button primary" type="button">${materialIcon("arrow_forward")} ${escapeHtml(buttonText)}</button>
        </div>
      </section>
      <div class="module-flow-strip" aria-label="${escapeHtml(t("dependencyTitle"))}">
        <span>${materialIcon("description")} G-code</span>
        ${materialIcon("arrow_forward", "small")}
        <span>${materialIcon(runbooks[flowMiddleRunbookId].icon)} ${escapeHtml(t(runbooks[flowMiddleRunbookId].title))}</span>
        ${materialIcon("arrow_forward", "small")}
        <span class="muted">${materialIcon("play_circle")} ${escapeHtml(t("executionRunbook"))}</span>
      </div>
    </div>`;
  document.getElementById("openModuleDependency")?.addEventListener("click", onOpen);
}

function renderWorkcellModule() {
  const height = state.workcell;
  if (!height) {
    elements.mainContent.innerHTML = `<div class="module-page"><p>${escapeHtml(t("checkingStatus"))}</p></div>`;
    return;
  }
  const currentMm = Number(height.surface_z_m) * 1000;
  const proposedMm = height.draft_surface_z_m === null ? currentMm : Number(height.draft_surface_z_m) * 1000;
  const printZeroMm = typeof height.print_zero_z_m === "number" && Number.isFinite(height.print_zero_z_m)
    ? height.print_zero_z_m * 1000 : null;
  const printOffsetMm = printZeroMm === null ? null : printZeroMm - currentMm;
  const printZApiReady = typeof height.print_z_placement_mode === "string";
  const busy = jobIsRunning() || state.jobSubmitting;
  const nozzle = height.nozzle?.tool0_to_nozzle_m || {};
  const previousTcp = height.printhead_recalibration_required ? height.installed_tcp_m : null;
  const tcpInput = height.printhead_recalibration_required ? nozzle : (height.installed_tcp_m || nozzle);
  const extrusion = height.klipper_extrusion || { units_per_e_mm: 1, direction: "positive" };
  elements.mainContent.innerHTML = `
    <div class="module-page workcell-page">
      ${modulePageHeader("workcell", 0)}
      <section class="section workcell-height-card">
        <div class="step-kicker">${escapeHtml(t("physicalTableTitle"))}</div>
        <h2>${escapeHtml(t("heightCurrent"))}: ${escapeHtml(currentMm.toFixed(3))} mm</h2>
        ${height.legacy_tool0_profile_locked ? `<div class="callout warning">${escapeHtml(t("heightLegacyProfile"))}</div>` : ""}
        <p>${escapeHtml(t("physicalTableLead"))}</p>
        <div class="callout warning">${materialIcon("warning")}<span>${escapeHtml(t("physicalTablePhysicalWarning"))}</span></div>
        <label class="workcell-height-input"><span>${escapeHtml(t("heightCandidate"))}</span>
          <input id="proposedHeightMm" type="number" min="-500" max="500" step="0.1" value="${escapeHtml(proposedMm.toFixed(3))}" ${busy ? "disabled" : ""}>
        </label>
        <div class="workcell-actions">
          <button id="setHeightDraftButton" class="button secondary" type="button" ${busy ? "disabled" : ""}>${escapeHtml(t("heightSaveDraft"))}</button>
          ${height.draft_surface_z_m !== null ? `<button id="discardHeightDraftButton" class="button quiet" type="button" ${busy ? "disabled" : ""}>${escapeHtml(t("heightDiscard"))}</button>` : ""}
        </div>
        <p class="muted">${escapeHtml(t("heightCandidatePlanning"))}</p>
      </section>
      <section class="section workcell-height-card">
        <div class="step-kicker">2 · G-code Z=0 · base_link</div>
        <h2>${escapeHtml(t("printZeroTitle"))}</h2>
        <p>${escapeHtml(t("printZeroLead"))}</p>
        ${!printZApiReady ? `<div class="callout danger">${materialIcon("lock")}<span>${escapeHtml(t("printZBackendRestart"))}</span></div>` : ""}
        ${height.print_z_reverification_required ? `<div class="callout warning">${materialIcon("height")}<span>${escapeHtml(t("printZReverificationRequired"))}</span></div>` : ""}
        <div class="callout info">${materialIcon("table_restaurant")}<span>${escapeHtml(t("printZeroFixedTable", { height: currentMm.toFixed(3) }))}</span></div>
        ${height.draft_surface_z_m !== null ? `<div class="callout warning">${materialIcon("height")}<span>${escapeHtml(t("heightOldProposalHint", { height: proposedMm.toFixed(3) }))}</span></div>` : ""}
        ${height.print_z_candidate && !height.print_z_reverification_required ? `<div class="callout warning">${materialIcon("height")}<span>${escapeHtml(t("printZeroCandidate"))}</span></div>` : ""}
        ${height.print_z_placement_mode === "operator_selected" && !height.print_z_candidate && printZeroMm !== null ? `<div class="callout info">${materialIcon("height")}<span>${escapeHtml(t("printZeroVerified", { height: printZeroMm.toFixed(3) }))}</span></div>` : ""}
        <div class="callout info">${materialIcon("route")}<span>${escapeHtml(t("printZeroMotionReminder"))}</span></div>
        <label class="workcell-height-input"><span>${escapeHtml(t("printZeroLabel"))}</span>
          <input id="printZeroZMm" type="number" min="${escapeHtml(currentMm.toFixed(3))}" max="2000" step="0.1" value="${printZeroMm === null ? "" : escapeHtml(printZeroMm.toFixed(3))}" ${busy || !printZApiReady ? "disabled" : ""}>
        </label>
        ${printOffsetMm === null ? "" : `<p class="muted">${escapeHtml(t("printZeroDelta", { offset: printOffsetMm.toFixed(3) }))}</p>`}
        <div class="workcell-actions">
          <button id="setPrintZDraftButton" class="button secondary" type="button" ${busy || !printZApiReady ? "disabled" : ""}>${escapeHtml(t("printZeroApply"))}</button>
          <button id="discardPrintZDraftButton" class="button quiet" type="button" ${busy || !printZApiReady ? "disabled" : ""}>${escapeHtml(t("printZeroDiscard"))}</button>
        </div>
      </section>
      <section class="section workcell-height-card">
        <h2>${escapeHtml(t("nozzleTcpTitle"))}</h2>
        ${height.printhead_recalibration_required ? `<div class="callout warning">${materialIcon("straighten")}<span>${escapeHtml(t("geometryRecalibrationRequired"))}</span></div>` : ""}
        <p>${escapeHtml(t(height.printhead_recalibration_required ? "nozzleTcpCadLead" : "nozzleTcpLead"))}</p>
        ${previousTcp ? `<p class="muted"><span>${escapeHtml(t("previousToolTcp"))}</span>: <code>[${["x", "y", "z"].map((axis) => (Number(previousTcp[axis] || 0) * 1000).toFixed(3)).join(", ")}] mm</code></p>` : ""}
        <div class="workcell-tcp-grid">
          ${["x", "y", "z"].map((axis) => `<label class="workcell-height-input"><span>${axis.toUpperCase()} (mm)</span><input id="nozzleTcp${axis.toUpperCase()}" type="number" min="-500" max="500" step="0.001" value="${escapeHtml((Number(tcpInput[axis] || 0) * 1000).toFixed(3))}" ${busy ? "disabled" : ""}></label>`).join("")}
        </div>
        <label class="check-row"><input id="nozzlePhysicallyMeasured" type="checkbox" ${state.nozzleVerification.measured ? "checked" : ""} ${busy ? "disabled" : ""}><span>${escapeHtml(t("nozzleMeasured"))}</span></label>
        <label class="check-row"><input id="nozzleOrientationVerified" type="checkbox" ${state.nozzleVerification.orientation ? "checked" : ""} ${busy ? "disabled" : ""}><span>${escapeHtml(t("nozzleOrientation"))}</span></label>
        <p>${escapeHtml(t("extrusionSetupLead"))}</p>
        <div class="workcell-tcp-grid">
          <label class="workcell-height-input"><span>${escapeHtml(t("extrusionUnitsLabel"))}</span><input id="installedExtrusionRatio" type="number" min="0.0001" max="100000" step="any" value="${escapeHtml(extrusion.units_per_e_mm)}" ${busy ? "disabled" : ""}></label>
          <label class="workcell-height-input"><span>${escapeHtml(t("extrusionDirection"))}</span><select id="installedExtrusionDirection" ${busy ? "disabled" : ""}><option value="positive" ${extrusion.direction === "positive" ? "selected" : ""}>${escapeHtml(t("directionPositive"))}</option><option value="negative" ${extrusion.direction === "negative" ? "selected" : ""}>${escapeHtml(t("directionNegative"))}</option></select></label>
        </div>
        <label class="check-row"><input id="installedExtrusionMeasured" type="checkbox" ${state.nozzleVerification.extrusion ? "checked" : ""} ${busy ? "disabled" : ""}><span>${escapeHtml(t("extrusionMeasuredInstalled"))}</span></label>
        <p class="muted">${escapeHtml(t("installedCalibrationStatus", {
          tcp: t(height.installed_tcp_measured ? "calibrationConfirmed" : "calibrationRequired"),
          orientation: t(height.tool_orientation_verified ? "calibrationConfirmed" : "calibrationRequired"),
          extrusion: t(height.extrusion_ratio_measured ? "calibrationConfirmed" : "calibrationRequired"),
        }))}</p>
        <button id="saveNozzleTcpButton" class="button secondary" type="button" ${busy ? "disabled" : ""}>${escapeHtml(t("nozzleSave"))}</button>
      </section>
      ${jobMarkup()}
    </div>`;
  document.getElementById("setHeightDraftButton")?.addEventListener("click", async () => {
    try {
      const raw = document.getElementById("proposedHeightMm").value.trim();
      const value = Number(raw);
      if (!raw || !Number.isFinite(value)) throw new Error(t("heightCandidate"));
      const unchanged = Math.round(value * 1000) === Math.round(currentMm * 1000);
      const response = await api("/api/workcell/height", {
        method: "PUT", body: unchanged ? { discard: true } : { surface_z_mm: value },
      });
      state.workcell = response;
      if (!unchanged || height.draft_surface_z_m !== null) {
        invalidatePlanningForWorkcellChange();
        state.printZMeasuredClearanceMm = "";
        state.heightMeasuredClearanceMm = "";
        Object.keys(state.workcellSafety).forEach((key) => { state.workcellSafety[key] = false; });
      }
      render();
    } catch (error) { notify(error.message, "error"); }
  });
  document.getElementById("setPrintZDraftButton")?.addEventListener("click", async () => {
    try {
      if (!printZApiReady) throw new Error(t("printZBackendRestart"));
      const raw = document.getElementById("printZeroZMm").value.trim();
      const value = Number(raw);
      if (!raw || !Number.isFinite(value)) throw new Error(t("printZeroLabel"));
      state.workcell = await api("/api/workcell/print-z", { method: "PUT", body: { print_zero_z_mm: value } });
      invalidatePlanningForWorkcellChange();
      state.printZMeasuredClearanceMm = "";
      Object.keys(state.workcellSafety).forEach((key) => { state.workcellSafety[key] = false; });
      render();
    } catch (error) { notify(error.message, "error"); }
  });
  document.getElementById("discardPrintZDraftButton")?.addEventListener("click", async () => {
    try {
      if (!printZApiReady) throw new Error(t("printZBackendRestart"));
      state.workcell = await api("/api/workcell/print-z", { method: "PUT", body: { discard: true } });
      invalidatePlanningForWorkcellChange();
      state.printZMeasuredClearanceMm = "";
      Object.keys(state.workcellSafety).forEach((key) => { state.workcellSafety[key] = false; });
      render();
    } catch (error) { notify(error.message, "error"); }
  });
  document.getElementById("discardHeightDraftButton")?.addEventListener("click", async () => {
    try {
      state.workcell = await api("/api/workcell/height", { method: "PUT", body: { discard: true } });
      invalidatePlanningForWorkcellChange();
      state.heightMeasuredClearanceMm = "";
      state.workcellSafety.visual_clearance_confirmed = false;
      render();
    } catch (error) { notify(error.message, "error"); }
  });
  document.getElementById("nozzlePhysicallyMeasured")?.addEventListener("change", (event) => {
    state.nozzleVerification.measured = event.target.checked;
  });
  document.getElementById("nozzleOrientationVerified")?.addEventListener("change", (event) => {
    state.nozzleVerification.orientation = event.target.checked;
  });
  document.getElementById("installedExtrusionMeasured")?.addEventListener("change", (event) => {
    state.nozzleVerification.extrusion = event.target.checked;
  });
  document.getElementById("saveNozzleTcpButton")?.addEventListener("click", async () => {
    try {
      const offset = Object.fromEntries(["x", "y", "z"].map((axis) => {
        const raw = document.getElementById(`nozzleTcp${axis.toUpperCase()}`).value.trim();
        if (!raw || !Number.isFinite(Number(raw))) throw new Error(t("nozzleTcpLead"));
        return [axis, Number(raw)];
      }));
      const ratioRaw = document.getElementById("installedExtrusionRatio").value.trim();
      const ratio = Number(ratioRaw);
      const direction = document.getElementById("installedExtrusionDirection").value;
      if (!ratioRaw || !Number.isFinite(ratio) || ratio < 0.0001 || ratio > 100000 ||
          !["positive", "negative"].includes(direction)) throw new Error(t("extrusionRatioInvalid"));
      state.workcell = await api("/api/workcell/nozzle-tcp", {
        method: "PUT",
        body: {
          operator_action: "save-nozzle-tcp",
          offset_mm: offset,
          physically_measured: state.nozzleVerification.measured,
          orientation_verified: state.nozzleVerification.orientation,
          klipper_extrusion: { units_per_e_mm: ratio, direction },
          extrusion_ratio_measured: state.nozzleVerification.extrusion === true,
        },
      });
      state.nozzleVerification = { measured: false, orientation: false, extrusion: false };
      invalidatePlanningForWorkcellChange();
      state.printZMeasuredClearanceMm = "";
      state.heightMeasuredClearanceMm = "";
      notify(t("nozzleSaved"), "success");
      render();
    } catch (error) { notify(error.message, "error"); }
  });
  bindJobCancel();
}

async function refreshWorkcellHeight() {
  try {
    const next = await api("/api/workcell/height");
    if (state.workcell && (
      state.workcell.planning_surface_z_m !== next.planning_surface_z_m
      || state.workcell.draft_surface_z_m !== next.draft_surface_z_m
      || state.workcell.print_zero_z_m !== next.print_zero_z_m
      || state.workcell.print_z_draft_m !== next.print_z_draft_m
    )) invalidatePlanningForWorkcellChange();
    state.workcell = next;
    if (state.activeRunbook === "workcell") render();
  } catch (error) { notify(error.message, "error"); }
}

function renderMotionModule() {
  const stepIndex = state.moduleSteps.motion || 0;
  if (!state.gcode.result) {
    renderModuleDependency(
      "motion",
      stepIndex,
      t("motionNeedsGcode"),
      t("openRequiredModule", { module: t("gcodeRunbook") }),
      () => selectRunbook("gcode"),
    );
    return;
  }
  if (stepIndex === 0) {
    if (state.gcode.plan || state.gcode.planning) {
      renderGcodePlanning();
      return;
    }
    renderModuleDependency(
      "motion",
      stepIndex,
      t("motionStepChannelsShort"),
      t("generateChannelPlan"),
      () => openGcodePlanning(),
    );
    return;
  }
  if (state.gcode.pathReview || state.gcode.pathReviewLoading || state.gcode.pathReviewError) {
    renderGcodePathReview();
    return;
  }
  renderModuleDependency(
    "motion",
    stepIndex,
    t("motionStepRobotShort"),
    t("buildRobotTrajectory"),
    () => openGcodePathReview(),
  );
}

function moveitExecutionReady() {
  const result = state.gcode.pathReviewMoveitResult;
  return Boolean(
    state.gcode.sourceFile
    && !state.gcode.pathReviewMoveitRunning
    && result?.status === "passed"
    && result?.planning_policy_version === PLANNING_POLICY_VERSION
    && result?.interpolation_validation === "passed"
    && result?.approach_status === "passed"
    && result?.collision_free === true
    && result?.joint_limits === "passed"
    && Number(result?.cartesian_fraction || 0) >= 1 - 1e-6
    && state.gcode.pathReviewMoveitJob?.id,
  );
}

function executionDuration(seconds) {
  const total = Math.max(0, Math.round(Number(seconds || 0)));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const remainder = total % 60;
  return hours > 0
    ? `${hours}h ${String(minutes).padStart(2, "0")}m`
    : `${minutes}m ${String(remainder).padStart(2, "0")}s`;
}

function preparationPhaseLabel(phase) {
  const labels = {
    queued: "preparationPhaseQueued",
    validating_inputs: "preparationPhaseValidatingInputs",
    parsing_gcode: "preparationPhaseParsingGcode",
    rebuilding_review: "preparationPhaseRebuildingReview",
    loading_moveit_review: "preparationPhaseLoadingMoveitReview",
    validating_review: "preparationPhaseValidatingReview",
    loading_trajectories: "preparationPhaseLoadingTrajectories",
    mapping_timeline: "preparationPhaseMappingTimeline",
    certifying_print_curve: "preparationPhaseCertifyingPrintCurve",
    certifying_approach_curve: "preparationPhaseCertifyingApproachCurve",
    writing_package: "preparationPhaseWritingPackage",
    complete: "preparationPhaseComplete",
    failed: "preparationPhaseFailed",
  };
  return t(labels[phase] || "preparationPhaseQueued");
}

function preparationProgressMarkup() {
  const preparation = state.execution.preparation || {};
  const progressValue = Number(preparation.progress);
  const progress = Number.isFinite(progressValue)
    ? Math.max(0, Math.min(1, progressValue))
    : 0;
  const percent = Math.round(progress * 100);
  const etaIsAvailable = preparation.estimated_remaining_s !== null
    && preparation.estimated_remaining_s !== undefined;
  const etaValue = etaIsAvailable
    ? Number(preparation.estimated_remaining_s)
    : Number.NaN;
  const eta = Number.isFinite(etaValue) && etaValue >= 0
    ? t("preparationEta", { time: executionDuration(etaValue) })
    : t("preparationEtaPending");
  return `
    <div class="execution-preparation-progress" role="status" aria-live="polite">
      <div class="execution-preparation-progress-heading">
        <span><small>${escapeHtml(t("preparationProgress"))}</small><strong>${escapeHtml(preparationPhaseLabel(preparation.phase))}</strong></span>
        <strong>${escapeHtml(String(percent))}%</strong>
      </div>
      <progress class="execution-preparation-track" value="${escapeHtml(String(progress))}" max="1" aria-label="${escapeHtml(t("preparationProgress"))}"></progress>
      <small>${escapeHtml(eta)}</small>
    </div>`;
}

function executionGateLabel(code) {
  const labels = {
    immutable_source: "executionGateSource",
    full_resolution_path: "executionGatePath",
    measured_tcp: "executionGateTcp",
    measured_print_bed: "executionGateBed",
    fixed_workcell_tabletop: "executionGateBed",
    tool0_reference: "executionGateTool0",
    moveit_collision_review: "executionGateMoveit",
    joint_trajectory_integrity: "executionGateTrajectory",
  };
  return t(labels[code] || code);
}

function executionPackageMarkup({ compact = false } = {}) {
  const packageData = state.execution.package;
  if (!packageData) return "";
  const tool0MotionTest = packageData.operation_mode === "tool0_motion_test";
  const tool0ReferenceMode = tool0MotionTest;
  const executionScope = packageData.execution_scope || {};
  const fullCold = tool0MotionTest && packageData.execution_extent === "full";
  const minimumDuration = packageData.estimated_minimum_duration_s ?? packageData.planned_duration_s;
  const timing = packageData.timing_audit || {};
  const timingParts = [
    ["nominalGcodeTime", Number(timing.source_nominal_gcode_duration_s)],
    ["reviewedMoveitTime", Number(timing.source_moveit_duration_s)],
    ["processOnlyHoldTime", Number(timing.process_only_hold_duration_s)],
    ["scheduleFloorExtension", Number(timing.schedule_floor_extension_s)],
    ["certifiedRetimeOverhead", Number(timing.certified_retiming_overhead_s)],
    ["certifiedApproachTime", Number(timing.certified_approach_duration_s)],
    ["finalExecutableTime", Number(packageData.planned_duration_s)],
  ].filter(([, value]) => Number.isFinite(value) && value >= 0);
  const hasTimingBreakdown = timingParts.length >= 2;
  const pathCoverage = Math.max(0, Math.min(1, Number(executionScope.end_progress || 0)));
  if (compact) {
    return `
      <section class="execution-package-card compact">
        <div class="execution-package-compact-heading">
          <span class="execution-package-compact-icon">${materialIcon("verified_user")}</span>
          <div>
            <div class="execution-package-compact-title">
              <strong>${escapeHtml(t(fullCold ? "fullColdPackageShort" : tool0MotionTest ? "motionTestPackageShort" : "productionPackageShort"))}</strong>
              <span class="status-badge pass">${escapeHtml(packageData.simulation ? t("simulationOnly") : t("ready"))}</span>
            </div>
            <small>${escapeHtml(t(fullCold ? "fullColdPackageShortDesc" : tool0MotionTest ? "motionTestPackageShortDesc" : "productionPackageShortDesc"))}</small>
          </div>
        </div>
        <div class="execution-package-compact-summary">
          <span><small>${escapeHtml(t(tool0MotionTest ? "minimumWallTime" : "plannedTime"))}</small><strong>${escapeHtml(executionDuration(minimumDuration))}</strong></span>
          <span><small>${escapeHtml(t(tool0MotionTest ? "motionTestCoverage" : "synchronizedBlocks"))}</small><strong>${escapeHtml(tool0MotionTest ? `${(pathCoverage * 100).toFixed(2)}%` : packageData.block_count)}</strong></span>
          ${packageData.heating_enabled === false ? `<span class="execution-package-heat-off">${escapeHtml(t("heatingOff"))}</span>` : ""}
        </div>
      </section>`;
  }
  const bed = packageData.print_bed_m;
  const tcp = packageData.tool_center_point_m;
  const tcpLength = Math.sqrt(
    Number(tcp.offset.x) ** 2 + Number(tcp.offset.y) ** 2 + Number(tcp.offset.z) ** 2,
  );
  return `
    <section class="execution-package-card">
      <div class="execution-package-heading">
        <span>${materialIcon("verified_user")}</span>
        <div><strong>${escapeHtml(t(fullCold ? "fullColdPackageReady" : tool0MotionTest ? "tool0ExecutionPackageReady" : "executionPackageReady"))}</strong><p>${escapeHtml(t(fullCold ? "fullColdPackageDesc" : tool0MotionTest ? "tool0ExecutionPackageReadyDesc" : "executionPackageReadyDesc"))}</p></div>
        <span class="status-badge pass">${escapeHtml(packageData.simulation ? t("simulationOnly") : t("ready"))}</span>
      </div>
      <div class="execution-metric-grid">
        <div><small>${escapeHtml(t("synchronizedBlocks"))}</small><strong>${escapeHtml(packageData.block_count)}</strong></div>
        <div><small>${escapeHtml(t(tool0MotionTest ? "coldMotorBlocks" : "extrusionBlocks"))}</small><strong>${escapeHtml(packageData.extrusion_block_count)}</strong></div>
        <div><small>${escapeHtml(t(tool0MotionTest ? "finalExecutableTime" : "plannedTime"))}</small><strong>${escapeHtml(executionDuration(packageData.planned_duration_s))}</strong></div>
        ${tool0MotionTest ? `<div><small>${escapeHtml(t("minimumWallTime"))}</small><strong>${escapeHtml(executionDuration(minimumDuration))}</strong></div>` : ""}
        ${tool0MotionTest ? `<div><small>${escapeHtml(t("motionTestCoverage"))}</small><strong>${escapeHtml(`${(pathCoverage * 100).toFixed(2)}%`)}</strong></div>` : ""}
        <div><small>${escapeHtml(t("stepper"))}</small><strong><code>${escapeHtml(packageData.manual_stepper)}</code></strong></div>
      </div>
      ${hasTimingBreakdown ? `
        <section class="execution-timing-breakdown" aria-label="${escapeHtml(t("executionTimingBreakdown"))}">
          <strong>${escapeHtml(t("executionTimingBreakdown"))}</strong>
          <div>
            ${timingParts.map(([label, seconds]) => `<span><small>${escapeHtml(t(label))}</small><strong>${escapeHtml(executionDuration(seconds))}</strong></span>`).join("")}
          </div>
        </section>` : ""}
      <p class="execution-progress-note">${escapeHtml(t("executionTimingHint"))}</p>
      <div class="execution-geometry-grid">
        <div>
          <span>${materialIcon("crop_free")}</span>
          <span><small>${escapeHtml(t(tool0MotionTest ? "motionTestGuardPlane" : "printBedLimits"))}</small><strong>X ${Number(bed.minimum.x).toFixed(3)}…${Number(bed.maximum.x).toFixed(3)} m</strong><strong>Y ${Number(bed.minimum.y).toFixed(3)}…${Number(bed.maximum.y).toFixed(3)} m · Z ${Number(bed.plane_z).toFixed(3)} m</strong></span>
        </div>
        <div>
          <span>${materialIcon("hardware")}</span>
          <span><small>${escapeHtml(t(tool0ReferenceMode ? "tool0Reference" : "nozzleTcp"))}</small><strong>${tool0ReferenceMode ? "TCP = [0, 0, 0]" : `${(tcpLength * 1000).toFixed(1)} mm`}</strong><strong>${tool0ReferenceMode ? "HEAT OFF" : `R ${(Number(tcp.collision_radius_m) * 1000).toFixed(1)} mm`}</strong></span>
        </div>
      </div>
      ${packageData.heating_enabled === false ? `<p class="execution-progress-note">${escapeHtml(t("heatingOff"))} · ${escapeHtml(t("executionHeatExternal"))}</p>` : ""}
      <div class="execution-gates">
        ${packageData.gates.map((gate) => `<div><span>${materialIcon("check_circle")}</span><strong>${escapeHtml(executionGateLabel(gate.code))}</strong></div>`).join("")}
      </div>
      <div class="execution-package-id"><span>SHA</span><code>${escapeHtml(packageData.execution_id)} · ${escapeHtml(packageData.source_sha256.slice(0, 16))}</code></div>
    </section>`;
}

function executionAuthorizationMarkup() {
  return `
    <section class="section execution-authorization">
      <div class="section-title-row"><h2>${escapeHtml(t("executionSafetyTitle"))}</h2><span class="status-badge warning">${escapeHtml(t("guardedMotion"))}</span></div>
      <p class="muted">${escapeHtml(t("executionSafetyDesc"))}</p>
      <div class="checklist execution-checklist">
        ${checklistRow("executionEstop", t("estopTitle"), t("estopDesc"), state.safety.emergency_stop)}
        ${checklistRow("executionClear", t("clearTitle"), t("clearDesc"), state.safety.workspace_clear)}
        ${checklistRow("executionSpeed", t("executionSpeedTitle"), t("executionSpeedDesc"), state.execution.reviewedLowSpeed)}
        ${checklistRow("executionWired", t("wiredTitle"), t("wiredDesc"), state.safety.wired_network)}
        ${checklistRow("executionExternal", t("externalControlPlayingExecution"), t("externalControlRequired"), state.execution.externalControlPlaying)}
      </div>
    </section>`;
}

function bindExecutionConfirmations() {
  const bindings = [
    ["executionEstop", state.safety, "emergency_stop"],
    ["executionClear", state.safety, "workspace_clear"],
    ["executionSpeed", state.execution, "reviewedLowSpeed"],
    ["executionWired", state.safety, "wired_network"],
    ["executionExternal", state.execution, "externalControlPlaying"],
  ];
  bindings.forEach(([id, target, key]) => {
    document.getElementById(id)?.addEventListener("change", (event) => {
      target[key] = event.target.checked;
      renderExecutionModule();
      updateFooter();
    });
  });
}

function executionRuntimeMarkup() {
  const runtime = state.execution.runtime;
  const job = state.execution.job || state.activeJob;
  const tool0MotionTest = state.execution.package?.operation_mode === "tool0_motion_test";
  const fullCold = tool0MotionTest && state.execution.package?.execution_extent === "full";
  const stateKeys = {
    starting: "executionStateStarting",
    preflight: "executionStatePreflight",
    running: "executionStateRunning",
    "at-start": "executionStateAtStart",
    completed: "executionStateCompleted",
    failed: "executionStateFailed",
    "stop-unconfirmed": "executionStateStopUnconfirmed",
    canceled: "executionStateCanceled",
    canceling: "executionStateCanceled",
  };
  const progress = Math.max(0, Math.min(1, Number(runtime?.progress || 0)));
  const stateName = runtime?.state || (job?.state === "running" ? "starting" : "");
  const visual = stateName === "completed" || stateName === "at-start"
    ? "pass"
    : ["failed", "stop-unconfirmed"].includes(stateName)
      ? "fail"
      : stateName === "canceled"
        ? "warning"
        : "neutral";
  const preflight = runtime?.preflight;
  const progressHint = runtime?.motion?.source === "live_joint_state"
    ? t("executionLiveJointProgressHint")
    : t("executionBlockProgressHint");
  const failureState = ["failed", "stop-unconfirmed"].includes(stateName);
  const movingToStart = stateName === "running" && runtime?.mode === "move-to-start";
  const remaining = runtime?.motion?.nominal_remaining_s;
  const rootMessage = runtime?.root_error?.message || runtime?.message || job?.status || t("executionWaiting");
  const stopReasons = Array.isArray(runtime?.stop?.reasons) ? runtime.stop.reasons : [];
  const runtimeMessage = stateName === "completed" && tool0MotionTest
    ? t(fullCold ? "fullColdFinishedDesc" : "motionTestFinishedDesc")
    : stateName === "stop-unconfirmed"
      ? t("executionStopLockedDesc")
      : failureState ? t("executionFailureDesc")
        : stateName === "running"
          ? runtime?.motion?.source === "live_joint_state" && typeof remaining === "number" && Number.isFinite(remaining)
            ? t("executionTimeRemaining", { seconds: Math.max(0, Math.ceil(remaining)) })
            : t(movingToStart ? "executionAwaitingMotion" : "executionRunningDesc")
          : runtime?.message || job?.status || t("executionWaiting");
  const preflightSignals = preflight && ["starting", "preflight"].includes(stateName)
    ? [
      ["executionSignalExternal", Boolean(preflight.program_state_received && preflight.external_control_running)],
      ["executionSignalJoints", Boolean(preflight.joint_state_fresh)],
      ["executionSignalKlipper", Boolean(preflight.klipper_action_server)],
      ["executionSignalTrajectory", Boolean(preflight.trajectory_action_server)],
    ]
    : [];
  return `
    <section class="execution-runtime-card ${escapeHtml(visual)}">
      <div class="execution-runtime-heading">
        <span>${materialIcon(stateName === "completed" ? "task_alt" : stateName === "failed" ? "error" : "precision_manufacturing")}</span>
        <div><small>${escapeHtml(t("executionProgress"))}</small><strong>${escapeHtml(movingToStart ? t("executionMovingToStart") : stateName ? t(stateKeys[stateName] || stateName) : t("executionWaiting"))}</strong><p>${escapeHtml(runtimeMessage)}</p></div>
        <strong>${Math.round(progress * 100)}%</strong>
      </div>
      ${failureState ? `<details class="execution-error-details">
        <summary>${escapeHtml(t("executionErrorDetails"))}</summary>
        <strong>${escapeHtml(t("executionRootCause"))}</strong><p>${escapeHtml(rootMessage)}</p>
        ${stopReasons.length ? `<strong>${escapeHtml(t("executionCurrentStop"))}</strong><ul>${stopReasons.map((reason) => `<li>${escapeHtml(String(reason))}</li>`).join("")}</ul>` : ""}
      </details>` : ""}
      ${preflightSignals.length ? `<small class="execution-preflight-caption">${escapeHtml(t("executionPreflightSnapshot"))}</small><div class="execution-preflight-grid">${preflightSignals.map(([label, ready]) => `
        <span class="${ready ? "ready" : "missing"}">${materialIcon(ready ? "check_circle" : "cancel", "small")}<strong>${escapeHtml(t(label))}</strong></span>
      `).join("")}</div>` : ""}
      <progress class="execution-progress-track" value="${progress}" max="1" aria-label="${escapeHtml(t("executionProgress"))}"></progress>
      <p class="execution-progress-note">${escapeHtml(progressHint)}</p>
      <div class="execution-live-metrics">
        <div><small>${escapeHtml(t("currentBlock"))}</small><strong>${escapeHtml(runtime?.current_block || 0)} / ${escapeHtml(runtime?.block_count || state.execution.package?.block_count || 0)}</strong></div>
        <div><small>${escapeHtml(t("sourceLine"))}</small><strong>${escapeHtml(runtime?.source_line || "—")}</strong></div>
        <div><small>${escapeHtml(t("blockKind"))}</small><strong>${escapeHtml(runtime?.block_kind || "—")}</strong></div>
      </div>
    </section>`;
}

async function preparePrintExecution() {
  if (!moveitExecutionReady() || state.execution.preparing || jobIsRunning()
      || state.execution.package || state.execution.runtime?.state === "stop-unconfirmed"
      || state.workcell?.draft_surface_z_m != null || state.workcell?.print_z_candidate) return;
  const executionState = state.execution;
  const sourceFile = state.gcode.sourceFile;
  const moveitJobId = state.gcode.pathReviewMoveitJob.id;
  const extent = state.execution.extent === "sample" ? "sample" : "full";
  const requestIsCurrent = () => state.execution === executionState
    && state.gcode.sourceFile === sourceFile
    && state.gcode.pathReviewMoveitJob?.id === moveitJobId;
  executionState.preparing = true;
  executionState.preparation = {
    state: "queued",
    phase: "queued",
    progress: 0,
    estimated_remaining_s: null,
  };
  executionState.error = "";
  render();
  try {
    const response = await fetchWithTimeout("/api/print/prepare", {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
        "Prefer": "respond-async",
        "X-Gcode-Filename": encodeURIComponent(sourceFile.name),
        "X-Moveit-Job-Id": moveitJobId,
        "X-Execution-Extent": extent,
      },
      body: sourceFile,
    }, 20000);
    let payload = await response.json().catch(() => ({}));
    if (!response.ok) {
      const error = new Error(payload.error || t("executionPackageFailed"));
      error.code = payload.code || "";
      throw error;
    }
    if (payload.preparation) {
      const preparationId = payload.preparation.id;
      const deadline = Date.now() + 2 * 60 * 60 * 1000;
      executionState.preparation = payload.preparation;
      render();
      while (!["succeeded", "failed", "canceled"].includes(payload.preparation.state)) {
        if (!preparationId || Date.now() >= deadline) throw new Error(t("requestTimedOut"));
        await new Promise((resolve) => window.setTimeout(resolve, 750));
        if (!requestIsCurrent()) {
          if (state.execution !== executionState) return;
          throw new Error(t("executionSourceChanged"));
        }
        const statusResponse = await fetchWithTimeout(
          `/api/print/prepare/${encodeURIComponent(preparationId)}`,
          { headers: { "Accept": "application/json", "Cache-Control": "no-store" } },
          20000,
        );
        payload = await statusResponse.json().catch(() => ({}));
        if (!statusResponse.ok) {
          const error = new Error(payload.error || t("executionPackageFailed"));
          error.code = payload.code || "";
          throw error;
        }
        if (!payload.preparation) throw new Error(t("executionPackageFailed"));
        executionState.preparation = payload.preparation;
        render();
      }
      if (payload.preparation.state !== "succeeded" || !payload.preparation.package) {
        const error = new Error(payload.preparation.error || t("executionPackageFailed"));
        error.code = payload.preparation.code || "";
        throw error;
      }
      payload = { package: payload.preparation.package };
    }
    if (!requestIsCurrent()) {
      if (state.execution !== executionState) return;
      throw new Error(t("executionSourceChanged"));
    }
    executionState.package = payload.package;
    executionState.runtime = null;
    executionState.error = "";
    notify(t("executionPackageReady"), "success");
  } catch (error) {
    if (state.execution !== executionState) return;
    executionState.error = error.message || t("executionPackageFailed");
    notify(executionState.error, "error");
  } finally {
    if (state.execution === executionState) {
      executionState.preparing = false;
      executionState.preparation = null;
      render();
    }
  }
}

async function refreshPrintExecutionStatus(renderAfter = true) {
  if (!state.execution.package) return;
  try {
    const payload = await api(`/api/print/${state.execution.package.execution_id}`);
    state.execution.package = payload.package;
    state.execution.runtime = payload.runtime;
    state.execution.requiresRevalidation = payload.requires_revalidation === true
      || payload.code === "requires_revalidation";
    state.execution.job = payload.job;
    const active = ["queued", "running"].includes(payload.job?.state)
      || ["starting", "preflight", "running", "canceling"].includes(payload.runtime?.state);
    window.clearTimeout(state.execution.pollTimer);
    if (active) {
      state.execution.pollTimer = window.setTimeout(() => refreshPrintExecutionStatus(true), 500);
    }
    if (renderAfter) render();
  } catch (error) {
    state.execution.error = error.message;
    if (renderAfter) render();
  }
}

async function startExecutionMotion(mode) {
  if (!["move-to-start", "execute"].includes(mode) || !state.execution.package || jobIsRunning() || state.execution.preparing) return;
  const runtimeState = state.execution.runtime?.state;
  if (["starting", "preflight", "running", "canceling", "stop-unconfirmed"].includes(runtimeState)) return;
  if (state.execution.requiresRevalidation
      || (state.execution.runtime?.mode === "move-to-start"
          && ["failed", "canceled"].includes(runtimeState))) return;
  const moveToStart = mode === "move-to-start";
  if (!moveToStart && runtimeState !== "at-start") return;
  await startJob(
    moveToStart ? "/api/print/move-to-start" : "/api/print/start",
    safetyPayload({
      execution_id: state.execution.package.execution_id,
      external_control_playing: state.execution.externalControlPlaying,
      reviewed_low_speed: state.execution.reviewedLowSpeed,
      operator_action: mode,
    }),
    () => {
      state.moduleSteps.execution = moveToStart ? 1 : 2;
      refreshPrintExecutionStatus(true);
    },
    () => {
      window.setTimeout(() => refreshPrintExecutionStatus(true), 120);
    },
  );
}

async function stopPrintExecution() {
  if (!state.execution.package || state.execution.stopping || state.jobCanceling) return;
  const execution = state.execution;
  const jobId = state.activeJob?.id;
  const cancelToken = jobId || execution.package.execution_id;
  execution.stopping = true;
  state.jobCanceling = cancelToken;
  render();
  try {
    const payload = await api(`/api/print/${execution.package.execution_id}/stop`, {
      method: "POST",
      body: {},
      timeoutMs: 65000,
    });
    if (state.execution !== execution || state.activeJob?.id !== jobId) return;
    state.activeJob = payload.job;
    execution.job = payload.job;
    if (jobId) pollJob(jobId);
    await refreshPrintExecutionStatus();
  } catch (error) {
    if (state.execution === execution && state.activeJob?.id === jobId) notify(error.message, "error");
  } finally {
    execution.stopping = false;
    if (state.jobCanceling === cancelToken) state.jobCanceling = null;
    render();
  }
}

function printRecoveryMarkup() {
  const interlock = state.printInterlock || {};
  const unavailable = interlock.recovery_available === false;
  return `
    <section class="section print-recovery">
      <span class="print-recovery-icon">${materialIcon("health_and_safety")}</span>
      <div class="print-recovery-copy">
        <small>${escapeHtml(t("recoveryKicker"))}</small>
        <h1>${escapeHtml(t("recoveryTitle"))}</h1>
        <p>${escapeHtml(t("recoveryDesc"))}</p>
        <div class="callout warning">${materialIcon("archive")}<span>${escapeHtml(t("recoveryPreservesEvidence"))}</span></div>
        ${unavailable ? `<div class="callout danger">${materialIcon("error")}<span>${escapeHtml(interlock.message || t("recoveryUnavailable"))}</span></div>` : ""}
        ${state.recoveryError ? `<div class="callout danger">${materialIcon("error")}<span>${escapeHtml(state.recoveryError)}</span></div>` : ""}
        <button id="recoverPrintStopButton" class="button primary" type="button" ${state.recoveringPrintStop || unavailable ? "disabled" : ""}>${materialIcon(state.recoveringPrintStop ? "progress_activity" : "restart_alt")} ${escapeHtml(t(state.recoveringPrintStop ? "recoveringPrintStop" : "recoverPrintStop"))}</button>
      </div>
    </section>`;
}

async function recoverPrintStop() {
  if (state.recoveringPrintStop || !state.printInterlock?.locked) return;
  state.recoveringPrintStop = true;
  state.recoveryError = "";
  render();
  try {
    const payload = await api("/api/print/recovery", {
      method: "POST",
      body: { operator_action: "verify-stop-and-recover" },
      timeoutMs: 150000,
    });
    state.printInterlock = payload.recovery?.print_interlock || { locked: false };
    resetPrintExecution();
    state.gcode.pathReviewMoveitJob = null;
    state.gcode.pathReviewMoveitResult = null;
    state.gcode.pathReviewMoveitError = "";
    state.gcode.pathReviewAutoValidationAttempted = false;
    state.completed.delete("motion");
    state.completed.delete("execution");
    state.moduleSteps.motion = 0;
    state.moduleSteps.execution = 0;
    notify(t("recoverySucceeded"), "success");
    await refreshStatus(false);
  } catch (error) {
    state.recoveryError = error.message || t("recoveryUnavailable");
    try {
      const latest = await api("/api/print/recovery");
      state.printInterlock = latest.print_interlock;
    } catch (_refreshError) {
      // Preserve the actionable recovery failure from the POST.
    }
  } finally {
    state.recoveringPrintStop = false;
    render();
  }
}

function renderExecutionModule() {
  const stepIndex = state.moduleSteps.execution || 0;
  if (state.printInterlock?.locked) {
    elements.mainContent.innerHTML = `
      <div class="module-page execution-page">
        ${modulePageHeader("execution", stepIndex)}
        ${printRecoveryMarkup()}
      </div>`;
    document.getElementById("recoverPrintStopButton")?.addEventListener("click", recoverPrintStop);
    return;
  }
  if (!moveitExecutionReady() && !state.execution.package) {
    renderModuleDependency(
      "execution",
      stepIndex,
      t("executionNeedsTrajectory"),
      t("openRequiredModule", { module: t("motionRunbook") }),
      () => {
        state.moduleSteps.motion = 1;
        selectRunbook("motion");
      },
    );
    return;
  }
  const packageData = state.execution.package;
  const tool0MotionTest = packageData?.operation_mode === "tool0_motion_test";
  const fullCold = tool0MotionTest && packageData?.execution_extent === "full";
  const runtime = state.execution.runtime;
  const atStart = runtime?.state === "at-start";
  const stopUnconfirmed = runtime?.state === "stop-unconfirmed";
  const requiresRevalidation = state.execution.requiresRevalidation
    || (runtime?.mode === "move-to-start" && ["failed", "canceled"].includes(runtime?.state));
  const active = state.jobSubmitting || ["queued", "running"].includes(state.execution.job?.state)
    || ["queued", "running"].includes(state.activeJob?.state)
    || ["starting", "preflight", "running", "canceling"].includes(runtime?.state);
  const authorizationReady = state.safety.emergency_stop && state.safety.workspace_clear
    && state.safety.wired_network && state.execution.reviewedLowSpeed && state.execution.externalControlPlaying;
  const stack = state.rosStatus || {};

  if (stepIndex === 0) {
    elements.mainContent.innerHTML = `
      <div class="module-page execution-page">
        ${modulePageHeader("execution", stepIndex)}
        ${stepHeader(t("executionKicker"), t("executionTitle"), t("executionLead"))}
        <div class="execution-readiness-layout">
          <div>
            ${packageData ? executionPackageMarkup() : `
              <section class="section execution-prepare-card">
                <span class="module-hero-icon">${materialIcon("inventory_2")}</span>
                <h2>${escapeHtml(t("prepareExecution"))}</h2>
                <fieldset class="execution-extent" ${state.execution.preparing || active ? "disabled" : ""}>
                  <legend>${escapeHtml(t("executionExtent"))}</legend>
                  <div class="execution-extent-options">
                    ${["full", "sample"].map((extent) => `<label class="execution-extent-option"><input id="executionExtent${extent === "full" ? "Full" : "Sample"}" type="radio" name="executionExtent" value="${extent}" ${state.execution.extent === extent ? "checked" : ""}><span><strong>${escapeHtml(t(extent === "full" ? "executionExtentFull" : "executionExtentSample"))}</strong><small>${escapeHtml(t(extent === "full" ? "executionExtentFullDesc" : "executionExtentSampleDesc"))}</small></span></label>`).join("")}
                  </div>
                </fieldset>
                <p>${escapeHtml(t("fullColdBoundary"))}</p>
                ${state.workcell?.draft_surface_z_m != null ? `<div class="callout warning">${materialIcon("lock")}<span>${escapeHtml(t("heightCandidateReview"))}</span></div>` : ""}
                ${state.workcell?.print_z_candidate ? `<div class="callout warning">${materialIcon("lock")}<span>${escapeHtml(t("printZeroCandidate"))}</span></div>` : ""}
                ${state.execution.preparing ? `${preparationProgressMarkup()}<p>${escapeHtml(t("preparingFullHint"))}</p>` : ""}
                <button id="prepareExecutionButton" class="button primary" type="button" ${state.execution.preparing || active || state.workcell?.draft_surface_z_m != null || state.workcell?.print_z_candidate ? "disabled" : ""}>${materialIcon(state.execution.preparing ? "progress_activity" : "lock")} ${escapeHtml(t(state.execution.preparing ? state.execution.extent === "full" ? "preparingFullExecution" : "preparingExecution" : "prepareExecution"))}</button>
              </section>`}
            ${state.execution.error ? `<div class="callout danger">${materialIcon("error")}<span>${escapeHtml(state.execution.error)}</span></div>` : ""}
          </div>
          <aside>
            <section class="info-panel">
              <div class="info-panel-header"><strong class="panel-title">${materialIcon("hub")} ${escapeHtml(t("systemHealthTitle"))}</strong></div>
              <div class="info-panel-body module-health-grid compact">
                ${liveHealthCard("precision_manufacturing", t("urDriver"), Boolean(stack.driver_started), "UR")}
                ${liveHealthCard("cable", t("externalControl"), Boolean(stack.external_control), t("healthExternalShort"))}
                ${liveHealthCard("route", t("moveit"), Boolean(stack.moveit_ready))}
                ${liveHealthCard("lan", t("klipperBridge"), Boolean(stack.klipper_bridge_ready), "Klipper")}
              </div>
            </section>
          </aside>
        </div>
      </div>`;
    document.getElementById("prepareExecutionButton")?.addEventListener("click", preparePrintExecution);
    document.querySelectorAll('input[name="executionExtent"]').forEach((input) => {
      input.addEventListener("change", (event) => {
        if (state.execution.preparing || active || state.execution.package) return;
        state.execution.extent = event.target.value;
        renderExecutionModule();
      });
    });
    return;
  }

  if (!packageData) {
    state.moduleSteps.execution = 0;
    renderExecutionModule();
    return;
  }

  if (stepIndex === 1) {
    elements.mainContent.innerHTML = `
      <div class="module-page execution-page">
        ${modulePageHeader("execution", stepIndex)}
        <div class="execution-run-layout">
          <div>
            ${executionPackageMarkup()}
            ${executionAuthorizationMarkup()}
          </div>
          <aside>
            ${executionRuntimeMarkup()}
            <section class="section execution-start-card">
              ${requiresRevalidation && !stopUnconfirmed ? `<div class="callout warning">${materialIcon("route")}<span><strong>${escapeHtml(t("approachRevalidationTitle"))}</strong>${escapeHtml(t("approachRevalidationDesc"))}</span></div><button id="reviewApproachAgainButton" class="button secondary full" type="button" ${active ? "disabled" : ""}>${materialIcon("fact_check")} ${escapeHtml(t("reviewApproachAgain"))}</button>` : ""}
              ${atStart ? `<div class="callout success">${materialIcon("my_location")}<span><strong>${escapeHtml(t("atPrintStart"))}</strong> ${escapeHtml(t("atPrintStartDesc"))}</span></div>` : ""}
              ${!tool0MotionTest && !state.demo ? `<div class="callout warning">${materialIcon("info")}<span>${escapeHtml(t(atStart ? "productionPrintNoticeAtStart" : "productionPrintNoticeBeforeStart"))}</span></div>` : ""}
              <button id="moveToStartButton" class="button secondary full" type="button" ${requiresRevalidation || stopUnconfirmed || active || atStart || !authorizationReady || state.workcell?.print_z_candidate ? "disabled" : ""}>${materialIcon("my_location")} ${escapeHtml(t("moveToStart"))}</button>
              <button id="startPrintButton" class="button primary full" type="button" ${stopUnconfirmed || active || !atStart || !authorizationReady || state.workcell?.print_z_candidate ? "disabled" : ""}>${materialIcon("play_arrow")} ${escapeHtml(t(fullCold ? "startFullColdTest" : tool0MotionTest ? "startMotionTest" : "startPrint"))}</button>
            </section>
            ${jobMarkup()}
          </aside>
        </div>
      </div>`;
    bindExecutionConfirmations();
    document.getElementById("reviewApproachAgainButton")?.addEventListener("click", () => {
      state.moduleSteps.motion = 1;
      selectRunbook("motion");
    });
    document.getElementById("moveToStartButton")?.addEventListener("click", () => startExecutionMotion("move-to-start"));
    document.getElementById("startPrintButton")?.addEventListener("click", () => startExecutionMotion("execute"));
    bindJobCancel();
    return;
  }

  if (stepIndex === 2) {
    const complete = runtime?.state === "completed";
    const finishedTitleKey = fullCold ? "fullColdFinishedTitle" : tool0MotionTest ? "motionTestFinishedTitle" : "executionFinishedTitle";
    const finishedDescKey = fullCold ? "fullColdFinishedDesc" : tool0MotionTest ? "motionTestFinishedDesc" : "executionFinishedDesc";
    const canControl = runtime?.state === "running"
      || ["queued", "running"].includes(state.execution.job?.state)
      || ["queued", "running"].includes(state.activeJob?.state);
    elements.mainContent.innerHTML = `
      <div class="module-page execution-page">
        ${modulePageHeader("execution", stepIndex)}
        <div class="execution-control-layout">
          <div>
            ${complete ? `
              <section class="execution-complete-card complete">
                <span>${materialIcon("task_alt")}</span>
                <div><h1>${escapeHtml(t(finishedTitleKey))}</h1><p>${escapeHtml(t(finishedDescKey))}</p></div>
              </section>` : ""}
            ${executionRuntimeMarkup()}
            <section class="section execution-controls-card">
              <div class="button-row">
                <button id="stopPrintButton" class="button danger" type="button" ${!canControl ? "disabled" : ""}>${materialIcon("stop_circle")} ${escapeHtml(t("stopPrint"))}</button>
              </div>
              <div class="callout warning execution-safety-callout">${materialIcon("stop_circle")}<span class="execution-safety-copy"><strong>${escapeHtml(t("stopSemantics"))}</strong></span></div>
            </section>
            ${jobMarkup()}
          </div>
          <aside>${executionPackageMarkup({ compact: true })}</aside>
        </div>
      </div>`;
    document.getElementById("stopPrintButton")?.addEventListener("click", stopPrintExecution);
    bindJobCancel();
    return;
  }
}

function liveHealthCard(icon, label, ready, shortLabel = label) {
  return `
    <div class="module-health-card ${ready ? "ready" : "waiting"}" title="${escapeHtml(label)}" aria-label="${escapeHtml(label)}: ${escapeHtml(t(ready ? "ready" : "waiting"))}">
      <span>${materialIcon(icon)}</span>
      <div><strong>${escapeHtml(shortLabel)}</strong><small>${materialIcon(ready ? "check_circle" : "radio_button_unchecked", ready ? "small pass-text" : "small muted-icon")} ${escapeHtml(t(ready ? "ready" : "waiting"))}</small></div>
    </div>`;
}

function renderOnlineModeGate() {
  const runbook = runbooks[state.activeRunbook];
  const ros = state.rosStatus || {};
  const servicesRunning = Boolean(
    ros.container_running
    || ros.driver_started
    || ros.moveit_running
    || ros.klipper_bridge_ready,
  );
  elements.mainContent.innerHTML = `
    <div class="mode-gate-page">
      <section class="mode-gate-card">
        <span class="mode-gate-icon">${materialIcon("shield_lock")}</span>
        <div class="mode-gate-copy">
          <span class="eyebrow">${escapeHtml(t("offlineModeGateKicker"))}</span>
          <h1>${escapeHtml(t("onlineModeRequired"))}</h1>
          <p class="lead">${escapeHtml(t("onlineModeRequiredDesc"))}</p>
          <div class="mode-gate-target">
            <span>${materialIcon(runbook.icon)}</span>
            <span><small>${escapeHtml(t("currentRunbook"))}</small><strong>${escapeHtml(t(runbook.title))}</strong></span>
            <span class="status-badge warning">${escapeHtml(t("onlineOnly"))}</span>
          </div>
          <div class="button-row mode-gate-actions">
            <button id="switchOnlineFromGate" class="button primary" type="button">${materialIcon("sensors")} ${escapeHtml(t("switchOnline"))}</button>
            <button id="openOfflinePreparation" class="button secondary" type="button">${materialIcon("description")} ${escapeHtml(t("openOfflinePreparation"))}</button>
            ${servicesRunning ? `<button id="stopServicesFromModeGate" class="button danger-quiet" type="button">${materialIcon("power_settings_new")} ${escapeHtml(t("stopRunningServices"))}</button>` : ""}
          </div>
        </div>
      </section>
      <aside class="mode-gate-aside">
        <section class="info-panel">
          <div class="info-panel-header"><strong class="panel-title">${materialIcon("task_alt")} ${escapeHtml(t("offlineMode"))}</strong><span class="status-badge neutral">${escapeHtml(t("readOnly"))}</span></div>
          <div class="info-panel-body">
            <p>${escapeHtml(t("offlineModeSafeWork"))}</p>
            <div class="callout ${servicesRunning ? "warning" : ""}">${materialIcon(servicesRunning ? "info" : "shield_lock")}<span>${escapeHtml(t(servicesRunning ? "backgroundServicesRunning" : "noBackgroundOperation"))}</span></div>
          </div>
        </section>
      </aside>
    </div>`;

  document.getElementById("switchOnlineFromGate")?.addEventListener("click", () => {
    changeOperatingMode("online");
  });
  document.getElementById("openOfflinePreparation")?.addEventListener("click", () => {
    selectRunbook("gcode");
  });
  document.getElementById("stopServicesFromModeGate")?.addEventListener("click", () => {
    startJob("/api/system/stop", { operator_action: "stop-ros" }, () => {
      state.rosStages = { driver: false, external: false, moveit: false, bridge: false };
      state.completed.delete("ros");
      notify(t("stopAccepted"));
    });
  });
  bindJobCancel();
}

function render() {
  if (
    state.activeRunbook !== "motion"
    || state.gcode.view !== "path-review"
  ) {
    destroyPathReviewViewer();
    stopPathReviewPlayback();
  }
  applyStaticTranslations();
  renderNavigation();
  if (!isOnlineMode() && runbookRequiresOnline()) {
    renderOnlineModeGate();
    updateFooter();
    updateGlobalStatus();
    return;
  }
  const moduleRenderers = {
    workcell: renderWorkcellModule,
    gcode: renderGcode,
    motion: renderMotionModule,
    execution: renderExecutionModule,
  };
  if (state.activeRunbook === "connection") {
    [renderSafety, renderUr, renderKlipper, renderRos][state.currentStep]();
  } else {
    moduleRenderers[state.activeRunbook]();
  }
  updateFooter();
  updateGlobalStatus();
}

function updateFooter() {
  const isOffline = !isOnlineMode();
  elements.footerSafetyIcon.textContent = isOffline ? "shield_lock" : "verified_user";
  elements.footerSafetyIcon.classList.toggle("safe", !isOffline);
  elements.footerSafetyIcon.classList.toggle("offline", isOffline);
  elements.motionLockContainer.classList.remove("unlocked");
  elements.nextButton.disabled = true;
  elements.nextButton.firstElementChild.textContent = t("continue");

  if (isOffline && runbookRequiresOnline()) {
    elements.footerSafetyText.textContent = t("offlineModeStatus");
    elements.motionLockText.textContent = t("motionLocked");
    elements.nextButton.disabled = true;
    elements.nextButton.firstElementChild.textContent = t("switchOnline");
    return;
  }

  if (state.activeRunbook === "connection") {
    elements.footerSafetyText.textContent = t("estopWorkspace");
    elements.motionLockText.textContent = t("motionLocked");
    const current = steps[state.currentStep];
    const complete = state.completed.has(current.id);
    elements.nextButton.disabled = !complete;
    elements.nextButton.firstElementChild.textContent = state.currentStep < steps.length - 1
      ? t(steps[state.currentStep + 1].title)
      : t("gcodeRunbook");
    return;
  }

  if (state.activeRunbook === "gcode") {
    elements.footerSafetyText.textContent = t("noHardwareCommands");
    elements.motionLockText.textContent = t("readOnlyWorkspace");
    const ready = Boolean(state.gcode.importMode === "gcode"
      && state.gcode.result?.safe_to_plan && state.gcode.sourceFile && !state.gcode.parsing);
    elements.nextButton.disabled = !ready;
    elements.nextButton.firstElementChild.textContent = t("workcellRunbook");
    return;
  }

  if (state.activeRunbook === "workcell") {
    elements.footerSafetyText.textContent = t(state.workcell?.installed_tcp_measured ? "heightMeasuredTcp" : "heightTrialUnverified");
    elements.motionLockText.textContent = t("motionLocked");
    elements.nextButton.disabled = state.gcode.importMode !== "gcode"
      || !state.gcode.result?.safe_to_plan || !state.gcode.sourceFile
      || typeof state.workcell?.print_zero_z_m !== "number" || !Number.isFinite(state.workcell.print_zero_z_m);
    elements.nextButton.firstElementChild.textContent = t("motionRunbook");
    return;
  }

  if (state.activeRunbook === "motion") {
    elements.footerSafetyText.textContent = t("planningOnly");
    elements.motionLockText.textContent = t("motionLocked");
    const stepIndex = state.moduleSteps.motion || 0;
    const ready = stepIndex === 0
      ? Boolean(state.gcode.plan)
      : hasValidatedMoveItTrajectory();
    elements.nextButton.disabled = !ready;
    elements.nextButton.firstElementChild.textContent = stepIndex === 0
      ? t("motionStepRobot")
      : t("executionRunbook");
    return;
  }

  if (state.activeRunbook === "execution") {
    const stepIndex = state.moduleSteps.execution || 0;
    const runtimeState = state.execution.runtime?.state;
    const executionActive = ["starting", "preflight", "running", "canceling"].includes(runtimeState);
    const executionComplete = runtimeState === "completed";
    elements.footerSafetyText.textContent = t("executionFooterSafety");
    elements.motionLockText.textContent = t(
      executionComplete
        ? "executionFooterComplete"
        : executionActive
          ? "executionFooterActive"
          : "executionFooterLocked",
    );
    if (stepIndex === 0) {
      elements.nextButton.disabled = !state.execution.package;
      elements.nextButton.firstElementChild.textContent = t("executionStepRun");
    } else if (stepIndex === 1) {
      elements.nextButton.disabled = !["running", "completed", "failed", "canceled", "stop-unconfirmed"].includes(runtimeState);
      elements.nextButton.firstElementChild.textContent = t("executionStepControl");
    } else {
      elements.nextButton.disabled = true;
      elements.nextButton.firstElementChild.textContent = t(executionComplete ? "executionFinishedTitle" : "executionStepControl");
    }
    return;
  }

  elements.footerSafetyText.textContent = t("continue");
  elements.motionLockText.textContent = t("motionLocked");
  elements.nextButton.firstElementChild.textContent = t("continue");
}

function updateGlobalStatus() {
  if (!isOnlineMode()) {
    elements.globalStatus.innerHTML = `
      <span class="status-dot neutral"></span>
      <span>${escapeHtml(t("offlineModeStatus"))}</span>`;
    return;
  }
  const ros = state.rosStatus || {};
  const ready = ros.external_control && ros.moveit_ready && ros.klipper_bridge_ready && !ros.unstable;
  const warning = Boolean(ros.unstable || ros.rtde_conflict);
  elements.globalStatus.innerHTML = `
    <span class="status-dot ${ready ? "pass" : warning ? "warning" : "neutral"}"></span>
    <span>${escapeHtml(ready ? t("onlineReadyStatus") : t("onlineNeedsConnectionStatus"))}</span>`;
}

async function refreshStatus(renderAfter = true) {
  const observedJob = state.activeJob;
  try {
    const payload = await api("/api/status");
    state.rosStatus = payload.ros;
    const previousInterlockLocked = Boolean(state.printInterlock?.locked);
    state.printInterlock = payload.print_interlock || null;
    if (payload.active_job && state.activeJob === observedJob && !state.jobSubmitting) {
      state.activeJob = payload.active_job;
      if (!state.jobPollTimer) pollJob(payload.active_job.id);
    }
    if (!state.demo) {
      if (payload.ros.driver_started) state.rosStages.driver = true;
      if (payload.ros.external_control) state.rosStages.external = true;
      if (payload.ros.moveit_ready) state.rosStages.moveit = true;
      state.rosStages.bridge = Boolean(payload.ros.klipper_bridge_ready);
      if (state.rosStages.moveit && state.rosStages.bridge) state.completed.add("ros");
    }
    if (renderAfter || previousInterlockLocked !== Boolean(state.printInterlock?.locked)) render();
    return payload;
  } catch (error) {
    notify(error.message, "error");
  }
}

function openSettings() {
  const form = elements.settingsForm;
  Object.entries(state.config).forEach(([key, value]) => {
    if (form.elements[key]) form.elements[key].value = value;
  });
  elements.settingsDialog.hidden = false;
  form.elements.ur_robot_ip.focus();
}

function closeSettings() {
  elements.settingsDialog.hidden = true;
  elements.settingsButton.focus();
}

async function saveConfig(updates, showToast = true) {
  const payload = await api("/api/config", { method: "PUT", body: updates });
  state.config = payload.config;
  if (showToast) notify(t("settingsSaved"), "success");
  return state.config;
}

async function submitSettings(event) {
  event.preventDefault();
  const formData = new FormData(elements.settingsForm);
  const payload = Object.fromEntries(formData.entries());
  ["ur_update_rate", "distance_m", "max_joint_speed_rad_s", "stepper_move", "stepper_speed", "stepper_accel"].forEach((key) => {
    payload[key] = Number(payload[key]);
  });
  try {
    await saveConfig(payload);
    closeSettings();
    render();
  } catch (error) {
    notify(error.message, "error");
  }
}

function bindGlobalEvents() {
  elements.offlineModeButton.addEventListener("click", () => {
    changeOperatingMode("offline");
  });
  elements.onlineModeButton.addEventListener("click", () => {
    changeOperatingMode("online");
  });
  elements.runbookButton.addEventListener("click", (event) => {
    event.stopPropagation();
    setRunbookMenuOpen(elements.runbookMenu.hidden);
  });
  document.addEventListener("click", (event) => {
    if (!event.target.closest(".runbook-picker")) setRunbookMenuOpen(false);
  });
  elements.settingsButton.addEventListener("click", openSettings);
  elements.closeSettingsButton.addEventListener("click", closeSettings);
  elements.cancelSettingsButton.addEventListener("click", closeSettings);
  elements.settingsForm.addEventListener("submit", submitSettings);
  elements.nextButton.addEventListener("click", () => {
    if (elements.nextButton.disabled) return;
    if (state.activeRunbook === "connection") {
      if (!state.completed.has(steps[state.currentStep].id)) return;
      if (state.currentStep < steps.length - 1) {
        state.currentStep += 1;
        render();
      } else {
        selectRunbook("gcode");
      }
      elements.mainContent.focus();
      return;
    }
    if (state.activeRunbook === "gcode") {
      selectRunbook("workcell");
      return;
    }
    if (state.activeRunbook === "workcell") {
      state.moduleSteps.motion = 0;
      selectRunbook("motion");
      return;
    }
    if (state.activeRunbook === "motion") {
      if ((state.moduleSteps.motion || 0) === 0) {
        state.moduleSteps.motion = 1;
        state.gcode.view = "path-review";
        openGcodePathReview();
      } else {
        state.moduleSteps.execution = 0;
        selectRunbook("execution");
      }
      return;
    }
    if (state.activeRunbook === "execution") {
      const stepIndex = state.moduleSteps.execution || 0;
      if (stepIndex < executionSteps.length - 1) {
        state.moduleSteps.execution = stepIndex + 1;
        render();
        elements.mainContent.focus();
      }
    }
  });
  elements.stopSystemButton.addEventListener("click", () => {
    closeSettings();
    startJob("/api/system/stop", { operator_action: "stop-ros" }, () => {
      state.rosStages = { driver: false, external: false, moveit: false, bridge: false };
      state.completed.delete("ros");
      notify(t("stopAccepted"));
    });
  });
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      if (!elements.runbookMenu.hidden) setRunbookMenuOpen(false);
      else if (!elements.settingsDialog.hidden) closeSettings();
    }
  });
  window.addEventListener("resize", () => {
    if (state.gcode.result) {
      if (state.activeRunbook === "motion" && state.gcode.view === "path-review") {
        state.gcode.pathReviewViewer?.resize?.();
      } else if (state.activeRunbook === "motion" && state.gcode.view === "planning") {
        window.requestAnimationFrame(drawPlanningTimeline);
      } else if (state.activeRunbook === "gcode") {
        window.requestAnimationFrame(drawGcodePreview);
      }
    }
  });
}

async function initialize() {
  [
    "stepNavigation",
    "mainContent",
    "runbookButton",
    "runbookMenu",
    "currentRunbookName",
    "runbookSidebarName",
    "currentDateTime",
    "demoBadge",
    "operatingModeSelector",
    "offlineModeButton",
    "onlineModeButton",
    "globalStatus",
    "settingsButton",
    "motionLockText",
    "motionLockContainer",
    "footerSafetyIcon",
    "footerSafetyText",
    "nextButton",
    "settingsDialog",
    "settingsForm",
    "closeSettingsButton",
    "cancelSettingsButton",
    "stopSystemButton",
    "toastRegion",
  ].forEach((id) => {
    elements[id] = document.getElementById(id);
  });
  localStorage.setItem("urStationLanguage", "en");
  bindGlobalEvents();
  try {
    const [health, config, mode, status, workcell] = await Promise.all([
      api("/api/health"),
      api("/api/config"),
      api("/api/mode"),
      api("/api/status"),
      api("/api/workcell/height"),
    ]);
    state.demo = health.demo;
    state.config = config.config;
    state.workcell = workcell;
    state.operatingMode = OPERATING_MODE_VALUES.has(mode.mode) ? mode.mode : "offline";
    state.rosStatus = status.ros;
    elements.demoBadge.hidden = !state.demo;
    if (status.active_job) {
      state.activeJob = status.active_job;
      pollJob(status.active_job.id);
    }
    render();
    updateClock();
    window.setInterval(updateClock, 30000);
    window.setInterval(() => {
      if (!state.activeJob || !["queued", "running"].includes(state.activeJob.state)) refreshStatus(false);
    }, 10000);
  } catch (error) {
    elements.mainContent.innerHTML = `<div class="callout danger">${escapeHtml(error.message)}</div>`;
    notify(error.message, "error");
  }
}

window.addEventListener("DOMContentLoaded", initialize);
