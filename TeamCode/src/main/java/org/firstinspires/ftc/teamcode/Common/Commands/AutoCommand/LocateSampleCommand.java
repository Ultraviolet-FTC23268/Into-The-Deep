package org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand;

import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.MAX_ROTATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.MAX_TRANSLATIONAL_SPEED;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.X_GAIN;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.Y_GAIN;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.hD;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.hP;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.k_static;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.min_power;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.min_power_h;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.xD;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.xP;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.yD;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.yP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.DetectionPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;

import java.util.Locale;

@Config
public class LocateSampleCommand extends CommandBase {

    // A list of system States.
    private enum State
    {
        //states are defined with default timeouts for each state in ms.
        STATE_INITIAL(500),             //Initial state
        STATE_VISUAL_SEARCH(3000),      //Visual search using camera
        STATE_SHIFT_BACK(2000),         //Shifting back when sample not detected
        STATE_MOVE_TO_SAMPLE(3000),     //Moving towards the sample
        STATE_PICK_UP_SAMPLE(3000),     //Picking up the sample
        STATE_PREPARE_TO_RTB(3000),     //Preparing to return to base or continue execution of OpMode
        STATE_STOP(-1); //No timeout    //Emergency stop (if needed)

        private final int timeoutMs; // Timeout in milliseconds

        State(int timeoutMs) {
            this.timeoutMs = timeoutMs;
        }

        public int getTimeout() {
            return timeoutMs;
        }
    }

    //Loop cycle time stats variables
    /**
     * Overall elapsed time
     */
    public ElapsedTime runtimeTimer = new ElapsedTime();

    /**
     * Time in current state
     */
    private ElapsedTime stateTimer = new ElapsedTime();

    /**
     * Current state
     */
    private State currentState;    // Current State Machine State.

    //private int[] stateTimeout
    /**
     * Telemetry object for displaying telemetry data on Driver Station.
     */
    private Telemetry telemetry;

    private RobotHardware robot = RobotHardware.getInstance();
    private DetectionPipeline.AnalyzedSample detectedSample = null;

    //public double slideExtendedPos = 0.4;
    public boolean finished = false;
    private boolean cancelled = false;
    private boolean isRobotAtSampleLocation = false;
    private final boolean isLoggingEnabled = true;

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;
    public static double TARGET_X = 25;

    private Pose startPose;

    public static PIDFController xController;
    public static PIDFController yController;
    public static PIDFController hController;

    /**
     * Allowed translational error in mm
     */
    public static double ALLOWED_TRANSLATIONAL_ERROR = 10;

    /**
     * Stable motion timer
     */
    private ElapsedTime stableTimer;

    /**
     * Time to ensure stability around desired position of the robot.
     */
    private final int stableTimerTimeout = 100;

    /**
     * Incremental distance for backward shift during a search
     */
    private final int backwardShiftDistance = 30;

    /**
     * Maximum allowed backward shift distance
     */
    private final int maxBackwardShiftDistance = 300;

    /**
     * Current backward shift offset. This is incremented each time we move back.
     */
    private int currentBackwardShiftOffset = 0;


    /**
     * Indicates that the robot has picked up the sample
     */
    private boolean hasPickedUpSample = false;

    // Target pose for moving to the sample (set once, used across execute() calls)
    private Pose moveToSampleTargetPose = null;

    /**
     * Stores the current pickup attempt number
     */
    private int currentPickupAttemptNumber = 0;
    /**
     * Maximum number of sample pickup attempts
     */
    private final int maxNumberOfPickupAttempts = 3;

    /**
     * A flag used to signal to the FSM that the pickup attempt failed
     */
    private boolean currentPickupAttemptFailed = false;

    public LocateSampleCommand(SampleType sampleType, Telemetry telemetry) {
        robot.pipeline.sampleType = sampleType;
        //robot.setPipelineEnabled(robot.pipeline, true);
        xController = new PIDFController(xP*2.5, 0.0, xD, 0);
        yController = new PIDFController(yP, 0.0, yD, 0);
        hController = new PIDFController(hP, 0.0, hD, 0);

        xController.reset();
        yController.reset();
        hController.reset();

        robot.intake.update(IntakeSubsystem.IntakeState.OVERVIEW);
        robot.intake.update(IntakeSubsystem.ExtendoState.EXTENDED);

        this.telemetry = telemetry;
        initStartSearching(); //capture the initial robot pose

        runtimeTimer.reset();           // Zero game clock
        newState(State.STATE_INITIAL);
    }

    @Override
    public void execute() {
        if (cancelled) {
            cancelSearch();
            newState(State.STATE_STOP);
            return;
        }

        // Send the current state info (state and time) back to first line of driver station telemetry.
        logTelemetry("0", String.format(Locale.US, "%4.1f ", stateTimer.time()) + currentState.toString());

        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //
        switch (currentState) {
            case STATE_INITIAL:
                processStateInitial();
                break;
            case STATE_VISUAL_SEARCH:
                processStateVisualSearch();
                break;
            case STATE_SHIFT_BACK:
                processStateShiftBack();
                break;
            case STATE_MOVE_TO_SAMPLE:
                processStateMoveToSample();
            case STATE_PICK_UP_SAMPLE:
                processStatePickUpSample();
            case STATE_STOP:
            default:
                break;

        }
    }

    /**
     * Process the Pick-Up-Sample state of the FSM
     */
    private void processStatePickUpSample() {
        //Check end-of-state condition
        if (hasPickedUpSample) {
            //Start new action
            SignalCompletion();
            //change state
            newState(State.STATE_PREPARE_TO_RTB);
        }
        else if (IsStateTimedOut(currentState)) {
            //Could not pick up the sample
            cancelSearch();
        }
        else {
            // Display Diagnostic data for this state.
            trackSamplePickup();
            logTelemetry("1", "Picking up sample (attempt:" + currentPickupAttemptNumber + ")");
        }
    }

    /**
     * Signals completion of the entire action
     */
    private void SignalCompletion() {
        finished = true;
        logTelemetry("1", "Picked up sample");
    }

    /**
     * Tracks the process of picking up the detected sample
     */
    private void trackSamplePickup() {
        if (currentPickupAttemptFailed) {
            currentPickupAttemptFailed = false;
            if (currentPickupAttemptNumber<maxNumberOfPickupAttempts) {
                //Retry picking up sample
                log("LocateSampleCommand: retrying sample pickup (attempt: " + currentPickupAttemptNumber + ")");
                pickUpSample();
            } else {
                cancelSearch();
            }
        }
    }

    /**
     * Process the Move-To-Sample state of the FSM
     */
    private void processStateMoveToSample() {
        //Check end-of-state condition
        if (isRobotAtSampleLocation) {
            //Start new action
            pickUpSample();
            //change state
            newState(State.STATE_PICK_UP_SAMPLE);
        }
        else if (IsStateTimedOut(currentState)) {
            //Could not reach the sample
            cancelSearch();
        }
        else {
            // Display Diagnostic data for this state.
            trackMovementToSample();
            Pose delta = getMoveToSampleDelta();
            logTelemetry("1", "Moving to sample (delta:" + delta + ")");
        }
    }

    /**
     * Processes the Shift Back state of the FSM
     */
    private void processStateShiftBack() {
        //Check end-of-state condition
        if (IsStateTimedOut(currentState)) {
            //Start new action
            DetectSample();
            //change state
            newState(State.STATE_VISUAL_SEARCH);
        }
        else {
            // Display Diagnostic data for this state.
            logTelemetry("1", "Shifting robot back");
        }
    }



    /**
     * Processes the Visual Search state of the FSM
     */
    private void processStateVisualSearch() {
        //Check end-of-state condition
        if (detectedSample != null) {
            //Start new action
            moveToSample();
            //change state
            newState(State.STATE_MOVE_TO_SAMPLE);
        }
        else if (IsStateTimedOut(currentState)) {
            //If we have not exceeded the maximum backward shift distance
            //then shift back, otherwise cancel the operation
            if (currentBackwardShiftOffset<maxBackwardShiftDistance) {
                //Start new action
                shiftRobotBack();
                //change state
                newState(State.STATE_SHIFT_BACK);
            } else {
                //Start new action
                cancelSearch();
                //change state
                newState(State.STATE_STOP);
            }
        }
        else {
            // Display Diagnostic data for this state.
            logTelemetry("1", "Searching for sample");
        }
    }

    /**
     * Process the initial state of the FSM
     */
    private void processStateInitial() {
        //Check end-of-state condition
        if (IsStateTimedOut(currentState)) {
            //Start new action
            DetectSample();
            //change state
            newState(State.STATE_VISUAL_SEARCH);
        } else {
            // Display Diagnostic data for this state.
            logTelemetry("1", "Initializing");
        }
    }

    /**
     * Checks if a specified state has timed out.
     * @param state state of the FSM
     * @return true if timed out
     */
    private boolean IsStateTimedOut(State state) {
        int timeout = state.getTimeout();
        return this.stateTimer.milliseconds()>timeout;
    }

    /**
     * Sets the new state for the Finite State Machine engine in this class.
     * @param newState new state value
     */
    private void newState(State newState)
    {
        // Reset the state time, and then change to next state.
        stateTimer.reset();
        currentState = newState;
    }

    /**
     * Signals cancellation of searching
     */
    private void cancelSearch() {
        finished = true;
        log("LocateSampleCommand2: Command cancelled.");
    }

    /**
     * Moves the robot to the sample once the sample has been found
     */
    private void moveToSample() {
        //if (isRobotAtSampleLocation || isMovingToSample) return; // Prevent redundant calls
        if (isRobotAtSampleLocation) return; // Prevent redundant calls

        log("LocateSampleCommand2: Moving to sample at " + detectedSample.getTranslate());

        if (moveToSampleTargetPose == null) {

            //TO DO: Lukas, please double-check this swapping of X and Y
            //and pose calculation

            //Account for swapped X and Y with the Localizer by swapping X and Y
            double sampleX = detectedSample.getTranslate().y;
            double sampleY = detectedSample.getTranslate().x;

            moveToSampleTargetPose = new Pose(startPose.y + sampleY,
                    startPose.x - (TARGET_X - sampleX),
                    startPose.heading);

            stableTimer = new ElapsedTime();
        }

        trackMovementToSample();
    }

    /**
     * Continues moving the robot to the sample once the sample has been found
     */
    private void trackMovementToSample() {
        if (isRobotAtSampleLocation) return; // Prevent redundant calls

        log("LocateSampleCommand2: continuing moving to sample at " + detectedSample.getTranslate());

        Pose robotPose = robot.localizer.getPose();
        Pose power = getPower(robotPose, moveToSampleTargetPose);
        log("LocateSampleCommand2: setting drivetrain power " + power + " based on robotPose " + robotPose + ", moveToSampleTargetPose " + moveToSampleTargetPose);
        robot.drivetrain.set(power);

        // Check if we are close enough to the target
        Pose delta = moveToSampleTargetPose.subtract(robotPose);
        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR) {
            log("LocateSampleCommand2: trackMovementToSample() achieved stability (delta = " + delta + ")");
            stableTimer.reset();
        } else {
            log("LocateSampleCommand2: trackMovementToSample() still not at sample location. Check for decreasing delta in logs. If not decreasing, change vector. (delta = " + delta + ")");
        }

        if (delta.toVec2D().magnitude() <= ALLOWED_TRANSLATIONAL_ERROR && stableTimer.milliseconds() > stableTimerTimeout) {
            isRobotAtSampleLocation = true;
            log("LocateSampleCommand2: Arrived at sample location.");
        }
    }

    /**
     * Returns the difference between the MoveToSampleTargetPose and the current robot position
     * @return Pose object representing the difference
     */
    private Pose getMoveToSampleDelta() {
        Pose robotPose = robot.localizer.getPose();
        return moveToSampleTargetPose.subtract(robotPose);
    }

    /**
     * Calculates the wrist angle and attempts to grab the sample
     */
    private void pickUpSample() {
        if (hasPickedUpSample) return;

        log("LocateSampleCommand2: picking up sample");

        DetectionPipeline.AnalyzedSample prevSample = detectedSample;
        detectedSample = robot.pipeline.chooseClosestValidSample();

        double adjustedServoAngle = Range.clip(
                wristNeutralPos - (((detectedSample != null ? detectedSample.getAngle() : prevSample.getAngle()) - 90) / 300),
                wristMinPos,
                wristMaxPos
        );

        robot.intakeWristServo.setPosition(adjustedServoAngle != wristMinPos ? adjustedServoAngle : wristNeutralPos);

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new WaitCommand(250),
                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                new WaitCommand(150),
                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                new WaitCommand(325),
                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED),
                new InstantCommand(this::verifySamplePickup)
        ));

    }

    /**
     * Verify that the sample is picked up. Retry the pickup, if necessary.
     */
    private void verifySamplePickup() {
        //Check if the intake is fully closed.
        //If it is fully closed, that means we dropped the sample.
        //If it is not fully closed, that means we are holding the sample.

        //TO DO: Lukas, add the code to check if the intake is fully closed.
        boolean isIntakeFullyClosed = false;

        if (isIntakeFullyClosed) {
            //We need to retry the pickup.
            currentPickupAttemptNumber++;
            currentPickupAttemptFailed = true;
        } else {
            hasPickedUpSample = true;  // Mark that we have completed the pickup process
            log("LocateSampleCommand2: Sample pickup complete.");
        }
    }

    /**
     * Attempts to detect a sample using AI Vision within allowed time
     */
    private void DetectSample() {
        detectedSample = robot.pipeline.chooseClosestValidSample();
        log("LocateSampleCommand2: DetectSample: detectedSample is " + (detectedSample == null ? "not found" : "found"));
    }

    private void shiftRobotBack() {
        log("LocateSampleCommand2: Shifting the robot back...");

        if (stableTimer == null) {
            stableTimer = new ElapsedTime();
        }

        Pose robotPose = robot.localizer.getPose();
        Pose targetPose = new Pose(robotPose.x - backwardShiftDistance, robotPose.y, robotPose.heading);
        currentBackwardShiftOffset += backwardShiftDistance;

        robot.drivetrain.set(getPower(robotPose, targetPose));

        Pose delta = targetPose.subtract(robotPose);
        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR)
            stableTimer.reset();

        if (stableTimer.milliseconds() > stableTimerTimeout) {
            stableTimer = null;
            startPose = robot.localizer.getPose();
            log("LocateSampleCommand2: Shifted robot back for better sample detection.");
        }
    }

    /**
     * Initializes times and starts searching
     */
    private void initStartSearching() {
        startPose = robot.localizer.getPose();
        log("LocateSampleCommand2: start searching");
    }

    @Override
    public boolean isFinished() {
        return finished || cancelled;
    }

    @Override
    public void end(boolean interrupted) {
        //robot.setPipelineEnabled(robot.pipeline, false);
        if(cancelled) {
            robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED);
            robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);
        }
        robot.drivetrain.set(0, 0, 0, 0);
    }

    public void cancel() {
        cancelled = true;
    }

    public Pose getPower(Pose robotPose, Pose targetPose) {

        if(targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if(targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        if (Math.abs(x_rotated) < min_power) x_rotated = Math.signum(x_rotated) * min_power;
        if (Math.abs(y_rotated) < min_power) y_rotated = Math.signum(y_rotated) * min_power;
        if (Math.abs(hPower) < min_power_h) hPower = Math.signum(hPower) * min_power;

        // Feed forward to counteract friction
        x_rotated += Math.signum(x_rotated) * k_static; // counteract friction
        y_rotated += Math.signum(y_rotated) * k_static;
        hPower += Math.signum(hPower) * k_static;

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED/Y_GAIN, MAX_TRANSLATIONAL_SPEED/Y_GAIN);

        return new Pose(x_rotated * X_GAIN, y_rotated * Y_GAIN, hPower);
    }

    /**
     * Logs the passed-in text to console.
     * @param text to log
     */
    private void log(String text) {
        if (isLoggingEnabled) {
            System.out.println(text);
        }
    }

    /**
     * Logs a message to the Driving Station and to the log file.
     * @param caption caption for the Driving Station
     * @param text the logged message
     */
    private void logTelemetry(String caption, String text) {
        telemetry.addData(caption, text);
        log(text);
    }
}
