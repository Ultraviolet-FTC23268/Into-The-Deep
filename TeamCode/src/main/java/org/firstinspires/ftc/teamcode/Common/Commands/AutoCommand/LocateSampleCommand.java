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

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.DetectionPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;

@Config
public class LocateSampleCommand extends CommandBase {

    private RobotHardware robot = RobotHardware.getInstance();
    private DetectionPipeline.AnalyzedSample detectedSample = null;

    public double slideExtendedPos = 0.4;
    public boolean finished = false;
    private boolean cancelled = false;
    private boolean isShifted = false;
    private boolean isRobotAtSampleLocation = false;
    private boolean isLoggingEnabled = true;

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;
    public static double TARGET_X = 25;
    private double servoAngle = wristNeutralPos;

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
     * action timer for stopping action when it takes too long to execute
     */
    private ElapsedTime killTimer;

    /**
     * Allowed time for an individual action. Used to be 1250. Need to re-check this.
     */
    private final int killTimerTimeout = 2000;

    /**
     * movement timer
     */

    private ElapsedTime moveTimer;

    /**
     * Allowed time for searching for the extra sample in the submersible
     */
    private final int searchMovementTimeout = 2000;

    /**
     * Overall duration of process allowed before the action is cancelled
     */
    private final int cancellationTimeout = 3000;

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
     * Indicates that the robot is moving towards the sample
     */
    private boolean isMovingToSample = false;

    /**
     * Indicates that the robot is picking up the sample
     */
    private boolean isPickingUpSample = false;

    /**
     * Indicates that the robot has picked up the sample
     */
    private boolean hasPickedUpSample = false;

    // Target pose for moving to the sample (set once, used across execute() calls)
    private Pose moveToSampleTargetPose = null;

    public LocateSampleCommand(SampleType sampleType) {
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
    }

    @Override
    public void execute() {
        if (cancelled) {
            cancelSearch();
            return;
        }

        if (moveTimer == null) initTimersAndStartSearching();

        if (detectedSample == null) {
            DetectSampleWithinTimeout();
            if (cancelled) {
                cancelSearch();
                return;
            }
        }
        else {
            //Sample found
            if (isRobotAtSampleLocation) pickUpSample();
            else moveToSample();
        }
    }

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
            /*
            moveToSampleTargetPose = new Pose(startPose.x - (TARGET_X - detectedSample.getTranslate().x),
                    startPose.y + detectedSample.getTranslate().y,
                    startPose.heading);
            */
            //Account for swapped X and Y with the localizer by inverting X and Y
            moveToSampleTargetPose = new Pose(startPose.y + detectedSample.getTranslate().y,
                    startPose.x - (TARGET_X - detectedSample.getTranslate().x),
                    startPose.heading);

            stableTimer = new ElapsedTime();
            killTimer = new ElapsedTime();

            // Mark as moving
            isMovingToSample = true;
        }

        Pose robotPose = robot.localizer.getPose();
        robot.drivetrain.set(getPower(robotPose, moveToSampleTargetPose));

        // Check if we are close enough to the target
        Pose delta = moveToSampleTargetPose.subtract(robotPose);
        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR)
            stableTimer.reset();

        if (delta.toVec2D().magnitude() <= ALLOWED_TRANSLATIONAL_ERROR && stableTimer.milliseconds() > stableTimerTimeout) {
            isRobotAtSampleLocation = true;
            isMovingToSample = false;
            log("LocateSampleCommand2: Arrived at sample location.");
        }


    }

    /**
     * Calculates the wrist angle and attempts to grab the sample
     */
    private void pickUpSample() {
        if (isPickingUpSample || hasPickedUpSample) return;

        log("LocateSampleCommand2: picking up sample");
        isPickingUpSample = true;

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
                new InstantCommand(() -> {
                    hasPickedUpSample = true;  // Mark that we have completed the pickup process
                    isPickingUpSample = false; // Reset flag
                    log("LocateSampleCommand2: Sample pickup complete.");
                })
        ));

    }

    /**
     * Attempts to detect a sample using AI Vision within allowed time
     */
    private void DetectSampleWithinTimeout() {
        if (moveTimer.milliseconds() < searchMovementTimeout) {
            detectedSample = robot.pipeline.chooseClosestValidSample();
            log("LocateSampleCommand2: detectedSample is " + (detectedSample == null ? "not found" : "found"));
        } else if (!isShifted) {
            // If the sample is still not found and timeout has expired, shift position
            shiftRobotBack();
        } else {
            // If shifting was performed but still no sample, cancel the command
            cancel();
            log("LocateSampleCommand2: Sample not found after shifting, cancelling...");
        }
    }

    private void shiftRobotBack() {
        log("LocateSampleCommand2: Shifting the robot back...");

        if (stableTimer == null) {
            stableTimer = new ElapsedTime();
            killTimer = new ElapsedTime();
        }

        Pose robotPose = robot.localizer.getPose();
        Pose targetPose = new Pose(robotPose.x - backwardShiftDistance, robotPose.y, robotPose.heading);
        currentBackwardShiftOffset += backwardShiftDistance;

        robot.drivetrain.set(getPower(robotPose, targetPose));

        Pose delta = targetPose.subtract(robotPose);
        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR)
            stableTimer.reset();

        if (stableTimer.milliseconds() > stableTimerTimeout || killTimer.milliseconds() > killTimerTimeout) {
            isShifted = true;
            stableTimer = null;
            killTimer = null;
            moveTimer = null;
            startPose = robot.localizer.getPose();
            log("LocateSampleCommand2: Shifted robot back for better sample detection.");
        }

        if (currentBackwardShiftOffset>maxBackwardShiftDistance) {
            //We haven't found the sample and reached the end of the allowed travel path,
            //so we need to abandon the search
            cancel();
        }
    }

    /**
     * Initializes times and starts searching
     */
    private void initTimersAndStartSearching() {
        moveTimer = new ElapsedTime();
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
}
