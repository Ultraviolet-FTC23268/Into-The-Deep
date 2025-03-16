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

import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Point;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
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
    private boolean shift = false;
    private boolean done = false;

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;
    public static double TARGET_X = 25;
    private double servoAngle = wristNeutralPos;

    private Pose startPose;

    public static PIDFController xController;
    public static PIDFController yController;
    public static PIDFController hController;
    public static double ALLOWED_TRANSLATIONAL_ERROR = 10;
    private ElapsedTime stable;
    private ElapsedTime kill;
    private ElapsedTime move;


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

        if (move == null) {
            move = new ElapsedTime();
            startPose = robot.localizer.getPose();
        }

        if(detectedSample == null && move.milliseconds() < 100) {
            detectedSample = robot.pipeline.chooseClosestValidSample();
        }

        else if(detectedSample == null && !shift) {
            if (stable == null) {
                stable = new ElapsedTime();
                kill = new ElapsedTime();
            }

            Pose robotPose = robot.localizer.getPose();
            Pose targetPose = new Pose(startPose.x - 300, startPose.y, startPose.heading);

            robot.drivetrain.set(getPower(robotPose, targetPose));

            Pose delta = targetPose.subtract(robotPose);
            if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR)
                stable.reset();

            if (stable.milliseconds() > 100 || kill.milliseconds() > 1250) {
                shift = true;
                stable = null;
                kill = null;
                move = null;
                startPose = robot.localizer.getPose();
            }
        }

        else if(detectedSample == null && shift && move.milliseconds() > 100) {
            cancel();
        }

        else if(detectedSample != null && !done) {

            if (stable == null) {
                stable = new ElapsedTime();
                kill = new ElapsedTime();
            }
            if (stable.milliseconds() > 100 || kill.milliseconds() > 2000) done = true;

            Pose robotPose = robot.localizer.getPose();
            Pose targetPose = new Pose(startPose.x - (TARGET_X - detectedSample.getTranslate().x), startPose.y + detectedSample.getTranslate().y, startPose.heading);

            robot.drivetrain.set(getPower(robotPose, targetPose));

            Pose delta = targetPose.subtract(robotPose);
            if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR)
                stable.reset();

        }

        else if(done) {

            DetectionPipeline.AnalyzedSample prevSample = detectedSample;
            detectedSample = robot.pipeline.chooseClosestValidSample();

            servoAngle = Range.clip(wristNeutralPos - (((detectedSample != null ? detectedSample.getAngle() : prevSample.getAngle()) - 90) / 300), wristMinPos, wristMaxPos);
            robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);

            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new WaitCommand(250),
                    new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                    new WaitCommand(150),
                    new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                    new WaitCommand(325),
                    new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                    new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)));

            finished = true;

        }

        if(cancelled)
            finished = true;

    }

    @Override
    public boolean isFinished() {
        return finished;
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

}
