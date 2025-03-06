package org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
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

public class LocateSampleCommand extends CommandBase {

    private RobotHardware robot = RobotHardware.getInstance();
    private DetectionPipeline.AnalyzedSample detectedSample;

    public static double slideExtendedPos = 0.4;
    public boolean finished = false;

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;
    private double servoAngle = wristNeutralPos;

    private PositionCommand POSITION_COMMAND;


    public LocateSampleCommand(SampleType sampleType) {
        robot.pipeline.sampleType = sampleType;
        robot.setPipelineEnabled(robot.pipeline, true);
    }

    @Override
    public void execute() {

        while(detectedSample == null) {
            slideExtendedPos += -0.0001;
            slideExtendedPos = Range.clip(slideExtendedPos, 0.2, 0.4);
            robot.slideLeftServo.setPosition(slideExtendedPos);
            detectedSample = robot.pipeline.chooseClosestValidSample();
        }

        servoAngle = Range.clip(wristNeutralPos - ((detectedSample.getAngle() - 90) / 300), wristMinPos, wristMaxPos);
        robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);

        Pose robotPose = robot.localizer.getPose();
        Point sample = detectedSample.getTranslate();
        Pose target = new Pose(robotPose.x + (sample.x - 65), robotPose.y + sample.y, robotPose.heading);

        POSITION_COMMAND = new PositionCommand(target);
        CommandScheduler.getInstance().schedule(POSITION_COMMAND);
        CommandScheduler.getInstance().schedule(new WaitCommand(150));
        CommandScheduler.getInstance().schedule(new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP));
        CommandScheduler.getInstance().schedule(new WaitCommand(150));
        CommandScheduler.getInstance().schedule(new iClawCommand(IntakeSubsystem.ClawState.CLOSED));
        CommandScheduler.getInstance().schedule(new WaitCommand(325));
        CommandScheduler.getInstance().schedule(new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE));
        CommandScheduler.getInstance().schedule(new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED));

        finished = true;

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        robot.setPipelineEnabled(robot.pipeline, false);
    }

}
