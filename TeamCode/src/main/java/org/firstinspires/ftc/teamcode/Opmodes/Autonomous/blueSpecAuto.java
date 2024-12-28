package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitConstantCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoHighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.HighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;


//@Config
@Autonomous(name = "\uD83D\uDD35⇾ Blue Specimen Auto")
public class blueSpecAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Color.BLUE;

        robot.init(hardwareMap);
        robot.enabled = true;

        robot.localizer.setPosition(new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.RADIANS, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.deposit.update(DepositSubsystem.DepositState.AUTO);
        robot.deposit.update(DepositSubsystem.ClawState.CLOSED);

        while (opModeInInit()) {
            try {
                Thread.sleep(50L);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(isStopRequested())
                break;
        }
        //robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);
        robot.intake.update(IntakeSubsystem.IntakeState.NEUTRAL);

        Pose spec1ScorePos = new Pose(660, -50, 0);
        Pose spec2ScorePos = new Pose(675, 150, 0);
        Pose spec3ScorePos = new Pose(675, 100, 0);
        Pose spec4ScorePos = new Pose(675, 50, 0);
        Pose spec5ScorePos = new Pose(675, 0, 0);

        Pose postScorePos = new Pose(575, -675, 0);
        Pose preScorePos = new Pose(575, -50, 0);
        Pose preIntakePos = new Pose(235, -750, 0);
        Pose intakePos = new Pose(80, -750, 0);
        Pose parkPose = new Pose(200, 250 ,0);

        ArrayList<Vector2D> pushPath = new ArrayList<>();
        pushPath.add(new Vector2D(575, -850));
        pushPath.add(new Vector2D(1650, -675));
        pushPath.add(new Vector2D(1650, -1000));
        pushPath.add(new Vector2D(-50, -1000));
        pushPath.add(new Vector2D(1650, -1000));
        pushPath.add(new Vector2D(1650, -1240));
        pushPath.add(new Vector2D(-50, -1240));
        pushPath.add(new Vector2D(1650, -1240));
        pushPath.add(new Vector2D(1650, -1400));
        pushPath.add(new Vector2D(-50, -1400));
        pushPath.add(new Vector2D(235, -750));

        ArrayList<Vector2D> parkPath = new ArrayList<>();
        parkPath.add(new Vector2D(350, 50));
        parkPath.add(new Vector2D(175, -750));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //Preload

                        new PositionCommand(spec1ScorePos)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(350),
                                                                      new AutoManualSpecOverrideCommand())),
                        new WaitCommand(250),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Push Samples

                        new PurePursuitConstantCommand(pushPath, 500, 0)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED))),

                        //Spec 2

                        new AutoHighSpecimenCommand()
                                .alongWith(new PositionCommand(intakePos)),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new PositionCommand(preScorePos)
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new PositionCommand(spec2ScorePos),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Spec 3

                        new PositionCommand(preIntakePos)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                           new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                           new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                           new AutoHighSpecimenCommand())),

                        new PositionCommand(intakePos),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new PositionCommand(preScorePos)
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new PositionCommand(spec3ScorePos),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Spec 4

                        new PositionCommand(preIntakePos)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                        new AutoHighSpecimenCommand())),

                        new PositionCommand(intakePos),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new PositionCommand(preScorePos)
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new PositionCommand(spec4ScorePos),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Spec 5

                        new PositionCommand(preIntakePos)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                        new AutoHighSpecimenCommand())),

                        new PositionCommand(intakePos),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new PositionCommand(preScorePos)
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new PositionCommand(spec5ScorePos),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Park

                        new PurePursuitCommand(parkPath, 500, Math.PI/2)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED)))


                )
        );
    }

    @Override
    public void run() {

        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}