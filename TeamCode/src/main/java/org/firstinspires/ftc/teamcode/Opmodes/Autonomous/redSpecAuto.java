package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitConstantCommand;
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

import java.util.ArrayList;


@Config
@Autonomous(name = "\uD83D\uDD34⇾ Red Specimen Auto")
public class redSpecAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Color.RED;

        robot.init(hardwareMap);
        robot.enabled = true;

        robot.localizer.setPosition(new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.RADIANS, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.deposit.update(DepositSubsystem.DepositState.AUTO);
        robot.deposit.update(DepositSubsystem.ClawState.CLOSED);

        while (!isStarted()) {
            telemetry.addLine("auto in init");
        }
        robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);
        robot.intake.update(IntakeSubsystem.IntakeState.NEUTRAL);

        Pose spec1ScorePos = new Pose(855, 100, 0);
        Pose spec2ScorePos = new Pose(845, 100, 0);
        Pose postScorePos = new Pose(400, 50, 0);
        Pose preIntakePos = new Pose(235, -750, 0);
        Pose intakePos = new Pose(20, -750, 0);
        Pose parkPose = new Pose(200, 250 ,0);

        Pose samp1Drop = new Pose (450, -600, -Math.PI/2);
        Pose samp2Pick = new Pose (480, -835, -0.75);
        Pose samp2Drop = new Pose (480, -835, -Math.PI/2);
        Pose samp3Pick = new Pose (515, -1025, -0.85);
        Pose samp3Drop = new Pose (515, -1025, -Math.PI/2);

        ArrayList<Vector2D> pickUp1Path = new ArrayList<>();
        pickUp1Path.add(new Vector2D(350, 50));
        pickUp1Path.add(new Vector2D(475, -675));
        pickUp1Path.add(new Vector2D(1400, -675));
        pickUp1Path.add(new Vector2D(1400, -1000));
        pickUp1Path.add(new Vector2D(150, -1000));
        pickUp1Path.add(new Vector2D(235, -750));

        ArrayList<Vector2D> pickUp2Path = new ArrayList<>();
        pickUp2Path.add(new Vector2D(350, 50));
        pickUp2Path.add(new Vector2D(475, -750));
        pickUp2Path.add(new Vector2D(1400, -900));
        pickUp2Path.add(new Vector2D(1400, -1240));
        pickUp2Path.add(new Vector2D(150, -1240));
        pickUp2Path.add(new Vector2D(235, -750));

        ArrayList<Vector2D> pickUp3Path = new ArrayList<>();
        pickUp3Path.add(new Vector2D(350, 50));
        pickUp3Path.add(new Vector2D(235, -750));
        /*


        pickUpPath.add(new Vector2D(1345, -1420));
        pickUpPath.add(new Vector2D(200, -1420));
        pickUpPath.add(new Vector2D(235, -750));
*/
        ArrayList<Vector2D> parkPath = new ArrayList<>();
        parkPath.add(new Vector2D(350, 50));
        parkPath.add(new Vector2D(235, -750));

        ArrayList<Vector2D> scorePath = new ArrayList<>();
        scorePath.add(new Vector2D(350, 100));
        scorePath.add(new Vector2D(845, 150));

        ArrayList<Vector2D> score2Path = new ArrayList<>();
        score2Path.add(new Vector2D(350, 150));
        score2Path.add(new Vector2D(845, 200));

        ArrayList<Vector2D> score3Path = new ArrayList<>();
        score3Path.add(new Vector2D(350, 200));
        score3Path.add(new Vector2D(845, 250));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PositionCommand(spec1ScorePos)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(100),
                                                                      new ManualSpecOverrideCommand())),
                        new WaitCommand(250),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),


                        new PurePursuitConstantCommand(pickUp1Path, 350, 0)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(800), new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL))),

                        new HighSpecimenCommand()
                                .alongWith(new PositionCommand(intakePos)),
                        new PurePursuitCommand(scorePath, 350, 0),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        new PurePursuitConstantCommand(pickUp2Path, 350, 0)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(800), new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL))),

                        new HighSpecimenCommand()
                                .alongWith(new PositionCommand(intakePos)),
                        new PurePursuitCommand(score2Path, 350, 0),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        new PurePursuitConstantCommand(pickUp3Path, 350, 0)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(800), new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL))),

                        new HighSpecimenCommand()
                                .alongWith(new PositionCommand(intakePos)),
                        new PurePursuitCommand(score3Path, 350, 0),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        new PositionCommand(parkPose)


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