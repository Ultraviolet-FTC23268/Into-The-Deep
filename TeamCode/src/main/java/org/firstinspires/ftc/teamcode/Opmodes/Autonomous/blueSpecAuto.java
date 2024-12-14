package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.HighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.PickUpCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
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
import java.util.List;


@Config
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

        robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);

        while (!isStarted()) {
            telemetry.addLine("auto in init");
        }

        robot.intake.update(IntakeSubsystem.IntakeState.NEUTRAL);

        Pose spec1ScorePos = new Pose(845, 50, 0);
        Pose spec2ScorePos = new Pose(845, 100, 0);
        Pose postScorePos = new Pose(400, 50, 0);
        Pose preIntakePos = new Pose(235, -750, 0);
        Pose intakePos = new Pose(30, -750, 0);

        Pose samp1Drop = new Pose (-450, 600, -Math.PI/2);
        Pose samp2Pick = new Pose (-480, 835, -0.75);
        Pose samp2Drop = new Pose (-480, 835, -Math.PI/2);
        Pose samp3Pick = new Pose (-515, 1025, -0.85);
        Pose samp3Drop = new Pose (-515, 1025, -Math.PI/2);

        ArrayList<Vector2D> pickUpPath = new ArrayList<>();
        pickUpPath.add(new Vector2D(350, 50));
        pickUpPath.add(new Vector2D(-450, 600));

        ArrayList<Vector2D> intakePath = new ArrayList<>();
        intakePath.add(new Vector2D(350, 50));
        intakePath.add(new Vector2D(235, -750));

        ArrayList<Vector2D> scorePath = new ArrayList<>();
        scorePath.add(new Vector2D(350, 50));
        scorePath.add(new Vector2D(845, 100));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PositionCommand(spec1ScorePos)
                                .alongWith(new SequentialCommandGroup(new WaitCommand(100),
                                                                      new ManualSpecOverrideCommand())),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),
                        new PositionCommand(postScorePos),
                        new PurePursuitCommand(pickUpPath, 350, -0.7),
                        new ExtendIntakeCommand(),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE)),
                        new PositionCommand(samp1Drop),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),

                        new PositionCommand(samp2Pick),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE)),
                        new PositionCommand(samp2Drop),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),

                        new PositionCommand(samp3Pick),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE)),
                        new PositionCommand(samp3Drop),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),

                        new PositionCommand(preIntakePos)
                                .alongWith(new RetractIntakeCommand()),
                        new HighSpecimenCommand()
                                .alongWith(new PositionCommand(intakePos)),
                        new PurePursuitCommand(scorePath, 350, 0),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY),
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED))

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