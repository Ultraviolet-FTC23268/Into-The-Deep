package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoHighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.LocateSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionSpeedyCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitConstantCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;

import java.util.ArrayList;


@Config
@Autonomous(name = "\uD83D\uDD35⇾ 6 Specimen Auto")
public class SixSpecAuto extends CommandOpMode {

    //DO NOT CHANGE
    private final double defaultDistance = 940;
    private final double dOffset = Globals.subDistance - defaultDistance;
    private final double scorePosition = 625 + dOffset;

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

    public static Pose pick1Pos = new Pose(435, -405, -0.83);
    public static Pose drop1Pos = new Pose(430, -315, -1.96);

    public static Pose pick2Pos = new Pose(550, -540, -0.98);
    public static Pose drop2Pos = new Pose(500, -440, -2.01);

    public static Pose pick3Pos = new Pose(685, -735, -1.18);
    public static Pose drop3Pos = new Pose(580, -700, -2.11);

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Color.BLUE;
        Globals.AUTO = true;

        robot.init(hardwareMap);
        robot.enabled = true;

        robot.localizer.resetPosAndIMU();
        robot.localizer.setPosition(new Pose2D(DistanceUnit.MM,25, 0, AngleUnit.RADIANS, 0));

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
        robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED);

        Pose spec1ScorePos = new Pose(scorePosition+20, 225, 0);
        Pose spec2ScorePos = new Pose(scorePosition+30, 175, 0);
        Pose spec3ScorePos = new Pose(scorePosition+30, 125, 0);
        Pose spec4ScorePos = new Pose(scorePosition+30, 75, 0);
        Pose spec5ScorePos = new Pose(scorePosition+50, 25, 0);
        Pose spec6ScorePos = new Pose(scorePosition+50, -50, 0);

        Pose preDropOffPos = new Pose(300, 150, 0);
        Pose dropOffPos = new Pose(30, -700, -2.5);
        Pose preScorePos = new Pose(575 + dOffset, -50, 0);
        Pose preIntakePos = new Pose(235, -750, 0);
        Pose intakePos = new Pose(-100, -750, 0);
        Pose fIntakePos = new Pose(35, -750, 0);
        Pose parkPos = new Pose(100, -1000, Math.PI/2);

        Pose pick1Pos = new Pose(380, -450, -0.66);
        Pose drop1Pos = new Pose(430, -315, -1.96);

        Pose pick2Pos = new Pose(550, -570, -0.86);
        Pose drop2Pos = new Pose(500, -440, -2.07);

        Pose pick3Pos = new Pose(705, -755, -1.03);
        Pose drop3Pos = new Pose(580, -700, -2.11);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //Preload

                        new PositionCommand(spec1ScorePos)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new LiftCommand(LiftSubsystem.LiftState.PRE_HIGH_CHAMBER),
                                                new WaitCommand(250),
                                                new DepositCommand(DepositSubsystem.DepositState.SPEC_DEPOSIT),
                                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                                new WaitCommand(650),
                                                new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE))),
                        //new WaitCommand(250),

                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //new InstantCommand(() -> robot.startCamera()),

                        new dClawCommand(DepositSubsystem.ClawState.OPEN),
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                        new AutoHighSpecimenCommand(),

                        //Pick up sample

                        new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                        new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                        new WaitCommand(150),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY*2),
                        new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),

                        //new LocateSampleCommand(SampleType.Blue),
                        //Drop extra sample

                        //new InstantCommand(() -> robot.closeCamera()),

                        new ParallelRaceGroup(new WaitCommand(450),
                        new PositionSpeedyCommand(preDropOffPos))
                                .alongWith(new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)),
                        new PositionSpeedyCommand(dropOffPos),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),

                        //Move Samples

                        new PositionSpeedyCommand(pick1Pos)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                                new WaitCommand(900),
                                                new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                        new ParallelRaceGroup(
                                new WaitCommand(600),
                                new PositionSpeedyCommand(drop1Pos)),

                        new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                        new PositionSpeedyCommand(pick2Pos)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                                new WaitCommand(750),
                                                new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                        new ParallelRaceGroup(
                                new WaitCommand(600),
                                new PositionSpeedyCommand(drop2Pos)),

                        new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                        new PositionSpeedyCommand(pick3Pos)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                                new WaitCommand(750),
                                                new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                        new ParallelRaceGroup(
                                new WaitCommand(750),
                                new PositionSpeedyCommand(drop3Pos)),

                        /*//Sample 1
                        new InstantCommand(() -> robot.intake.changeWristPos(1)),
                        new PositionCommand(pick1Pos)
                                .alongWith(
                                        new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED)),
                        new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                        new WaitCommand(250),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new ParallelRaceGroup(
                                new WaitCommand(600),
                        new PositionSpeedyCommand(drop1Pos)),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),

                        //Sample 2
                        new InstantCommand(() -> robot.intake.changeWristPos(1)),
                        new PositionCommand(pick2Pos),
                        new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                        new WaitCommand(250),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new ParallelRaceGroup(
                                new WaitCommand(600),
                        new PositionSpeedyCommand(drop2Pos)),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),

                        //Sample 3
                        new InstantCommand(() -> robot.intake.changeWristPos(1)),
                        new PositionCommand(pick3Pos),
                        new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                        new WaitCommand(250),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new ParallelRaceGroup(
                                        new WaitCommand(600),
                        new PositionSpeedyCommand(drop3Pos)),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(150),*/

                        //Spec 2
                        new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED),
                        new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL),
                        new PositionCommand(fIntakePos),
                        new PositionCommand(fIntakePos),
                        new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new ParallelRaceGroup(
                                new WaitCommand(1100),
                        new PositionCommand(preScorePos))
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new ParallelRaceGroup(
                                new WaitCommand(350),
                        new PositionCommand(spec2ScorePos)),
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
                        new ParallelRaceGroup(
                                new WaitCommand(500),
                                new PositionCommand(intakePos)),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new ParallelRaceGroup(
                                new WaitCommand(1100),
                        new PositionCommand(preScorePos))
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new ParallelRaceGroup(
                                new WaitCommand(350),
                        new PositionCommand(spec3ScorePos)),
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
                        new ParallelRaceGroup(
                                new WaitCommand(500),
                                new PositionCommand(intakePos)),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new ParallelRaceGroup(
                                new WaitCommand(1100),
                        new PositionCommand(preScorePos))
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new ParallelRaceGroup(
                                new WaitCommand(350),
                        new PositionCommand(spec4ScorePos)),
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
                        new ParallelRaceGroup(
                                new WaitCommand(500),
                                new PositionCommand(intakePos)),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new ParallelRaceGroup(
                                new WaitCommand(1100),
                        new PositionCommand(preScorePos))
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new ParallelRaceGroup(
                                new WaitCommand(350),
                        new PositionCommand(spec5ScorePos)),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Spec 6

                        new PositionCommand(preIntakePos)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                        new AutoHighSpecimenCommand())),

                        new ParallelRaceGroup(
                                new WaitCommand(500),
                                new PositionCommand(intakePos)),
                        new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new ParallelRaceGroup(
                                new WaitCommand(1100),
                        new PositionCommand(preScorePos))
                                .alongWith(new AutoManualSpecOverrideCommand()),
                        new ParallelRaceGroup(
                                new WaitCommand(350),
                        new PositionCommand(spec6ScorePos)),
                        new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                                new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                        //Park

                        new PositionCommand(parkPos)
                                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED)))
                                        //new InstantCommand(() -> RobotHardware.getInstance().intakeArmServo.setPosition(IntakeSubsystem.iArmNeutralPos+.1)),
                                        //new InstantCommand(() -> RobotHardware.getInstance().intakeArmServo.setPosition(IntakeSubsystem.iArmNeutralPos+.1))))


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
        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

    }
}