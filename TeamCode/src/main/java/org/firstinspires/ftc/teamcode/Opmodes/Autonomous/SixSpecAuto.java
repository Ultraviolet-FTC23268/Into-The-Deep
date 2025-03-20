package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoHighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.LocateSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionSpeedyCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;


@Config
@Autonomous(name = "\uD83D\uDD35⇾ 6 Specimen Auto")
public class SixSpecAuto extends CommandOpMode {

    public boolean isLoggingEnabled = true;

    //DO NOT CHANGE
    private final double defaultDistance = 940;
    private final double dOffset = Globals.subDistance - defaultDistance;
    private final double scorePosition = 625 + dOffset;

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

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

        Pose sweep1Pos = new Pose(380, -450, -0.66);
        Pose drop1Pos = new Pose(430, -315, -1.96);

        Pose sweep2Pos = new Pose(550, -570, -0.86);
        Pose drop2Pos = new Pose(500, -440, -2.07);

        Pose sweep3Pos = new Pose(705, -755, -1.03);
        Pose drop3Pos = new Pose(580, -700, -2.11);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        cmdStartCamera(),
                        cmdGoToStartPositionAndPrepareLiftAndIntake(spec1ScorePos),
                        cmdScoreSpecimen1(),
                        cmdLocateAndGrabExtraSample(SampleType.Blue, telemetry),
                        cmdStopCamera(),
                        cmdDropOffExtraSample(preDropOffPos, dropOffPos),
                        cmdSweepFloorSamplesToOperator(sweep1Pos, drop1Pos, sweep2Pos, drop2Pos, sweep3Pos, drop3Pos),
                        cmdRetractIntake(),
                        cmdScoreSpecimen2(preScorePos, spec2ScorePos, fIntakePos),
                        cmdScoreSpecimen3456(3, preScorePos, spec3ScorePos, preIntakePos, intakePos),
                        cmdScoreSpecimen3456(4, preScorePos, spec4ScorePos, preIntakePos, intakePos),
                        cmdScoreSpecimen3456(5, preScorePos, spec5ScorePos, preIntakePos, intakePos),
                        cmdScoreSpecimen3456(6, preScorePos, spec6ScorePos, preIntakePos, intakePos),
                        cmdPark(parkPos)
                )
        );
    }

    /**
     * Parks the robot at the designated parking area
     * @param parkPos parking position
     * @return scheduler command
     */
    private Command cmdPark(Pose parkPos) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdPark with parkPos " + parkPos)),
                new PositionCommand(parkPos)
                .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED)))
        );
    }

    /**
     * Scores the Nth specimen on the submersible (3,4,5,6)
     * @param preScorePos preliminary position to move the robot to for scoring
     * @param scorePos final position for scoring
     * @param preIntakePos preliminary intake position
     * @param intakePos final intake position
     * @return scheduler command
     */
    private Command cmdScoreSpecimen3456(int specimenIndex, Pose preScorePos, Pose scorePos, Pose preIntakePos, Pose intakePos) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdScoreSpecimen3456 (specimen " + specimenIndex + ") with preScorePos " + preScorePos + ", scorePos " + scorePos + ", preIntakePos " + preIntakePos + ", intakePos " + intakePos)),
                //Spec 3
                new PositionCommand(preIntakePos)
                        .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                new AutoHighSpecimenCommand())),
                new PositionCommand(intakePos, 500),
                cmdScoreSpecimen(preScorePos, scorePos)
        );
    }

    /**
     * Scores the second specimen on the submersible
     * @param preScorePos preliminary position to move the robot to for scoring
     * @param scorePos final position for scoring
     * @param intakePos intake position
     * @return scheduler command
     */
    private Command cmdScoreSpecimen2(Pose preScorePos, Pose scorePos, Pose intakePos) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdScoreSpecimen2 with preScorePos " + preScorePos + ", scorePos " + scorePos + ", intakePos " + intakePos)),
                //Spec 2
                new PositionCommand(intakePos, 2000),
                cmdScoreSpecimen(preScorePos, scorePos)
        );
    }

    /**
     * Scores a specimen using preScorePos position and scorePos position
     * @param preScorePos preliminary position to move the robot to
     * @param scorePos final position to move the robot to for scoring
     * @return scheduler command
     */
    private Command cmdScoreSpecimen(Pose preScorePos, Pose scorePos) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing generic cmdScoreSpecimen with preScorePos " + preScorePos + ", scorePos " + scorePos)),
                new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                new PositionCommand(preScorePos, 1100)
                        .alongWith(new AutoManualSpecOverrideCommand()),
                new PositionCommand(scorePos,350),
                new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                        new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                        new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                        new WaitCommand(Globals.SPEC_SCORE_DELAY))
        );
    }

    /**
     * Retracts intake and puts it in neutral position
     * @return scheduler command
     */
    private Command cmdRetractIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdRetractIntake")),
                new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED),
                new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL)
        );
    }

    /**
     * Move the three floor samples to the operator with the sweeping motion.
     * @param pick1Pos start of the sweep position for sample 1
     * @param drop1Pos end of the sweep position for sample 1
     * @param pick2Pos start of the sweep position for sample 2
     * @param drop2Pos end of the sweep position for sample 2
     * @param pick3Pos start of the sweep position for sample 3
     * @param drop3Pos end of the sweep position for sample 3
     * @return scheduler command
     */
    private Command cmdSweepFloorSamplesToOperator(Pose pick1Pos, Pose drop1Pos, Pose pick2Pos, Pose drop2Pos, Pose pick3Pos, Pose drop3Pos) {
        return new SequentialCommandGroup(
                //Move Samples
                new InstantCommand(() -> log("Executing cmdSweepFloorSamplesToOperator... Sweeping floor sample 1")),
                new PositionSpeedyCommand(pick1Pos)
                .alongWith(
                        new SequentialCommandGroup(
                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                new WaitCommand(900),
                                new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                new PositionSpeedyCommand(drop1Pos, 600),

                new InstantCommand(() -> log("Sweeping floor sample 2")),
                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                new PositionSpeedyCommand(pick2Pos)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                        new WaitCommand(750),
                                        new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                new PositionSpeedyCommand(drop2Pos, 600),

                new InstantCommand(() -> log("Sweeping floor sample 3")),
                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                new PositionSpeedyCommand(pick3Pos)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                        new WaitCommand(750),
                                        new IntakeCommand(IntakeSubsystem.IntakeState.SWEEP))),
                new PositionSpeedyCommand(drop3Pos, 750)
        );
    }

    /**
     * Drops off the extra sample with the operator.
     * @param preDropOffPos pre-drop-off position
     * @param dropOffPos drop-off position
     * @return scheduler command
     */
    private Command cmdDropOffExtraSample(Pose preDropOffPos, Pose dropOffPos) {
        //Drop extra sample
        int preDropOffMovementTimeout = 450;
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdDropOffExtraSample with preDropOffPos: " + preDropOffPos + ", dropOffPos: " + dropOffPos)),
                new PositionSpeedyCommand(preDropOffPos, preDropOffMovementTimeout)
                    .alongWith(new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)),
                new PositionSpeedyCommand(dropOffPos),
                new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED)
        );
    }


    /**
     * Finds and grabs the extra sample for the 6th specimen by the front intake.
     * @param sampleType specify the color of the sample we are looking for.
     * @return scheduler command
     */
    private Command cmdLocateAndGrabExtraSample(SampleType sampleType, Telemetry telemetry) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdLocateAndGrabExtraSample")),
                new LocateSampleCommand(sampleType, telemetry)
        );
    }

    /**
     * Scores the first specimen
     * @return scheduler command
     */
    private Command cmdScoreSpecimen1() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdScoreSpecimen1")),
                new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                new WaitCommand(Globals.SPEC_SCORE_DELAY));
    }

    /**
     * Puts the robot at the start position ready to score specimen 1 with the lift and intake prepared for action.
     * @param spec1ScorePos start position
     * @return Command for scheduler
     */
    private Command cmdGoToStartPositionAndPrepareLiftAndIntake(Pose spec1ScorePos) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> log("Executing cmdGoToStartPositionAndPrepareLiftAndIntake with Pose: " + spec1ScorePos)),
                new PositionCommand(spec1ScorePos)
                .alongWith(
                        new SequentialCommandGroup(
                                new LiftCommand(LiftSubsystem.LiftState.PRE_HIGH_CHAMBER),
                                new WaitCommand(250),
                                new DepositCommand(DepositSubsystem.DepositState.SPEC_DEPOSIT),
                                new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                                new WaitCommand(650),
                                new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE)))
        );
    }

    /**
     * Starts the camera
     * @return An InstantCommand that starts the camera.
     */
    private Command cmdStartCamera() {
        return new InstantCommand(() -> {
            log("Starting camera...");
            robot.startCamera();
        });
    }
    /**
     * Stops the camera
     * @return An InstantCommand that stops the camera.
     */
    private Command cmdStopCamera() {
        return new InstantCommand(() -> {
            log("Stopping camera...");
            robot.closeCamera();
        });
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