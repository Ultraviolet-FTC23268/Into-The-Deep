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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.TransferCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
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

import java.util.ArrayList;


//@Config
@Autonomous(name = "\uD83D\uDD35⇽ Sample Auto")
public class BlueSampleAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Color.BLUE;
        Globals.AUTO = true;

        robot.init(hardwareMap);
        robot.enabled = true;

        robot.localizer.setPosition(new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.RADIANS, -Math.PI/2));

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
        robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);
        robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED);
        robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);

        Pose scorePos = new Pose(140, 590, -Math.PI/4);
        Pose samp2Pos = new Pose(600, 390, 0);
        Pose samp3Pos = new Pose(600, 650, 0);
        Pose samp4Pos = new Pose(660, 690, 0.53);

        ArrayList<Vector2D> parkPath = new ArrayList<>();
        parkPath.add(new Vector2D(1500, 500));
        parkPath.add(new Vector2D(1350, -240));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //Preload
                        new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
                        new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT),
                        new PositionCommand(new Pose(250, 500, -Math.PI/4)),
                        new PositionCommand(scorePos),

                        new SequentialCommandGroup( //manual score because this thing might be stupid and dumb
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        //Sample 2
                        new InstantCommand(() -> robot.intake.slideExtendedPos = 0),
                        new PositionCommand(samp2Pos)
                                .alongWith(new ExtendIntakeCommand()),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new WaitCommand(350),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY*2),
                                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                                new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)),
                        new TransferCommand(),

                        new SequentialCommandGroup(
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
                                new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT)),
                        new WaitCommand(250),

                        new PositionCommand(scorePos)
                                .alongWith(new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED)),

                        new SequentialCommandGroup( //manual score because this thing might be stupid and dumb
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        //Sample 3
                        new InstantCommand(() -> robot.intake.slideExtendedPos = 0.2),
                        new PositionCommand(samp3Pos)
                                .alongWith(new ExtendIntakeCommand()),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new WaitCommand(350),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY*2),
                                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                                new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)),
                        new TransferCommand(),

                        new SequentialCommandGroup(
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
                                new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT)),
                        new WaitCommand(250),

                        new PositionCommand(scorePos)
                                .alongWith(new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED)),

                        new SequentialCommandGroup( //manual score because this thing might be stupid and dumb
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        //Sample 4
                        new InstantCommand(() -> robot.intake.slideExtendedPos = 0.2),
                        new PositionCommand(samp4Pos)
                                .alongWith(new ExtendIntakeCommand()),
                        new InstantCommand(() -> robot.intake.changeWristPos(-1)),
                        new SequentialCommandGroup(
                                new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                                new WaitCommand(350),
                                new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY*2),
                                new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                                new ExtendoCommand(IntakeSubsystem.ExtendoState.RETRACTED)),
                        new TransferCommand(),
                        new InstantCommand(() -> robot.intake.servoAngle = IntakeSubsystem.wristNeutralPos),

                        new SequentialCommandGroup(
                                new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
                                new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT)),
                        new WaitCommand(250),

                        new PositionCommand(scorePos),

                        new SequentialCommandGroup( //manual score because this thing might be stupid and dumb
                                new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED)),

                        new InstantCommand(() -> robot.intake.slideExtendedPos = 1),

                        //Park
                        new PurePursuitCommand(parkPath, 500, Math.PI/2)

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