package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;

public class ScoreCommand extends SequentialCommandGroup {

    LiftSubsystem lift = RobotHardware.getInstance().lift;
    DepositSubsystem deposit = RobotHardware.getInstance().deposit;
    IntakeSubsystem intake = RobotHardware.getInstance().intake;
    public ScoreCommand() {

            addCommands(new InstantCommand(() -> RobotHardware.getInstance().intakeRead = false),
                        new InstantCommand(() -> RobotHardware.getInstance().depositRead = false));

            if(deposit.depositState == DepositSubsystem.DepositState.SAMP_DEPOSIT)
                addCommands(
                        new dClawCommand(DepositSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED)
                );
            else if(deposit.depositState == DepositSubsystem.DepositState.SPEC_DEPOSIT)
                addCommands(
                        new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                        new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                        new WaitCommand(Globals.SPEC_SCORE_DELAY),
                        new dClawCommand(DepositSubsystem.ClawState.OPEN),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL)
                );
            else if(deposit.depositState == DepositSubsystem.DepositState.SPEC_INTAKE)
                addCommands(
                        new dClawCommand(DepositSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                        new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL)
                );
            else if(intake.intakeState == IntakeSubsystem.IntakeState.EXTENDED && intake.extendoState == IntakeSubsystem.ExtendoState.TRANSFER)
                addCommands(
                        new TransferCommand()
                );
            else if(intake.intakeState == IntakeSubsystem.IntakeState.EXTENDED && intake.extendoState == IntakeSubsystem.ExtendoState.RETRACTED)
                addCommands(
                        new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL)
                );
            else if(intake.intakeState == IntakeSubsystem.IntakeState.INTAKE)
                addCommands(
                        new PickUpCommand()
                );
            else
                addCommands(
                        new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                        new LiftCommand(LiftSubsystem.LiftState.RETRACTED)
                );
    }
}