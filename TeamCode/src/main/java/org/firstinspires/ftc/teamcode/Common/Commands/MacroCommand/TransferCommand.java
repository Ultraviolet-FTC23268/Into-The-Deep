package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand() {

        //if(RobotHardware.getInstance().intake.sampleDetected())
            addCommands(
                    new InstantCommand(() -> RobotHardware.getInstance().intakeRead = true),
                    new InstantCommand(() -> RobotHardware.getInstance().depositRead = true),
                    new DepositCommand(DepositSubsystem.DepositState.TRANSFER),
                    new WaitCommand(Globals.TSETUP_DELAY),
                    new IntakeCommand(IntakeSubsystem.IntakeState.PRE_TRANSFER),
                    new WaitCommand(Globals.TSWAP_DELAY),
                    new IntakeCommand(IntakeSubsystem.IntakeState.TRANSFER),
                    new WaitCommand(Globals.TPLACE_DELAY),
                    //new WaitUntilCommand(RobotHardware.getInstance().deposit::sampleDetected),
                    new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                    new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                    new WaitCommand(Globals.CLAW_MOVE_DELAY),
                    new IntakeCommand(IntakeSubsystem.IntakeState.POST_TRANSFER),
                    new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                    new WaitCommand(Globals.PULL_OUT_DELAY),
                    new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL),
                    new InstantCommand(() -> RobotHardware.getInstance().intakeRead = false),
                    new InstantCommand(() -> RobotHardware.getInstance().depositRead = false)
            );

    }
}