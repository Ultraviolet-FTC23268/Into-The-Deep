package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class ManualSpecOverrideCommand extends SequentialCommandGroup {

    public ManualSpecOverrideCommand() {

        super(
            new dClawCommand(DepositSubsystem.ClawState.CLOSED),
            new WaitCommand(Globals.CLAW_MOVE_DELAY),
            new LiftCommand(LiftSubsystem.LiftState.PRE_HIGH_CHAMBER),
            new DepositCommand(DepositSubsystem.DepositState.SPEC_DEPOSIT),
            new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
            new InstantCommand(() -> RobotHardware.getInstance().depositRead = false)
        );

    }
}