package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

public class ClimbUpCommand extends SequentialCommandGroup {
    public ClimbUpCommand() {

        super (
                new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL)
        );

    }
}