package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;

public class HighSampleCommand extends SequentialCommandGroup {
    public HighSampleCommand() {

        super (
            new LiftCommand(LiftSubsystem.LiftState.HIGH_BUCKET),
            new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT)
        );

    }
}