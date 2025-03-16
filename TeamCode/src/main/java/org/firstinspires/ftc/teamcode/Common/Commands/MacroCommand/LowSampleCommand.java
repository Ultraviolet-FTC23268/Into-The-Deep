package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

public class LowSampleCommand extends SequentialCommandGroup {
    public LowSampleCommand() {

        super (
            new LiftCommand(LiftSubsystem.LiftState.LOW_BUCKET),
            new DepositCommand(DepositSubsystem.DepositState.SAMP_DEPOSIT)
        );

    }
}