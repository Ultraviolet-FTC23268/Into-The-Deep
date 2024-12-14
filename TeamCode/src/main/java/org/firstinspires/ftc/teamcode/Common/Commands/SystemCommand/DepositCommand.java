package org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class DepositCommand extends InstantCommand {
    public DepositCommand(DepositSubsystem.DepositState state) {
        super(
                () -> RobotHardware.getInstance().deposit.update(state)
        );
    }
}
