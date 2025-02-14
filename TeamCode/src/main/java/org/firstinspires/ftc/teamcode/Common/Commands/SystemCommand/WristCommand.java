package org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class WristCommand extends InstantCommand {
    public WristCommand(IntakeSubsystem.WristState state) {
        super(
                () -> RobotHardware.getInstance().intake.update(state)
        );
    }
}
