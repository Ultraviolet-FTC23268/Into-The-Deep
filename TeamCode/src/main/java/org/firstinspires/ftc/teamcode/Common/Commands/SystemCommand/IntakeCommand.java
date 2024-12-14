package org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem.IntakeState state) {
        super(
                () -> RobotHardware.getInstance().intake.update(state)
        );
    }
}
