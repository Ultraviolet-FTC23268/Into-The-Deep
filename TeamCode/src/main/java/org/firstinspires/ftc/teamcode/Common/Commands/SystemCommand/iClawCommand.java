package org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class iClawCommand extends InstantCommand {
    public iClawCommand(IntakeSubsystem.ClawState state) {
        super(
                () -> RobotHardware.getInstance().intake.update(state)
        );
    }
}
