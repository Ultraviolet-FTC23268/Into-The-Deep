package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class ReadyIntakeCommand extends SequentialCommandGroup {
    public ReadyIntakeCommand() {

        if(RobotHardware.getInstance().intake.intakeState == IntakeSubsystem.IntakeState.NEUTRAL)
            addCommands(new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED));
        else if(RobotHardware.getInstance().intake.intakeState == IntakeSubsystem.IntakeState.EXTENDED)
            addCommands(new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL));

    }
}