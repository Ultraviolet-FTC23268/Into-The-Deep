package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand() {

        if(RobotHardware.getInstance().intake.intakeState == IntakeSubsystem.IntakeState.EXTENDED)
            addCommands(
                    new ExtendoCommand(IntakeSubsystem.ExtendoState.EXTENDED),
                    new WaitCommand(Globals.EXTEND_DELAY),
                    new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE)
            );

    }
}