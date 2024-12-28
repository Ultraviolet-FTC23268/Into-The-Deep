package org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class AutoHighSpecimenCommand extends SequentialCommandGroup {

    public AutoHighSpecimenCommand() {

            addCommands(
                new DepositCommand(DepositSubsystem.DepositState.SPEC_INTAKE),
                    new WaitCommand(250),
                    new IntakeCommand(IntakeSubsystem.IntakeState.NEUTRAL)
            );

    }
}