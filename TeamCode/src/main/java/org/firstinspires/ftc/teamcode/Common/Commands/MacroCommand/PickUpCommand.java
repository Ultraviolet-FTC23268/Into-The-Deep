package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class PickUpCommand extends SequentialCommandGroup {
    public PickUpCommand() {

            addCommands(
                    //new InstantCommand(() -> RobotHardware.getInstance().intakeRead = true),
                    new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                    new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                    new WaitCommand(Globals.CLAW_MOVE_DELAY)
            );

           // if(RobotHardware.getInstance().intake.sampleDetected())
                addCommands(
                    new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED),
                    new ExtendoCommand(IntakeSubsystem.ExtendoState.TRANSFER)
                    //new InstantCommand(() -> RobotHardware.getInstance().intakeRead = false)
                );
            /*else
                addCommands(
                        new IntakeCommand(IntakeSubsystem.IntakeState.INTAKE),
                        new iClawCommand(IntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(Globals.CLAW_MOVE_DELAY),
                        new InstantCommand(() -> RobotHardware.getInstance().intakeRead = false)
                );*/

    }
}