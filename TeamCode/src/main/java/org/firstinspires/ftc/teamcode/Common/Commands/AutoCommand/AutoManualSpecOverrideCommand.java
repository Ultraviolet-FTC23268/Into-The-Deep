package org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class AutoManualSpecOverrideCommand extends SequentialCommandGroup {

    public AutoManualSpecOverrideCommand() {

        super(
                new LiftCommand(LiftSubsystem.LiftState.PRE_HIGH_CHAMBER),
                new DepositCommand(DepositSubsystem.DepositState.SPEC_DEPOSIT)
                //new WaitCommand(250)
                //new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED)
        );

    }
}