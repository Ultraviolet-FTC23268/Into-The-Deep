package org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoHighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.dClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class AutomaticScoreCommand extends SequentialCommandGroup {

    Pose preIntakePos = new Pose(235, -750, 0);
    Pose intakePos = new Pose(60, -750, 0);
    Pose preScorePos = new Pose(670, 0, 0);
    Pose specScorePos = new Pose(680, Globals.specPose, 0);
    //Pose postPos = new Pose(500, Globals.specPose, 0);

    public AutomaticScoreCommand() {

        addCommands(
                new PositionCommand(preIntakePos)
                        .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN),
                                new DepositCommand(DepositSubsystem.DepositState.NEUTRAL),
                                new LiftCommand(LiftSubsystem.LiftState.RETRACTED),
                                new AutoHighSpecimenCommand())),
                new PositionCommand(intakePos),
                new dClawCommand(DepositSubsystem.ClawState.CLOSED),
                new WaitCommand(Globals.CLAW_MOVE_DELAY),
                new PositionCommand(preScorePos)
                        .alongWith(new AutoManualSpecOverrideCommand()),
                new PositionCommand(specScorePos)
                /*new WaitCommand(150),
                new SequentialCommandGroup( //manual score because this thing is stupid and dumb
                        new LiftCommand(LiftSubsystem.LiftState.HIGH_CHAMBER),
                        new InstantCommand(() -> RobotHardware.getInstance().depositElbowServo.setPosition(DepositSubsystem.elbowSpecScorePos)),
                        new WaitCommand(Globals.SPEC_SCORE_DELAY)),

                new PositionCommand(postPos)
                    .alongWith(new SequentialCommandGroup(new dClawCommand(DepositSubsystem.ClawState.OPEN)))
                        //new DepositCommand(DepositSubsystem.DepositState.SPEC_DEPOSIT),
                        //new LiftCommand(LiftSubsystem.LiftState.PRE_HIGH_CHAMBER)))*/
        );

    }
}