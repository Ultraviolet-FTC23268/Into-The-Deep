
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
@TeleOp(name = "FSM Test")
@Disabled
public class fsmTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double loopTime = 0.0;

    public static DepositSubsystem.DepositState dState = DepositSubsystem.DepositState.NEUTRAL;
    public static IntakeSubsystem.IntakeState iState = IntakeSubsystem.IntakeState.INTAKE;

    public static IntakeSubsystem.ClawState cState = IntakeSubsystem.ClawState.CLOSED;
    public static DepositSubsystem.ClawState c2State = DepositSubsystem.ClawState.CLOSED;

    private GamepadEx gamepadEx;


    @Override
    public void initialize() {

        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);
        robot.enabled = true;

        gamepadEx = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.deposit.update(dState);
        schedule(new SequentialCommandGroup(new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.update(iState))));


    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("target: ", robot.lift.getTargetPos());
        telemetry.addData("position: ", robot.lift.getLeftPos());

        telemetry.update();

        if(gamepadEx.getButton(GamepadKeys.Button.A))
            robot.intake.update(iState);

        if(gamepadEx.getButton(GamepadKeys.Button.B))
            robot.intake.update(cState);

        if(gamepadEx.getButton(GamepadKeys.Button.X))
            robot.deposit.update(c2State);

        if(gamepadEx.getButton(GamepadKeys.Button.Y))
            robot.deposit.update(dState);

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP))
            robot.intake.update(IntakeSubsystem.ExtendoState.EXTENDED);

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN))
            robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);

        CommandScheduler.getInstance().run();

    }

}
