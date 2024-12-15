package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.HighSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.HighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Utility.MathUtils;

@Config
@TeleOp(name = "RAH RAH RAH RAH")
public class Teleop extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.read();

        robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);
        robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED);
        robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new HighSpecimenCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> schedule(new ManualSpecOverrideCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new HighSampleCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new ScoreCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new ExtendIntakeCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new RetractIntakeCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new IntakeCommand(IntakeSubsystem.IntakeState.EXTENDED)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> schedule(new iClawCommand(IntakeSubsystem.ClawState.OPEN)));


        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        robot.intake.touchpadValue = -gamepadEx.getRightY();
        robot.intake.leftShoulderInput = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        robot.intake.rightShoulderInput = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);

        double left_stick_y = -gamepad1.left_stick_y,
                left_stick_x = -gamepad1.left_stick_x;
        double theta = Math.atan2(left_stick_y, left_stick_x) + Math.PI * 0.5;

        if (Math.abs(theta) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta - Math.PI) < Globals.ANGLE_SNAPPING_THRESHHOLD) {
            left_stick_x = 0;
        }
        if (Math.abs(theta - Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta + Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD ) {
            left_stick_y = 0;
        }

        robot.drivetrain.set(new Pose(left_stick_y, left_stick_x, MathUtils.joystickScalar(gamepad1.right_trigger - gamepad1.left_trigger, 0.01)), 0);


        double loop = System.nanoTime();
        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
        telemetry.addData("Sample Angle ", robot.alignmentPipeline.getSampleAngle());
        loopTime = loop;
        telemetry.update();
    }
}