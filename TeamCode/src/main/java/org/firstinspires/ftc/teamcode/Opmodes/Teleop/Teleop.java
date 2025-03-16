package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.AutoHighSpecimenCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.AutomaticScoreCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ClimbUpCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.HighSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.LowSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ManualSpecOverrideCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ReadyIntakeCommand;
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
    private final PIDFController hController = new PIDFController(1.1, 0, 0.025, 0);
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private boolean lockHeading = false;

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.read();

        Globals.AUTO = false;

        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new AutoHighSpecimenCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> schedule(new iClawCommand(IntakeSubsystem.ClawState.OPEN)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new HighSampleCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new ScoreCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new ClimbUpCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new InstantCommand(() -> robot.lift.setTargetPos(1500))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new ReadyIntakeCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> schedule(new LowSampleCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> schedule(new AutomaticScoreCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> schedule(new InstantCommand(() -> robot.lift.setTargetPos(15))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> schedule(new InstantCommand(() -> robot.intake.changeWristPos(1))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> schedule(new InstantCommand(() -> robot.intake.changeWristPos(-1))));

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            //telemetry.update("auto in init");
            try {
                Thread.sleep(50L);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        robot.deposit.update(DepositSubsystem.DepositState.NEUTRAL);
        schedule(new SequentialCommandGroup(new WaitCommand(1000),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED)),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED))));
    }


    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        if(gamepad1.touchpad)
            robot.localizer.setPosition(new Pose2D(DistanceUnit.MM,60, -750, AngleUnit.RADIANS, 0));

        robot.lift.retract = gamepadEx.getButton(GamepadKeys.Button.Y);
        robot.intake.stickValue = -gamepadEx.getRightY();

        if(gamepadEx.getRightX() >= 0.92)
            schedule(new ExtendIntakeCommand());
        else if(gamepadEx.getRightX() <= -0.92)
            schedule(new RetractIntakeCommand());

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT))
            robot.imuOffset = robot.localizer.getHeading();

        if(gamepadEx.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
            lockHeading = true;

        double left_stick_y = -gamepad1.left_stick_y,
                left_stick_x = -gamepad1.left_stick_x;
        double theta = Math.atan2(left_stick_y, left_stick_x) + Math.PI * 0.5;

        if (Math.abs(theta) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta - Math.PI) < Globals.ANGLE_SNAPPING_THRESHHOLD) {
            left_stick_x = 0;
        }
        if (Math.abs(theta - Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta + Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD ) {
            left_stick_y = 0;
        }

        double error = normalizeRadians(normalizeRadians(0) - normalizeRadians(robot.localizer.getHeading()));
        double headingCorrection = -hController.calculate(0 - robot.imuOffset, error) * 12.4 / robot.getVoltage();
        if (Math.abs(headingCorrection) < 0.01)
            headingCorrection = 0;

        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(turn) > 0.002)
            lockHeading = false;

        robot.drivetrain.set(new Pose(left_stick_y, left_stick_x, lockHeading ? headingCorrection: MathUtils.joystickScalar(turn, 0.01)), 0);

        double loop = System.nanoTime();
        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}

//balls