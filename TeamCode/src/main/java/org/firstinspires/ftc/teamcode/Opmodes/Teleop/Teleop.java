package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Utility.MathUtils;

@Config
@TeleOp(name = "Mecanum Teleop")
public class Teleop extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;
    private boolean lastJoystickUpRight = false;
    private boolean lastJoystickDownRight = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.read();

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

        double left_stick_y = -gamepad1.left_stick_y,
                left_stick_x = gamepad1.left_stick_x;
        double theta = Math.atan2(left_stick_y, left_stick_x) + Math.PI * 0.5;

        if (Math.abs(theta) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta - Math.PI) < Globals.ANGLE_SNAPPING_THRESHHOLD) {
            left_stick_x = 0;
        }
        if (Math.abs(theta - Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD || Math.abs(theta + Math.PI * 0.5) < Globals.ANGLE_SNAPPING_THRESHHOLD ) {
            left_stick_y = 0;
        }

        robot.drivetrain.set(new Pose(left_stick_x, left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);


        double loop = System.nanoTime();
        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}