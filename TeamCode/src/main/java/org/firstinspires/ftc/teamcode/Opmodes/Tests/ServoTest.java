
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
@TeleOp(name = "Servo Test")
@Disabled
public class ServoTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double loopTime = 0.0;

    public static double clawPos = 0.2;
    public static double elbowPos = 0.94;
    public static double wristPos = 0.61;
    public static double armPos = 0.58;
    public static double arm2Pos = 0.58;
    public static double railPos = 0.41;
    public static double dArmPos = 0.3;
    public static double dArm2Pos = 0.3;
    public static double dElbowPos = 0.5;
    public static double dClawPos = 0.13;
    public static double leftSlidePos = 0.2;
    public static double rightSlidePos = 0.2;

    private GamepadEx gamepadEx;


    @Override
    public void initialize() {

        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);
        robot.enabled = true;

        gamepadEx = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));

        telemetry.update();

        CommandScheduler.getInstance().run();

        if(gamepadEx.getButton(GamepadKeys.Button.A)) {

            robot.railServo.setPosition(railPos);
            schedule(new WaitCommand(100));
            robot.depositArmServo.setPosition(dArmPos);
            schedule(new WaitCommand(100));
            robot.depositArm2Servo.setPosition(dArm2Pos);
            schedule(new WaitCommand(100));
            robot.depositElbowServo.setPosition(dElbowPos);
            schedule(new WaitCommand(100));
            robot.depositClawServo.setPosition(dClawPos);
            schedule(new WaitCommand(100));

            robot.intakeClawServo.setPosition(clawPos);
            schedule(new WaitCommand(100));
            robot.intakeElbowServo.setPosition(elbowPos);
            schedule(new WaitCommand(100));
            robot.intakeWristServo.setPosition(wristPos);
            schedule(new WaitCommand(100));
            robot.intakeArmServo.setPosition(armPos);
            schedule(new WaitCommand(100));
            robot.intakeArm2Servo.setPosition(arm2Pos);
            schedule(new WaitCommand(100));

            robot.slideLeftServo.setPosition(leftSlidePos);
            schedule(new WaitCommand(100));
            robot.slideRightServo.setPosition(rightSlidePos);
            schedule(new WaitCommand(100));

        }

    }

}
