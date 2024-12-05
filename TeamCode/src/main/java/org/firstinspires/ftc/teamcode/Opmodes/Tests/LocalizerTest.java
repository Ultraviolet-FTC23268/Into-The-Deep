package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.MathUtils;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Autonomous(name = "Localizer Test")
//@Disabled
public class LocalizerTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public static double SIZE = 10.0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        robot.enabled = true;
        robot.read();

        while (!isStarted()) {
            robot.read();
            telemetry.addLine("auto in init");
            telemetry.update();
        }

    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.update();
        robot.write();

        robot.drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        /*Pose currentPose = robot.localizer.getPose();
        TelemetryPacket packet = new TelemetryPacket();
        drawCircleWithLine(packet.fieldOverlay(), (currentPose.getX() - SIZE*0.5), (currentPose.getY() - SIZE*0.5), SIZE, currentPose.heading);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);*/

        double loop = System.nanoTime();
        telemetry.addData("xPos ", robot.localizer.getPose().getX());
        telemetry.addData("yPos ", robot.localizer.getPose().getY());
        telemetry.addData("hPos ", robot.localizer.getPose().heading);

        CommandScheduler.getInstance().run();

    }

    public static void drawCircleWithLine(Canvas canvas, double x, double y, double radius, double heading) {

        // Draw the circle
        canvas.strokeCircle(x, y, radius);

        // Calculate the end points of the line based on the heading and radius
        double lineLength = radius * 2; // The line length is twice the radius
        double x1 = x + Math.cos(heading + Math.PI*0.5) * lineLength;
        double y1 = y + Math.sin(heading + Math.PI*0.5) * lineLength;

        // Draw the line
        canvas.strokeLine(x, y, x1, y1);
    }

}
