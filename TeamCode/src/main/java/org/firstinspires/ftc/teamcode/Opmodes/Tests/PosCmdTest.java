
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
@Autonomous(name = "Pos Command Test")
@Disabled
public class PosCmdTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double loopTime = 0.0;

    public static double posX = 0;
    public static double posY = 0;
    public static double posH = 0;

    public static double posX2 = 0;
    public static double posY2 = 0;
    public static double posH2 = 0;

    public static double SIZE = 10.0;

    Pose testPos = new Pose();
    Pose testPos2 = new Pose();

    @Override
    public void initialize() {

        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        testPos = new Pose(posX, posY, posH);
        testPos2 = new Pose(posX2, posY2, posH2);

        while (!isStarted()) {
            robot.read();
            telemetry.addLine("auto in init");
            telemetry.update();
            robot.clearBulkCache();
            robot.write();
        }

        // robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0)); // not neccessary anymore because version 1.0 automatically sets the pose to 0,0,0 when update is called for the first time
        timer = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(testPos),
                        new PositionCommand(testPos2)
                )
        );
    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        Pose currentPose = robot.localizer.getPose();

        TelemetryPacket packet = new TelemetryPacket();
        drawCircleWithLine(packet.fieldOverlay(), (currentPose.getX() - SIZE*0.5), (currentPose.getY() - SIZE*0.5), SIZE, currentPose.heading);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("xPos ", currentPose.getX());
        telemetry.addData("yPos ", currentPose.getY());
        telemetry.addData("hPos ", currentPose.heading);
        loopTime = loop;
        telemetry.update();

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
