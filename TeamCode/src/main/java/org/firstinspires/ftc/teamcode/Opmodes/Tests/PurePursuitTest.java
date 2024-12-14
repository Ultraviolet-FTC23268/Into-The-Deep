package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import java.util.ArrayList;

//@Config
@Autonomous(name = "PurePursuit Test")
@Disabled
public class PurePursuitTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    public static double SIZE = 7;

    public static ArrayList<Vector2D> targetPoints = new ArrayList<>();
    public static double finalHeading = 0;
    public static double lookAheadRadius = 8;
    public static PurePursuitCommand ppc;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap);
        robot.enabled = true;

        targetPoints.clear();
        targetPoints.add(new Vector2D(0, 32));
        targetPoints.add(new Vector2D(32, 32));
        targetPoints.add(new Vector2D(32, 0));
        targetPoints.add(new Vector2D(10, 40));
        targetPoints.add(new Vector2D(0, 0));



        ppc = new PurePursuitCommand(targetPoints, lookAheadRadius, finalHeading);

        while (!isStarted()) {
            robot.read();
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(ppc);

    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.update();
        robot.write();

        Pose currentPose = robot.localizer.getPose();
        TelemetryPacket packet = new TelemetryPacket();
        drawCircleWithLine(packet.fieldOverlay(), (currentPose.getX() - SIZE*0.5), (currentPose.getY() - SIZE*0.5), SIZE, currentPose.heading);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

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
        double x1 = x + Math.cos(heading + Math.PI * 0.5) * lineLength;
        double y1 = y + Math.sin(heading + Math.PI * 0.5) * lineLength;

        // Draw the line
        canvas.strokeLine(x, y, x1, y1);
    }
/*
    public static void drawPurePursuitPath(Canvas canvas, PurePursuitCommand purePursuitCommand) {
        if(!purePursuitCommand.hasGeneratedPath) return;
        Vector2D intersection = purePursuitCommand.getIntersectionPoint();
        ArrayList<Vector2D> targetPoints = purePursuitCommand.targetPoints;

        for (int i = 0; i < targetPoints.size() - 1; i++) {
            canvas.strokeLine(targetPoints.get(i).x, targetPoints.get(i).y, targetPoints.get(i + 1).x,
                    targetPoints.get(i + 1).y);
        }

        canvas.strokeCircle(intersection.x, intersection.y, 2);
    }*/
}
