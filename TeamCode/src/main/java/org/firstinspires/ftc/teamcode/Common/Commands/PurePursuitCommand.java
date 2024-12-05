package org.firstinspires.ftc.teamcode.Common.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.*;
import java.util.ArrayList;

@Config
public class PurePursuitCommand extends CommandBase {

    private RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain = robot.drivetrain;
    public Pose finalPose;

    private ElapsedTime timer;
    private ElapsedTime stable;
    public static double ALLOWED_TRANSLATIONAL_ERROR = 5;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(15);

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);
    public static double ACCEL_LIMIT = 0.5;
    public static double STABLE_MS = 250;
    public static double MAX_TIME_MS = 5000;

    public ArrayList<Vector2D> targetPoints;
    public double lookaheadRadius;
    public int currentTarget = 1;

    private boolean finished = false;

    public PurePursuitCommand(ArrayList<Vector2D> targetPoints, double lookaheadRadius, double finalHeading) {

        this.targetPoints = targetPoints;
        this.lookaheadRadius = lookaheadRadius;
        this.finalPose = new Pose(targetPoints.get(targetPoints.size() - 1), finalHeading);
        
    }

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
        }
        if (stable == null) {
            stable = new ElapsedTime();
        }

        Pose nextLocation = this.getNextPoint();
        Pose robotPose = robot.localizer.getPose();

        // power the robot to go towards the intersection point
        Pose powers = getPower(robotPose, nextLocation);
        drivetrain.set(powers.scale(Math.min(1, timer.seconds() / ACCEL_LIMIT)));

    }
    @Override
    public boolean isFinished() {
        Pose robotPose = robot.localizer.getPose();
        Pose delta = finalPose.subt(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }
        boolean timedOut = timer.milliseconds() > MAX_TIME_MS;
        boolean foundPathEnd = stable.milliseconds() > STABLE_MS;

        return timedOut || (foundPathEnd && finished);
    }

    @Override
    public void end(boolean interrupted) {
        targetPoints.clear();
        drivetrain.set(new Pose());
    }

    private Pose getNextPoint() {

        Vector2D robotPos = robot.localizer.getPose().toVec2D();
        Vector2D targetPoint = targetPoints.get(currentTarget);
        Vector2D prevPoint = targetPoints.get(currentTarget-1);

        //Checking if next point is within robot radius
        if(robotPos.distanceTo(targetPoint) > lookaheadRadius) {

            Vector2D intersection = lineCircleIntersection(prevPoint, targetPoint, robotPos, lookaheadRadius);

            if (currentTarget == targetPoints.size() - 1)
                return new Pose(intersection, finalPose.heading);
            //add checking if flipping backwards is quicker than forwards
            double heading = intersection.subt(robotPos).angle() - Math.PI/2;;
            return new Pose(intersection, heading);

        }
        else {

            if(currentTarget == targetPoints.size() - 1) {

                finished = true;
                return finalPose;

            }

            currentTarget += 1;
            return getNextPoint();

        }

    }

    public Pose getPower(Pose robotPose, Pose intersectionPoint) {

        Pose targetPose = intersectionPoint;

        if (targetPose.heading - robotPose.heading > Math.PI)
            targetPose.heading -= 2 * Math.PI;
        if (targetPose.heading - robotPose.heading < -Math.PI)
            targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        // Lower limit for power to counteract friction
        if (Math.abs(x_rotated) < min_power)
            x_rotated = Math.signum(x_rotated) * min_power;
        if (Math.abs(y_rotated) < min_power)
            y_rotated = Math.signum(y_rotated) * min_power;
        if (Math.abs(hPower) < min_power_h)
            hPower = Math.signum(hPower) * min_power;

        // Feed forward to counteract friction
        x_rotated += Math.signum(x_rotated) * k_static;
        y_rotated += Math.signum(y_rotated) * k_static;
        hPower += Math.signum(hPower) * k_static;

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        return new Pose(x_rotated * X_GAIN, y_rotated, hPower);
    }

    public static Vector2D lineCircleIntersection(Vector2D pointA, Vector2D pointB, Vector2D center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;
        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;
        double pBy2 = bBy2 / a;
        double q = c / a;
        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return pointA;
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;
        Vector2D p1 = new Vector2D(pointA.x - baX * abScalingFactor1, pointA.y - baY * abScalingFactor1);
        if (disc == 0) {
            return p1;
        }
        Vector2D p2 = new Vector2D(pointA.x - baX * abScalingFactor2, pointA.y - baY * abScalingFactor2);
        return Math.hypot(pointB.x - p1.x, pointB.y - p1.y) > Math.hypot(pointB.x - p2.x, pointB.y - p2.y) ? p2 : p1;
    }

}
