package org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import static org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDriveConstants.*;

@Config
public class PositionCommand extends CommandBase {
    Drivetrain drivetrain;
    public Pose targetPose;

    public static final PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static final PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static final PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 25;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(10);

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double STABLE_MS = 100;
    public double DEAD_MS = 1250;
   
    public PositionCommand(Pose targetPose) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;

        resetControllers();
    }

    public PositionCommand(Pose targetPose, int dead) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;

        xController.reset();
        yController.reset();
        hController.reset();

        DEAD_MS = dead;
    }

    public void resetControllers() {
        xController.reset();
        yController.reset();
        hController.reset();
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robotPose = robot.localizer.getPose();

//        System.out.println("TARGET POSE " + targetPose);

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose robotPose = robot.localizer.getPose();
        Pose delta = targetPose.subtract(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
    }

    public Pose getPower(Pose robotPose) {

        if(targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if(targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        if (Math.abs(x_rotated) < min_power) x_rotated = Math.signum(x_rotated) * min_power;
        if (Math.abs(y_rotated) < min_power) y_rotated = Math.signum(y_rotated) * min_power;
        if (Math.abs(hPower) < min_power_h) hPower = Math.signum(hPower) * min_power;

        // Feed forward to counteract friction
        x_rotated += Math.signum(x_rotated) * k_static; // counteract friction
        y_rotated += Math.signum(y_rotated) * k_static;
        hPower += Math.signum(hPower) * k_static;

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED/Y_GAIN, MAX_TRANSLATIONAL_SPEED/Y_GAIN);

        return new Pose(x_rotated * X_GAIN, y_rotated * Y_GAIN, hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
