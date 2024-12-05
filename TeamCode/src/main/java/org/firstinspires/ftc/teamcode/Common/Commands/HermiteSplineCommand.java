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
public class HermiteSplineCommand extends CommandBase {
    private RobotHardware robot = RobotHardware.getInstance();
    Drivetrain drivetrain = robot.drivetrain;
    public Pose finalPose;
    private ElapsedTime timer;
    private ElapsedTime stable;
    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(15);
    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double STABLE_MS = 250;
    public static double MAX_TIME_MS = 5000;

    public ArrayList<Vector2D> targetPoints;
    public int currentTarget = 1;

    public HermiteSplineCommand(ArrayList<Vector2D> targetPoints, double finalHeading) {
        this.targetPoints = targetPoints;
        this.finalPose = new Pose(targetPoints.get(targetPoints.size() - 1), finalHeading);
    }
}