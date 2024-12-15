package org.firstinspires.ftc.teamcode.Common.Drive.mecanum;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumDriveConstants {
    public static double MAX_LINEAR_SPEED = 79.2;
    public static double MAX_LINEAR_ACCELERATION = 20;

    public static double MAX_TRANSLATIONAL_SPEED = 1;
    public static double MAX_ROTATIONAL_SPEED = 0.5;

    public static double STRAFE_GAIN = 8;
    public static double FORWARD_GAIN = 6;
    public static double ROTATIONAL_GAIN = 0.65;
    public static double X_GAIN = 2.00;
    public static double Y_GAIN = 2.00;

    public static double ACCEL_LIMIT = 0.5;
    
    public static double xP = 0.0005;
    public static double xD = 0.00001;

    public static double yP = 0.002;
    public static double yD = 0.00015;

    public static double hP = 1.1;
    public static double hD = 0.025;


    public static double k_static = 0.05;
    public static double min_power = 0;
    public static double min_power_h = 0;
}