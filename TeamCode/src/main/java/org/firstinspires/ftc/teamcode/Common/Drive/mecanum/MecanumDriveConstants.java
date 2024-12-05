package org.firstinspires.ftc.teamcode.Common.Drive.mecanum;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumDriveConstants {
    public static double MAX_LINEAR_SPEED = 79.2;
    public static double MAX_LINEAR_ACCELERATION = 20;

    public static double MAX_TRANSLATIONAL_SPEED = 0.7;
    public static double MAX_ROTATIONAL_SPEED = 0.5;

    public static double STRAFE_GAIN = 8;
    public static double FORWARD_GAIN = 6;
    public static double ROTATIONAL_GAIN = 0.65;
    public static double X_GAIN = 2.00;

    public static double ACCEL_LIMIT = 0.5;
    
    public static double xP = 0.095;
    public static double xD = 0.011;

    public static double yP = 0.09;
    public static double yD = 0.011;

    public static double hP = 1.1;
    public static double hD = 0.045;


    public static double k_static = 0.017;
    public static double k_static_h = 0.017;
    public static double min_power = 0;
    public static double min_power_h = 0;
}