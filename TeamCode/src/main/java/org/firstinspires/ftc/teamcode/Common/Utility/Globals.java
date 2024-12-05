package org.firstinspires.ftc.teamcode.Common.Utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;

@Config
public class Globals {

    public static boolean AUTO = false;

    public static double ANGLE_SNAPPING_THRESHHOLD = Math.toRadians(15); // +- 15 degrees

    //LIFT POS
    public static int RETRACTED_POS = 0;
    public static int LOW_BUCKET_POS = 0;
    public static int HIGH_BUCKET_POS = 0;
    public static int LOW_CHAMBER_POS = 0;
    public static int HIGH_CHAMBER_POS = 0;

    public static double LIFT_ERROR_TOLERANCE = 0;

}
