package org.firstinspires.ftc.teamcode.Common.Utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {

    public static boolean AUTO = false;
    public static Color ALLIANCE = Color.BLUE;

    //CHANGE THIS TO LASER POINTER VALUE
    public static double subDistance = 940;

    public static int intakeAlphaThreshold = 500;
    public static int depositAlphaThreshold = 100;

    public static int specPose = 125;

    public static double ANGLE_SNAPPING_THRESHHOLD = Math.toRadians(30); // +- 30 degrees

    //LIFT POS
    public static int RETRACTED_POS = 25;
    public static int LOW_BUCKET_POS = 1100;
    public static int HIGH_BUCKET_POS = 2600;
    public static int PRE_HIGH_CHAMBER_POS = 1250;
    public static int HIGH_CHAMBER_POS = 1550;
    public static int LIFT_OFFSET = 0;

    public static int LIFT_ERROR_TOLERANCE = 0;

    public static int CLAW_MOVE_DELAY = 150;
    public static int SPEC_SCORE_DELAY = 200;
    public static int TSETUP_DELAY = 250;
    public static int TSWAP_DELAY = 350;
    public static int TPLACE_DELAY = 250;
    public static int PULL_OUT_DELAY = 250;
    public static int EXTEND_DELAY = 400;

}
