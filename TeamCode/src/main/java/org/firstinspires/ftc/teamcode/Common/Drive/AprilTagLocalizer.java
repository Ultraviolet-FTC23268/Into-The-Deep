package org.firstinspires.ftc.teamcode.Common.Drive;

import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;


public class AprilTagLocalizer {
    public static final double Y_OFFSET = 8.7;
    public static final double X_OFFSET = -0.1;

    public static final Pose BLUE_BACKDROP_POSITION = new Pose(-35.5, 61.25, Math.PI/2);
    public static final Pose RED_BACKDROP_POSITION = new Pose(-35.5, -61.25, -Math.PI/2);

    public static Pose convertBlueBackdropPoseToGlobal(Pose pipeline) {
        pipeline.x += X_OFFSET;
        pipeline.y += Y_OFFSET;
        pipeline.x *= -1;
        pipeline.y *= -1;

        return pipeline.add(BLUE_BACKDROP_POSITION);
    }

    public static Pose convertRedBackdropPoseToGlobal(Pose pipeline) {
        pipeline.x += X_OFFSET;
        pipeline.y += Y_OFFSET;

        return pipeline.add(RED_BACKDROP_POSITION);
    }
}