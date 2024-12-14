package org.firstinspires.ftc.teamcode.Common.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.ClawAlignmentPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot = RobotHardware.getInstance();

    private ClawAlignmentPipeline alignmentPipeline;
    private VisionPortal portal;

    public IntakeState intakeState = IntakeState.OVERVIEW;
    public ClawState clawState = ClawState.OPEN;
    public ExtendoState extendoState = ExtendoState.RETRACTED;

    public static double wristNeutralPos = 0.61;

    public static double iClawClosedPos = 0.41;
    public static double iClawOpenPos = 0.2;
    public static double iClawLoosePos = 0.38;

    public static double slideRetractedPos = 0.2;
    public static double slidePreTransferPos = 0.22;
    public static double slideExtendedPos = 0.44;
    public static double slideOffset = 0.0;

    public static double elbowNeutralPos = 0.15;
    public static double elbowOverviewPos = 0.32;
    public static double elbowPreTransferPos = 0.83;
    public static double elbowTransferPos = 0.88;
    public static double elbowExtendedPos = 0.35;
    public static double elbowIntakePos = 0.22;
    public static double elbowPickupPos = 0.23;

    public static double iArmOverviewPos = 0.7;
    public static double iArmNeutralPos = 0.42;
    public static double iArmTransferPos = 0.68;
    public static double iArmPostTransferPos = 0.8;
    public static double iArmExtendedPos = 0.85;
    public static double iArmIntakePos = 0.91;
    public static double iArmOffset = 0;
    public static double iArmPickupPos = 0.94;

    private boolean pickupReady = false;

    public static double maxExtension = 0.44;

    private Color detectedColor = Color.NOTHING;
    private float[] hsvValues = new float[3];
    private int alpha = 0;

    private int wristRotation = 0;

    public enum IntakeState {
        NEUTRAL,
        OVERVIEW,
        PRE_TRANSFER,
        TRANSFER,
        POST_TRANSFER,
        EXTENDED,
        INTAKE,
        PICK_UP
    }

    public enum ClawState {
        CLOSED,
        OPEN
    }

    public enum ExtendoState {
        RETRACTED,
        TRANSFER,
        EXTENDED
    }

    public IntakeSubsystem() {}

    public void update(IntakeState state) {

        intakeState = state;
        switch (state) {

            case NEUTRAL:
                update(ClawState.OPEN);
                update(ExtendoState.RETRACTED);
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmNeutralPos);
                robot.intakeArm2Servo.setPosition(iArmNeutralPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowNeutralPos);
                break;
            case OVERVIEW:
                update(ClawState.OPEN);
                update(ExtendoState.RETRACTED);
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmOverviewPos);
                robot.intakeArm2Servo.setPosition(iArmOverviewPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowOverviewPos);
                break;
            case PRE_TRANSFER:
                update(ClawState.CLOSED);
                update(ExtendoState.TRANSFER);
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmTransferPos);
                robot.intakeArm2Servo.setPosition(iArmTransferPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowPreTransferPos);
                robot.closeCamera();
                break;
            case TRANSFER:
                update(ExtendoState.RETRACTED);
                robot.intakeClawServo.setPosition(iClawLoosePos);
                robot.intakeElbowServo.setPosition(elbowTransferPos);
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmTransferPos);
                robot.intakeArm2Servo.setPosition(iArmTransferPos + iArmOffset);
                break;
            case POST_TRANSFER:
                robot.intakeArmServo.setPosition(iArmPostTransferPos);
                robot.intakeArm2Servo.setPosition(iArmPostTransferPos + iArmOffset);
                break;
            case EXTENDED:
                pickupReady = false;
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmExtendedPos);
                robot.intakeArm2Servo.setPosition(iArmExtendedPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowExtendedPos);
                break;
            case INTAKE:
                update(ClawState.OPEN);
                robot.intakeArmServo.setPosition(iArmIntakePos);
                robot.intakeArm2Servo.setPosition(iArmIntakePos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowIntakePos);
                pickupReady = true;
                robot.startCamera();

                break;
            case PICK_UP:
                update(ClawState.OPEN);
                robot.intakeArmServo.setPosition(iArmPickupPos);
                robot.intakeArm2Servo.setPosition(iArmPickupPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowPickupPos);
                pickupReady = true;
                break;

        }

    }

    public void update(ClawState state) {

        clawState = state;
        switch (state) {

            case CLOSED:
                robot.intakeClawServo.setPosition(iClawClosedPos);
                break;
            case OPEN:
                robot.intakeClawServo.setPosition(iClawOpenPos);
                break;

        }

    }

    public void update(ExtendoState state) {

        extendoState = state;
        switch (state) {

            case RETRACTED:
                robot.slideLeftServo.setPosition(slideRetractedPos);
                robot.slideRightServo.setPosition(slideRetractedPos + slideOffset);
                break;
            case TRANSFER:
                robot.slideLeftServo.setPosition(slidePreTransferPos);
                robot.slideRightServo.setPosition(slidePreTransferPos + slideOffset);
                break;
            case EXTENDED:
                robot.slideLeftServo.setPosition(slideExtendedPos);
                robot.slideRightServo.setPosition(slideExtendedPos + slideOffset);
                break;

        }

    }

    public void pickup() {
        if(pickupReady) {

            robot.intakeArmServo.setPosition(iArmPickupPos);
            robot.intakeArm2Servo.setPosition(iArmPickupPos + iArmOffset);
            robot.intakeElbowServo.setPosition(elbowPickupPos);
            update(ClawState.CLOSED);
            pickupReady = false;

        }
    }

    public void calculateExtension(double distance) {

        //blah blah inverse kinematics math math lalala
        if(distance > maxExtension)
            slideExtendedPos = maxExtension;
        else
            slideExtendedPos = 0;

    }

    public void read() {

        int red = robot.intakeClawColor.red();
        int green = robot.intakeClawColor.green();
        int blue = robot.intakeClawColor.blue();

        // Read alpha (proximity/brightness)
        alpha = robot.intakeClawColor.alpha();

        // Convert RGB to HSV
        android.graphics.Color.RGBToHSV(
                red * 8, // Scale RGB values
                green * 8,
                blue * 8,
                hsvValues
        );

        // Get the hue from the HSV values
        float hue = hsvValues[0];

        // Determine the detected color based on hue ranges
        if (hue >= 0 && hue <= 30 || hue >= 340 && hue <= 360) {
            detectedColor = Color.RED;
        } else if (hue >= 70 && hue <= 90) {
            detectedColor = Color.YELLOW;
        } else if (hue >= 210 && hue <= 240) {
            detectedColor = Color.BLUE;
        } else {
            detectedColor = Color.NOTHING;
        }

    }

    public boolean sampleDetected() {
        Color allianceColor = Globals.ALLIANCE;
        return (detectedColor == allianceColor || detectedColor == Color.YELLOW) && alpha > Globals.intakeAlphaThreshold;
    }

    public Color getDetectedColor() {
        return detectedColor;
    }

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;

    public void loop() {
        double servoAngle = Range.clip(wristNeutralPos - ((alignmentPipeline.getSampleAngle() - 90)/300), wristMinPos, wristMaxPos);
        if (intakeState == IntakeState.INTAKE)
            robot.intakeWristServo.setPosition(servoAngle);
    }

}
