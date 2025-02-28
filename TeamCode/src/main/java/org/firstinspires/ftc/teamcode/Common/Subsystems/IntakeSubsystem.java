package org.firstinspires.ftc.teamcode.Common.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Opmodes.Teleop.Teleop;

@Config
public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot = RobotHardware.getInstance();

    public IntakeState intakeState = IntakeState.OVERVIEW;
    public ClawState clawState = ClawState.OPEN;
    public ExtendoState extendoState = ExtendoState.RETRACTED;

    public WristState wristState = WristState.NEUTRAL;

    public double stickValue = 0;
    private int wristPos = 0;

    public static double wristNeutralPos = 0.61;
    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristMinus45Pos = 0.45;
    public static double wristPlus45Pos = 0.77;

    public static double iClawClosedPos = 0.43;
    public static double iClawOpenPos = 0.2;
    public static double iClawLoosePos = 0.38;

    public static double slideRetractedPos = 0.2;
    public static double slidePreTransferPos = 0.22;
    public static double slideExtendedPos = 0.4;
    public static double slideOffset = 0.0;

    public static double elbowNeutralPos = 0.15;
    public static double elbowOverviewPos = 0.32;
    public static double elbowPreTransferPos = 0.83;
    public static double elbowTransferPos = 0.88;
    public static double elbowExtendedPos = 0.35;
    public static double elbowIntakePos = 0.22;
    public static double elbowPickupPos = 0.23;
    public static double elbowSweepPos = .62;

    public static double iArmOverviewPos = 0.44;
    public static double iArmNeutralPos = 0.18;//0.37
    public static double iArmTransferPos = 0.46;
    public static double iArmPostTransferPos = 0.54;
    public static double iArmExtendedPos = 0.6;
    public static double iArmIntakePos = 0.66;
    public static double iArmPickupPos = 0.76;
    public static double iArmSweepPos = 0.95;
    public static double iArmOffset = 0;

    private boolean pickupReady = false;

    public static double maxExtension = 0.4;

    private Color detectedColor = Color.NOTHING;
    private float[] hsvValues = new float[3];
    private int alpha = 0;

    public double servoAngle = wristNeutralPos;
    public static double wristMultiplier = 0.005;
    public static double extendoMultiplier = 0.0025;

    public boolean autoWrist = false;

    public enum IntakeState {
        NEUTRAL,
        OVERVIEW,
        PRE_TRANSFER,
        TRANSFER,
        POST_TRANSFER,
        EXTENDED,
        INTAKE,
        PICK_UP,
        SWEEP
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

    public enum WristState {
        MINUS_90,
        MINUS_45,
        NEUTRAL,
        PLUS_45,
        PLUS_90
    }

    public IntakeSubsystem() {}

    public void update(IntakeState state) {

        intakeState = state;
        switch (state) {

            case NEUTRAL:
                update(ClawState.OPEN);
                update(ExtendoState.RETRACTED);
                update(WristState.NEUTRAL);
                robot.intakeWristServo.setPosition(wristNeutralPos);
                robot.intakeArmServo.setPosition(iArmNeutralPos);
                robot.intakeArm2Servo.setPosition(iArmNeutralPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowNeutralPos);
                //robot.closeCamera();
                break;
            case OVERVIEW:
                update(ClawState.OPEN);
                update(ExtendoState.RETRACTED);
                update(WristState.NEUTRAL);
                robot.intakeArmServo.setPosition(iArmOverviewPos);
                robot.intakeArm2Servo.setPosition(iArmOverviewPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowOverviewPos);
                break;
            case PRE_TRANSFER:
                update(ClawState.CLOSED);
                update(ExtendoState.TRANSFER);
                update(WristState.NEUTRAL);
                robot.intakeArmServo.setPosition(iArmTransferPos);
                robot.intakeArm2Servo.setPosition(iArmTransferPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowPreTransferPos);
                //robot.closeCamera();
                break;
            case TRANSFER:
                update(ExtendoState.RETRACTED);
                update(WristState.NEUTRAL);
                robot.intakeClawServo.setPosition(iClawLoosePos);
                robot.intakeElbowServo.setPosition(elbowTransferPos);
                robot.intakeArmServo.setPosition(iArmTransferPos);
                robot.intakeArm2Servo.setPosition(iArmTransferPos + iArmOffset);
                break;
            case POST_TRANSFER:
                update(WristState.NEUTRAL);
                robot.intakeArmServo.setPosition(iArmPostTransferPos);
                robot.intakeArm2Servo.setPosition(iArmPostTransferPos + iArmOffset);
                break;
            case EXTENDED:
                //robot.startCamera();
                pickupReady = false;
                update(WristState.NEUTRAL);
                robot.intakeArmServo.setPosition(iArmExtendedPos);
                robot.intakeArm2Servo.setPosition(iArmExtendedPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowExtendedPos);
                break;
            case INTAKE:
                update(ClawState.OPEN);
                update(WristState.NEUTRAL);
                wristPos = 0;
                robot.intakeArmServo.setPosition(iArmIntakePos);
                robot.intakeArm2Servo.setPosition(iArmIntakePos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowIntakePos);
                pickupReady = true;

                break;
            case PICK_UP:
                update(ClawState.OPEN);
                robot.intakeArmServo.setPosition(iArmPickupPos);
                robot.intakeArm2Servo.setPosition(iArmPickupPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowPickupPos);
                pickupReady = true;
                break;
            case SWEEP:
                update(ClawState.CLOSED);
                robot.intakeArmServo.setPosition(iArmSweepPos);
                robot.intakeArm2Servo.setPosition(iArmSweepPos + iArmOffset);
                robot.intakeElbowServo.setPosition(elbowSweepPos);
                pickupReady = false;
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

    public void update(WristState state) {

        wristState = state;
            switch (state) {

                case MINUS_90:
                    robot.intakeWristServo.setPosition(wristMinPos);
                    break;
                case MINUS_45:
                    robot.intakeWristServo.setPosition(wristMinus45Pos);
                    break;
                case NEUTRAL:
                    robot.intakeWristServo.setPosition(wristNeutralPos);
                    break;
                case PLUS_45:
                    robot.intakeWristServo.setPosition(wristPlus45Pos);
                    break;
                case PLUS_90:
                    robot.intakeWristServo.setPosition(wristMaxPos);
                    break;

            }

    }

    public void updateWristPos(int row) {

        if(intakeState == IntakeState.INTAKE)
            switch (row) {
                case -2:
                    this.update(WristState.MINUS_90);
                    break;
                case -1:
                    this.update(WristState.MINUS_45);
                    break;
                case 0:
                    this.update(WristState.NEUTRAL);
                    break;
                case 1:
                    this.update(WristState.PLUS_45);
                    break;
                case 2:
                    this.update(WristState.PLUS_90);
                    break;
            }

    }

    public void changeWristPos(int amount) {

        if(intakeState == IntakeState.INTAKE)
            wristPos += amount;

        if(wristPos > 2)
            wristPos = -2;
        if(wristPos < -2)
            wristPos = 2;

        updateWristPos(wristPos);

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

        /*int red = robot.intakeClawColor.red();
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
        }*/

    }

    public boolean sampleDetected() {
        return (detectedColor == Globals.ALLIANCE || detectedColor == Color.YELLOW) && alpha > Globals.intakeAlphaThreshold;
    }

    public Color getDetectedColor() {
        return detectedColor;
    }

    public void loop() {
        if (intakeState == IntakeState.INTAKE) {
            /*if(autoWrist && !Globals.AUTO) {
                servoAngle = Range.clip(wristNeutralPos - ((robot.alignmentPipeline.getSampleAngle() - 90) / 300), wristMinPos, wristMaxPos);
            }
            else {
                //robot.intakeWristServo.setPosition(servoAngle != wristMaxPos ? servoAngle : wristNeutralPos);
                if (leftShoulderInput) servoAngle += wristMultiplier;
                else if (rightShoulderInput) servoAngle -= wristMultiplier;
                servoAngle = Range.clip(servoAngle, wristMinPos, wristMaxPos);
            }*/
            //robot.intakeWristServo.setPosition(servoAngle);

            slideExtendedPos += stickValue * extendoMultiplier;
            slideExtendedPos = Range.clip(slideExtendedPos, slideRetractedPos, maxExtension);
            robot.slideLeftServo.setPosition(slideExtendedPos);
            robot.slideRightServo.setPosition(slideExtendedPos + slideOffset);
        }
    }

}
