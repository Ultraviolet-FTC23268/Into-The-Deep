package org.firstinspires.ftc.teamcode.Common.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Utility.Color;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
public class DepositSubsystem extends SubsystemBase {

    private final RobotHardware robot = RobotHardware.getInstance();

    public DepositState depositState = DepositState.TRANSFER;

    public ClawState clawState = ClawState.OPEN;

    public static double dClawClosedPos = 0.46;
    public static double dClawOpenPos = 0.09;

    public static double railRetractedPos = 0.41;
    public static double railSpecPos = 0.55;
    public static double railExtendedPos = 0.61;

    public static double elbowNeutralPos = 0.08;
    public static double elbowTransferPos = 0.68;
    public static double elbowSampDepositPos = 0.87;
    public static double elbowSpecScorePos = 0.8;
    public static double elbowSpecDepositPos = 0.63;
    public static double elbowSpecIntakePos = 0.63;
    public static double elbowAutoPos = 0.71;

    public static double dArmNeutralPos = 0.43;
    public static double dArmTransferPos = 0.12;
    public static double dArmSampDepositPos = 0.55;
    public static double dArmSpecDepositPos = 0.1;
    public static double dArmSpecIntakePos = 0.81;
    public static double dArmAutoPos = 0.2;

    public static double dArmOffset = 0;

    private Color detectedColor = Color.NOTHING;
    private float[] hsvValues = new float[3];
    private int alpha = 0;

    public enum DepositState {
        AUTO,
        NEUTRAL,
        TRANSFER,
        SAMP_DEPOSIT,
        SPEC_DEPOSIT,
        SPEC_INTAKE
    }

    public enum ClawState {
        CLOSED,
        OPEN
    }

    public DepositSubsystem() {}

    public void update(DepositState state) {

        depositState = state;
        switch (state) {

            case AUTO:
                //update(ClawState.CLOSED);
                robot.depositElbowServo.setPosition(elbowAutoPos);
                robot.depositArmServo.setPosition(dArmAutoPos);
                robot.depositArm2Servo.setPosition(dArmAutoPos + dArmOffset);
                robot.railServo.setPosition(railRetractedPos);
                //update(ClawState.OPEN);
                break;
            case NEUTRAL:
                //update(ClawState.CLOSED);
                robot.depositElbowServo.setPosition(elbowNeutralPos);
                robot.depositArmServo.setPosition(dArmNeutralPos);
                robot.depositArm2Servo.setPosition(dArmNeutralPos + dArmOffset);
                robot.railServo.setPosition(railRetractedPos);
                //update(ClawState.OPEN);
                break;
            case TRANSFER:
                update(ClawState.CLOSED);
                robot.depositElbowServo.setPosition(elbowTransferPos);
                robot.depositArmServo.setPosition(dArmTransferPos);
                robot.depositArm2Servo.setPosition(dArmTransferPos + dArmOffset);
                robot.railServo.setPosition(railRetractedPos);
                update(ClawState.OPEN);
                break;
            case SAMP_DEPOSIT:
                update(ClawState.CLOSED);
                robot.depositElbowServo.setPosition(elbowSampDepositPos);
                robot.depositArmServo.setPosition(dArmSampDepositPos);
                robot.depositArm2Servo.setPosition(dArmSampDepositPos + dArmOffset);
                robot.railServo.setPosition(railRetractedPos);
                break;
            case SPEC_DEPOSIT:
                update(ClawState.CLOSED);
                robot.depositElbowServo.setPosition(elbowSpecDepositPos);
                robot.depositArmServo.setPosition(dArmSpecDepositPos);
                robot.depositArm2Servo.setPosition(dArmSpecDepositPos + dArmOffset);
                robot.railServo.setPosition(railExtendedPos);
                break;
            case SPEC_INTAKE:
                robot.depositElbowServo.setPosition(elbowSpecIntakePos);
                robot.depositArmServo.setPosition(dArmSpecIntakePos);
                robot.depositArm2Servo.setPosition(dArmSpecIntakePos + dArmOffset);
                robot.railServo.setPosition(railSpecPos);
                break;

        }

    }

    public void update(ClawState state) {

        clawState = state;
        switch (state) {

            case CLOSED:
                robot.depositClawServo.setPosition(dClawClosedPos);
                break;
            case OPEN:
                robot.depositClawServo.setPosition(dClawOpenPos);
                break;

        }

    }

    public void read() {

        int red = robot.depositClawColor.red();
        int green = robot.depositClawColor.green();
        int blue = robot.depositClawColor.blue();

        // Read alpha (proximity/brightness)
        alpha = robot.depositClawColor.alpha();

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
        } else if (hue >= 180 && hue <= 250) {
            detectedColor = Color.BLUE;
        } else {
            detectedColor = Color.NOTHING;
        }

    }

    public boolean sampleDetected() {
        Color allianceColor = Globals.ALLIANCE;
        return (detectedColor == allianceColor || detectedColor == Color.YELLOW) && alpha > Globals.depositAlphaThreshold;
    }

    public Color getDetectedColor() {
        return detectedColor;
    }

}
