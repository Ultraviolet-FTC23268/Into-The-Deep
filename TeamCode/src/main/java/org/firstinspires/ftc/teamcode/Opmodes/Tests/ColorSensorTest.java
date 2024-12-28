package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;

import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@TeleOp(name = "Color Sensor Test")
@Disabled
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware
        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        robot.depositRead = true;
        robot.intakeRead = true;

        RevColorSensorV3 dSensor = robot.depositClawColor;
        RevColorSensorV3 iSensor = robot.intakeClawColor;

        float[] hsvValuesDeposit = new float[3];
        float[] hsvValuesIntake = new float[3];

        float dHue;
        float iHue;

        waitForStart();

        while (opModeIsActive()) {

            robot.clearBulkCache();
            robot.read();

            Color.RGBToHSV(dSensor.red() * 8, dSensor.green() * 8, dSensor.blue() * 8, hsvValuesDeposit);
            Color.RGBToHSV(iSensor.red() * 8, iSensor.green() * 8, iSensor.blue() * 8, hsvValuesIntake);
            dHue = hsvValuesDeposit[0];
            iHue = hsvValuesIntake[0];

            // Display values on the Driver Station
            telemetry.addData("Deposit Hue", dHue);
            telemetry.addData("Deposit Alpha", dSensor.alpha());
            telemetry.addData("Deposit Color", robot.deposit.getDetectedColor());
            telemetry.addLine();
            telemetry.addData("Intake Hue", iHue);
            telemetry.addData("Intake Alpha", iSensor.alpha());
            telemetry.addData("Intake Color", robot.intake.getDetectedColor());
            telemetry.update();

        }
    }
}
