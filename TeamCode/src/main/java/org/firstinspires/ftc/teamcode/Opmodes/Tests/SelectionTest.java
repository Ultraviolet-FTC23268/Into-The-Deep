
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.ClawAlignmentPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Config
@TeleOp(name = "Limelight Test")
//@Disabled
public class SelectionTest extends CommandOpMode {

    private ElapsedTime timer;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Limelight3A limelight;
    private int selectedClassIndex = 0;
    private String[] classNames = {"RedSample", "BlueSample", "YellowSample"};
    private static final double BOTTOM_CENTER_X = 0.0;
    private static final double BOTTOM_CENTER_Y = -30.0;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void run() {

        if (isStopRequested())
            limelight.stop();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            LLResultTypes.DetectorResult bestTarget = null;
            double bestDistance = Double.MAX_VALUE;

            // Filter and prioritize targets
            for (LLResultTypes.DetectorResult detection : detections) {
                if (detection.getClassName().equals(classNames[selectedClassIndex])) {
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();
                    double distance = Math.sqrt(Math.pow(tx - BOTTOM_CENTER_X, 2) + Math.pow(ty - BOTTOM_CENTER_Y, 2));

                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestTarget = detection;
                    }
                }
            }

            // Output selected target
            if (bestTarget != null) {
                telemetry.addData("Selected Target Class", classNames[selectedClassIndex]);
                telemetry.addData("Target X", bestTarget.getTargetXDegrees());
                telemetry.addData("Target Y", bestTarget.getTargetYDegrees());
            } else {
                telemetry.addData("No targets for class", classNames[selectedClassIndex]);
            }
        } else {
            telemetry.addData("Limelight", "No valid data");
        }
        telemetry.update();


    }

}
