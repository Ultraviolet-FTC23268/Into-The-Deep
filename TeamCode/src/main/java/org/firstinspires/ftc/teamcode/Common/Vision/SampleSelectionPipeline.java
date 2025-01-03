package org.firstinspires.ftc.teamcode.Common.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class SampleSelectionPipeline {

    private Limelight3A limelight;
    private Telemetry telemetry;
    private int selectedClassIndex = 0;
    private String[] classNames = {"RedSample", "BlueSample", "YellowSample"};
    private static final double BOTTOM_CENTER_X = 0.0;
    private static final double BOTTOM_CENTER_Y = -30.0;

    public SampleSelectionPipeline(Limelight3A limelightInstance, Telemetry telemetry) {
        this.limelight = limelightInstance;
        this.telemetry = telemetry;
    }

    public void init() {
        // Initialize Limelight
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setSelectedClass(int classIndex) {
        if (classIndex >= 0 && classIndex < classNames.length) {
            selectedClassIndex = classIndex;
            telemetry.addData("Selected Class", classNames[classIndex]);
            telemetry.update();
        }
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<DetectorResult> detections = result.getDetectorResults();

            DetectorResult bestTarget = null;
            double bestDistance = Double.MAX_VALUE;

            // Filter and prioritize targets
            for (DetectorResult detection : detections) {
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
