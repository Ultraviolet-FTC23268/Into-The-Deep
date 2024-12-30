package org.firstinspires.ftc.teamcode.Common.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleSelectionPipeline {
    private Limelight3A limelight;
    private Telemetry telemetry;

    public SampleSelectionPipeline(Limelight3A limelightInstance, Telemetry telemetry) {
        this.limelight = limelightInstance;
        this.telemetry = telemetry;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();  // X offset of the target
            double ty = result.getTy();  // Y offset of the target

            // Update telemetry
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.update();

            // You can also store the center point if needed
            double[] center = {tx, ty};  // Example: store the center as an array of [tx, ty]
        } else {
            telemetry.addData("No targets detected", "");
            telemetry.update();
        }
    }
}
