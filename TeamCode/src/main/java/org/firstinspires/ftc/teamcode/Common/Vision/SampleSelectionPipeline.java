//Currently just gets the data from the limelight and finds the centers of the bounding boxes.
//DOES NOT DO ANY SELECTION

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

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.update();

            double[] center = {tx, ty};
        } else {
            telemetry.addData("No targets detected", "");
            telemetry.update();
        }
    }
}
