
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
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

@Config
@TeleOp(name = "Limelight Test")
//@Disabled
public class SelectionTest extends CommandOpMode {

    private ElapsedTime timer;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Limelight3A limelight;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void run() {

        if(isStopRequested())
            limelight.stop();

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
