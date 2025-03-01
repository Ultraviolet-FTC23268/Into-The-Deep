
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.ClawAlignmentPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.DetectionPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name = "Vision Test")
//@Disabled
public class VisionTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    private DetectionPipeline pipeline;
    private VisionPortal portal;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double loopTime = 0.0;
    private double lastAngle = 0;

    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;

    public static double minChange = 12.5;

    private double servoAngle = wristNeutralPos;

    private double currentAngle = 0;
    private DetectionPipeline.AnalyzedSample detectedSample;

    @Override
    public void initialize() {

        Globals.AUTO = false;
        CommandScheduler.getInstance().reset();

        pipeline = new DetectionPipeline();
        pipeline.sampleType = SampleType.Blue;
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(pipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        robot.init(hardwareMap);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        FtcDashboard.getInstance().startCameraStream(pipeline, 30.0);
        robot.intake.update(IntakeSubsystem.IntakeState.OVERVIEW);
        robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);
        //robot.intakeWristServo.setPosition(wristNeutralPos);

    }

    @Override
    public void run() {

        CommandScheduler.getInstance().run();

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        detectedSample = null;
        while (detectedSample == null && opModeIsActive()) {
            detectedSample = pipeline.chooseClosestValidSample();
        }

        telemetry.addData("Sample Angle ", detectedSample.getAngle());
        telemetry.addData("Area ", pipeline.AREA);
        telemetry.update();
/*
        if (Math.abs(lastAngle - currentAngle) > minChange){
            servoAngle = Range.clip(wristNeutralPos - ((currentAngle - 90) / 300), wristMinPos, wristMaxPos);
            robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);
        }
*/
        if (isStopRequested()) {
            portal.close();
        }

    }

}
