
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "Claw Auto Align Test")
//@Disabled
public class ClawAutoAlignTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    private ClawAlignmentPipeline alignmentPipeline;
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


    @Override
    public void initialize() {

        Globals.AUTO = false;
        CommandScheduler.getInstance().reset();

        alignmentPipeline = new ClawAlignmentPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(alignmentPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        robot.init(hardwareMap);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        FtcDashboard.getInstance().startCameraStream(alignmentPipeline, 30);

        robot.intake.update(IntakeSubsystem.IntakeState.INTAKE);
        robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);
        //robot.intakeWristServo.setPosition(wristNeutralPos);
    }

    @Override
    public void run() {

        if (isStopRequested()) {
            portal.close();
        }

        currentAngle = alignmentPipeline.getSampleAngle();

        CommandScheduler.getInstance().run();

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        telemetry.addData("Sample Angle ", currentAngle);
        telemetry.update();

        if (Math.abs(lastAngle - currentAngle) > minChange){
            servoAngle = Range.clip(wristNeutralPos - ((currentAngle - 90) / 300), wristMinPos, wristMaxPos);
            robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);
        }


        lastAngle = currentAngle;

    }

}
