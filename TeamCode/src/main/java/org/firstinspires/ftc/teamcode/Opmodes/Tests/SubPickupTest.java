
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Commands.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Point;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.DetectionPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name = "Auto Pickup Test")
//@Disabled
public class SubPickupTest extends CommandOpMode {

    private ElapsedTime timer;
    private GamepadEx gamepadEx;

    private final RobotHardware robot = RobotHardware.getInstance();
    private DetectionPipeline pipeline;
    private VisionPortal portal;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double loopTime = 0.0;
    public static double wristMinPos = 0.28;
    public static double wristMaxPos = 0.93;
    public static double wristNeutralPos = 0.61;
    private double servoAngle = wristNeutralPos;

    private Pose target = new Pose(0,0,0);
    private DetectionPipeline.AnalyzedSample detectedSample;

    @Override
    public void initialize() {

        gamepadEx = new GamepadEx(gamepad1);

        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new SequentialCommandGroup(
                        new PositionCommand(target),
                        new WaitCommand(150),
                        new IntakeCommand(IntakeSubsystem.IntakeState.PICK_UP),
                        new WaitCommand(150),
                        new iClawCommand(IntakeSubsystem.ClawState.CLOSED),
                        new WaitCommand(325),
                        new IntakeCommand(IntakeSubsystem.IntakeState.OVERVIEW))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> schedule(new iClawCommand(IntakeSubsystem.ClawState.OPEN)));

        pipeline = new DetectionPipeline();
        pipeline.sampleType = SampleType.Red;
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
        telemetry.addData("Location ", detectedSample.getTranslate());
        telemetry.update();
/*
        if (Math.abs(lastAngle - currentAngle) > minChange){
            servoAngle = Range.clip(wristNeutralPos - ((currentAngle - 90) / 300), wristMinPos, wristMaxPos);
            robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);
        }
*/
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER) && detectedSample != null) {
            servoAngle = Range.clip(wristNeutralPos - ((detectedSample.getAngle() - 90) / 300), wristMinPos, wristMaxPos);
            robot.intakeWristServo.setPosition(servoAngle != wristMinPos ? servoAngle : wristNeutralPos);

            Pose robotPose = robot.localizer.getPose();
            Point sample = detectedSample.getTranslate();
            target = new Pose(robotPose.x + (sample.x - 65), robotPose.y + sample.y, robotPose.heading);
        }

        if (isStopRequested()) {
            portal.close();
        }

    }

}
