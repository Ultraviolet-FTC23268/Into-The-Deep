
package org.firstinspires.ftc.teamcode.Opmodes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.LocateSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.AutoCommand.OldLocateSampleCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.SystemCommand.iClawCommand;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.DetectionPipeline;
import org.firstinspires.ftc.teamcode.Common.Vision.SampleType;

@Config
@TeleOp(name = "Scan Test")
//@Disabled
public class ScanTest extends CommandOpMode {

    private ElapsedTime timer;
    private GamepadEx gamepadEx;

    private final RobotHardware robot = RobotHardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Pose target = new Pose(0,0,0);
    private DetectionPipeline.AnalyzedSample detectedSample;

    public static double strafeSpeed = 0.25;

    @Override
    public void initialize() {

        gamepadEx = new GamepadEx(gamepad1);

        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new LocateSampleCommand(SampleType.Blue)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new ExtendIntakeCommand()));
        gamepadEx.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> schedule(new iClawCommand(IntakeSubsystem.ClawState.OPEN)));

        robot.init(hardwareMap);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        FtcDashboard.getInstance().startCameraStream(robot.pipeline, 30.0);
        robot.intake.update(IntakeSubsystem.IntakeState.EXTENDED);
        robot.intake.update(IntakeSubsystem.ExtendoState.RETRACTED);
        //robot.intakeWristServo.setPosition(wristNeutralPos);
        robot.startCamera();

    }

    @Override
    public void run() {

        CommandScheduler.getInstance().run();

        robot.clearBulkCache();
        robot.read();
        robot.update();
        robot.write();

        if(gamepadEx.getButton(GamepadKeys.Button.X))
            robot.drivetrain.set(0, strafeSpeed, 0, 0);

        if(gamepadEx.getButton(GamepadKeys.Button.B))
            robot.drivetrain.set(0, -strafeSpeed, 0, 0);

        if(isStopRequested())
            robot.closeCamera();

    }

}
