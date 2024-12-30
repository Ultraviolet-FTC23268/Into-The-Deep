package org.firstinspires.ftc.teamcode.Common.Utility;

import android.util.Size;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Vision.ClawAlignmentPipeline;
import org.firstinspires.ftc.teamcode.Common.goBILDA.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.List;

public class RobotHardware {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public MotorEx liftLeft;
    public MotorEx liftRight;

    public Servo intakeClawServo;
    public Servo depositClawServo;
    public Servo intakeElbowServo;
    public Servo depositElbowServo;
    public Servo depositArmServo;
    public Servo depositArm2Servo;
    public Servo intakeArmServo;
    public Servo intakeArm2Servo;
    public Servo intakeWristServo;
    public Servo railServo;
    public Servo slideLeftServo;
    public Servo slideRightServo;

    public Motor.Encoder leftLiftEncoder;
    public Motor.Encoder rightLiftEncoder;

    public RevColorSensorV3 depositClawColor;
    public RevColorSensorV3 intakeClawColor;

    //public BNO055IMU imu;

    private double voltage = 12.0;
    private ElapsedTime voltageTimer;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    public MecanumDrivetrain drivetrain;
    public LiftSubsystem lift;
    public DepositSubsystem deposit;
    public IntakeSubsystem intake;
    public GoBildaPinpointDriver localizer;

    private VisionPortal visionPortal;
    public ClawAlignmentPipeline alignmentPipeline;

    public boolean depositRead = false;
    public boolean intakeRead = false;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        /*this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/

        voltageTimer = new ElapsedTime();

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft = new MotorEx(hardwareMap, "leftLiftMotor");
        liftRight = new MotorEx(hardwareMap, "rightLiftMotor");

        liftLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLiftEncoder = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
        rightLiftEncoder = new MotorEx(hardwareMap, "frontRightMotor").encoder;

        leftLiftEncoder.setDirection(Motor.Direction.REVERSE);

        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        depositClawServo = hardwareMap.get(Servo.class, "depositClawServo");
        intakeElbowServo = hardwareMap.get(Servo.class, "intakeElbowServo");
        depositElbowServo = hardwareMap.get(Servo.class, "depositElbowServo");
        depositArmServo = hardwareMap.get(Servo.class, "depositArmServo");
        depositArm2Servo = hardwareMap.get(Servo.class, "depositArm2Servo");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        intakeArm2Servo = hardwareMap.get(Servo.class, "intakeArm2Servo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        railServo = hardwareMap.get(Servo.class, "railServo");
        slideLeftServo = hardwareMap.get(Servo.class, "slideLeftServo");
        slideRightServo = hardwareMap.get(Servo.class, "slideRightServo");

        depositArm2Servo.setDirection(Servo.Direction.REVERSE);
        intakeArm2Servo.setDirection(Servo.Direction.REVERSE);
        slideLeftServo.setDirection(Servo.Direction.REVERSE);

        localizer = hardwareMap.get(GoBildaPinpointDriver.class,"localizer");
        localizer.setOffsets(-98.2, -171.651);
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //depositClawColor = hardwareMap.get(RevColorSensorV3.class, "depositClawColor");
        intakeClawColor = hardwareMap.get(RevColorSensorV3.class, "intakeClawColor");

        drivetrain = new MecanumDrivetrain();
        lift = new LiftSubsystem();
        deposit = new DepositSubsystem();
        intake = new IntakeSubsystem();

        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            //m.setConstant(100);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber()))
                CONTROL_HUB = m;
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        alignmentPipeline = new ClawAlignmentPipeline();

    }

    public void read() {
        lift.read();
        if(depositRead);
            //deposit.read();
        if(intakeRead);
            //intake.read();

    }

    public void write() {
        drivetrain.write();
        lift.write();
    }

    public void update() {
        localizer.update();
        lift.loop();
        intake.loop();
    }

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }


    public double getVoltage() {
        return voltage;
    }

    public void startCamera() {

        alignmentPipeline = new ClawAlignmentPipeline();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(alignmentPipeline)
                .enableLiveView(false)
                .build();
    }

    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        this.visionPortal.setProcessorEnabled(processor, enabled);
    }

}
