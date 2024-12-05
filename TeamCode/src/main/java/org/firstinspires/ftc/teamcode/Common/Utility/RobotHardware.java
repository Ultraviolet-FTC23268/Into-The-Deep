package org.firstinspires.ftc.teamcode.Common.Utility;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.goBILDA.GoBildaPinpointDriver;

import java.util.List;

public class RobotHardware {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public MotorEx liftLeft;
    public MotorEx liftRight;

    public Motor.Encoder leftArmEncoder;
    public Motor.Encoder rightArmEncoder;

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
    public GoBildaPinpointDriver localizer;

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

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

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

        leftArmEncoder = new MotorEx(hardwareMap, "leftLiftMotor").encoder;
        rightArmEncoder = new MotorEx(hardwareMap, "rightLiftMotor").encoder;

        localizer = hardwareMap.get(GoBildaPinpointDriver.class,"localizer");
        localizer.setOffsets(-84.0, -168.0); //FIND OUT ACTUAL VALUES
        localizer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localizer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD); //CHECK THIS
        localizer.resetPosAndIMU();

        drivetrain = new MecanumDrivetrain();
        lift = new LiftSubsystem();

        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            //m.setConstant(100);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber()))
                CONTROL_HUB = m;
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

    }

    public void read() {
        lift.read();
    }

    public void write() {
        drivetrain.write();
        lift.write();
    }

    public void update() {
        localizer.update();
    }

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }


    public double getVoltage() {
        return voltage;
    }

}
