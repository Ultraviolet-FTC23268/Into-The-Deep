package org.firstinspires.ftc.teamcode.Common.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.Common.Drive.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.Common.Utility.PDFLController;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.*;

@Config
public class LiftSubsystem extends SubsystemBase {

    private RobotHardware robot = RobotHardware.getInstance();

    private final ElapsedTime timer;
    private AsymmetricMotionProfile liftProfile;
    public ProfileState liftMotionState;
    public LiftState liftState = LiftState.RETRACTED;

    public PDFLController controller;

    public int leftLiftPos;
    public int rightLiftPos;
    public double power = 0.0;

    public static int targetPosition = 0;
    public static double liftRaiseSpeed = 1;
    public static double liftLowerSpeed = -1;
    private boolean withinTolerance = false;
    public boolean isReady = false;

    public double time = 0.0;
    public static boolean isUp = false;

    public static double P = 0;
    public static double D = 0;
    public static double F = 0;
    public static double Li = 0;
    public static double Ls = 0;
    public static double homeConstant = 0;


    public enum LiftState {
        RETRACTED,
        LOW_BUCKET,
        HIGH_BUCKET,
        LOW_CHAMBER,
        HIGH_CHAMBER

    }

    public LiftSubsystem() {

        this.liftProfile = new AsymmetricMotionProfile(0, 1, new ProfileConstraints(0, 0, 0));
        this.controller = new PDFLController(P, D, F, Li, Ls, LIFT_ERROR_TOLERANCE);
        this.controller.setHomeConstant(homeConstant);
        this.timer = new ElapsedTime();
    }

    public void update(LiftState state) {
        liftState = state;

        switch (state) {

            case RETRACTED:
                setTargetPos(RETRACTED_POS);
                break;
            case LOW_BUCKET:
                setTargetPos(LOW_BUCKET_POS);
                break;
            case HIGH_BUCKET:
                setTargetPos(HIGH_BUCKET_POS);
                break;
            case LOW_CHAMBER:
                setTargetPos(LOW_CHAMBER_POS);
                break;
            case HIGH_CHAMBER:
                setTargetPos(HIGH_CHAMBER_POS);
                break;

        }

    }

    public void loop() {
        this.controller.setConstants(P, D, F, Li, Ls, LIFT_ERROR_TOLERANCE);

        liftMotionState = liftProfile.calculate(timer.time());
        if (liftMotionState.v != 0) {
            setTargetPos((int) liftMotionState.x);
            time = timer.time();
        }

        withinTolerance = Math.abs(getRightPos() - getTargetPos()) < LIFT_ERROR_TOLERANCE;

        power = Range.clip(controller.calculate(getRightPos(), getTargetPos()), -1, 1);

        this.controller.setHome(liftState == LiftState.RETRACTED);

    }
    public void read() {
        try {
            //leftLiftPos = robot.leftArmEncoder.getPosition();
            rightLiftPos = robot.rightArmEncoder.getPosition();

        } catch (Exception e) {
            leftLiftPos = 0;
            rightLiftPos = 0;
        }
    }

    public void write() {
        if (robot.enabled) {
            try {
                robot.liftLeft.set(power);
                robot.liftRight.set(-power);
            } catch (Exception e) {
            }
        }
    }

    public double getLeftPos() {
        return leftLiftPos;
    }

    public double getRightPos() {
        return rightLiftPos;
    }

    public void setTargetPos(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPos() {
        return targetPosition;
    }

    public double getPower() {return power;}

    public boolean isWithinTolerance() {
        return withinTolerance;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, ProfileConstraints constraints) {
        //constraints.convert(LIFT_TICKS_PER_INCH);
        this.liftProfile = new AsymmetricMotionProfile(getLeftPos(), targetPos, constraints);
        resetTimer();
    }


}
