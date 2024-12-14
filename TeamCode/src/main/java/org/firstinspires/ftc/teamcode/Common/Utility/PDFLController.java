package org.firstinspires.ftc.teamcode.Common.Utility;

public class PDFLController {
    private double kP, kD, kF, kL, kLIntercept, kLSlope;
    private double lastError = 0;
    private double lastTimeStamp = 0;
    private double minErrorThreshhold = 0;
    private boolean home = false;
    private double homeConstant = 0;

    public boolean isHome() {
        return home;
    }

    public void setHome(boolean home) {
        this.home = home;
    }

    public double getHomeConstant() {
        return homeConstant;
    }

    public void setHomeConstant(double homeConstant) {
        this.homeConstant = homeConstant;
    }

    public double getMinErrorThreshhold() {
        return minErrorThreshhold;
    }

    public void setMinErrorThreshhold(double minErrorThreshhold) {
        this.minErrorThreshhold = minErrorThreshhold;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    /*public double getkL() {
        return kL;
    }

    public void setkL(double kL) {
        this.kL = kL;
    }*/

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getLastTimeStamp() {
        return lastTimeStamp;
    }

    public void setLastTimeStamp(double lastTimeStamp) {
        this.lastTimeStamp = lastTimeStamp;
    }

    public void setConstants(double kP, double kD, double kF, double kL, double minErrorThreshhold) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        //this.kLIntercept = kLIntercept;
        //this.kLSlope = kLSlope;
        this.minErrorThreshhold = minErrorThreshhold;
    }

    public PDFLController(double kP, double kD, double kF, double kL, double minErrorThreshhold) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        //this.kLIntercept = kLIntercept;
        //this.kLSlope = kLSlope;
        this.minErrorThreshhold = minErrorThreshhold;
    }

    public double calculate(double current, double target) {
        if (home) {
            return homeConstant;
        }
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        double dt = currentTimeStamp - lastTimeStamp;

        double error = target - current;
        double dError = error - lastError;
        double P = error * kP;
        double D = dt > 1E6 ? (dError / dt) * kD : 0;
        double F = kF;
        double L = Math.abs(error) > minErrorThreshhold ? Math.signum(error) * kL : 0;//(kLIntercept + kLSlope * current) : 0;
        // We dont want to add a lower limit term if we are within the
        // minErrorThreshhold

        lastError = error;
        lastTimeStamp = currentTimeStamp;

        return P + D + F + L;

    }

    public void reset() {
        lastError = 0;
        lastTimeStamp = 0;
    }

}
