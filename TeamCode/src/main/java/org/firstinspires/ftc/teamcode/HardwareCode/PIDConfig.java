package org.firstinspires.ftc.teamcode.HardwareCode;

public class PIDConfig {
    private double kP = 0.0001;
    private double kD = 0.0000;
    private double kI = 0;
    private double kF = 0;
    private double goalX = 0;


    public PIDConfig(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkI(double newKI) {
        kI = newKI;
    }

    public double getkI() {
        return kI;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }
    public double getkD() {
        return kD;
    }
    public void setkF(double newKF) { kF = newKF; }

    public double getkF() {
        return kF;
    }
}
