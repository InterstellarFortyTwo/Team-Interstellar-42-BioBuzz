package org.firstinspires.ftc.teamcode.HardwareCode;

public class SwerveRateLimiterSettings {
    double acceleration = 2.5;
    double deceleration = 4.0;
    double rotAcceleration = 3.0;
    double rotDeceleration = 5.0;
    public SwerveRateLimiterSettings(double acceleration,double deceleration, double rotAcceleration, double rotDeceleration) {
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.rotAcceleration = rotAcceleration;
        this.rotDeceleration = rotDeceleration;
    }
    public void setAcceleration(double newAcceleration) { acceleration = newAcceleration; }
    public double getAcceleration() { return acceleration; }
    public void setDeceleration(double newDeceleration) { deceleration = newDeceleration; }
    public double getDeceleration() { return  deceleration; }
    public void setRotAcceleration(double newRotAcceleration) { rotAcceleration = newRotAcceleration; }
    public double getRotAcceleration() { return rotAcceleration; }
    public void setRotDeceleration(double newRotDeceleration) { rotDeceleration = newRotDeceleration; }
    public double getRotDeceleration() { return  rotDeceleration; }
}
