package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareCode.PIDConfig;
import org.firstinspires.ftc.teamcode.HardwareCode.FieldToggle;
import org.firstinspires.ftc.teamcode.HardwareCode.testingFlywheelVelocity;
import org.firstinspires.ftc.teamcode.HardwareCode.SwerveRateLimiterSettings;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "NathanOpMode")
public class NathanOpMode extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime loopTimer = new ElapsedTime();

    boolean isShooting = false;
    double flywheelVel;
    double targetFlywheelVel = 1160;
    boolean lastB = false;
    boolean lastC = false;
    boolean lastE = false;
    boolean lastF = false;
    boolean isShooting2 = false;
    boolean angleSnapping = false;
    boolean lastD = false;
    boolean intakeIsOn = false;
    boolean flowerServo = false;
    boolean intakeIsReverse = false;
    boolean flywheelIsReversed = false;
    boolean fieldToggle;
    double testingVel = 1160;
    double maxAccel = 2.5;
    double maxDecel = 4.0;
    double maxRotAccel = 3.0;
    double maxRotDecel = 5.0;

    double r_integralSum;
    public static double f_kD = 0, f_kI = 0, f_kP = 1.16, f_kF = 11.6;
    PIDConfig LaunchPID = new PIDConfig(f_kP, f_kI, f_kD, f_kF);
    FieldToggle Movement = new FieldToggle(false);
    testingFlywheelVelocity Vel = new testingFlywheelVelocity(testingVel);
    SwerveRateLimiterSettings Swerve = new SwerveRateLimiterSettings(maxAccel, maxDecel, maxRotAccel, maxRotDecel);

    double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    //Index to select the current step size from the array
    int stepIndex = 1;

    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    double yaw;
    double axial;
    double lateral;
    boolean lastA;
    double y;
    double x;
    double rotCheck;
    double turn;
    double heading;
    double Degrees;

    DcMotor frontleft;
    DcMotor backleft;
    DcMotor frontright;
    DcMotor backright;
    DcMotorEx flywheel;
    DcMotorEx intake;
    CRServo windmill;
    Servo leftServo;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windmill = hardwareMap.get(CRServo.class, "windmill");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap.get(IMU.class, "imu");

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        imu.resetYaw();
        waitForStart();
        runtime.reset();
        targetFlywheelVel = testingVel;
        while (opModeIsActive()) {

            //-----------------------------------initialization------------------------------------------

            double dt = loopTimer.seconds();
            loopTimer.reset();
            YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
            heading = angle.getYaw(AngleUnit.RADIANS);
            Degrees = angle.getYaw(AngleUnit.DEGREES);
            flywheelVel = -flywheel.getVelocity();
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rotCheck = gamepad1.right_stick_x;
            turn = gamepad1.right_stick_x;

            //----------------------------------Testing & Preferences---------------------------------------------

            if(gamepad2.leftBumperWasPressed()) {
                Vel.setTestingFlywheelVelocity(Vel.getTestingFlywheelVelocity() + 10);
            }
            if(gamepad2.rightBumperWasPressed()) {
                Vel.setTestingFlywheelVelocity(Vel.getTestingFlywheelVelocity() - 10);
            }
            testingVel = Vel.getTestingFlywheelVelocity();

            if (gamepad2.bWasPressed()) {
                stepIndex = (stepIndex +1) % stepSizes.length; //Modulo wraps the index back to 0
            }
            if (gamepad2.dpadLeftWasPressed()) {
                LaunchPID.setkP(LaunchPID.getkP() - stepSizes[stepIndex]);
            }
            if (gamepad2.dpadRightWasPressed()) {
                LaunchPID.setkP(LaunchPID.getkP() + stepSizes[stepIndex]);
            }
            if (gamepad2.dpadUpWasPressed()) {
                LaunchPID.setkF(LaunchPID.getkF() + stepSizes[stepIndex]);
            }
            if (gamepad2.dpadDownWasPressed()) {
                LaunchPID.setkF(LaunchPID.getkF() - stepSizes[stepIndex]);
            }
            f_kP = LaunchPID.getkP();
            f_kF = LaunchPID.getkF();

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(f_kP, f_kI, f_kD, f_kF);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            if (gamepad2.a && !lastF) {
                isShooting2 = !isShooting2;
            }
            lastF = gamepad2.a;

            if(isShooting2) {

                shootTimer.reset();

                flywheel.setVelocity(testingVel);

                if (targetFlywheelVel - flywheelVel >= -80 && targetFlywheelVel - flywheelVel <= 10) {
                    windmill.setPower(1);
                }
                else {
                    windmill.setPower(0);
                }
            }
            else {
                flywheel.setPower(0);
                windmill.setPower(0);
            }


            if(gamepad2.bWasPressed()) {
                Swerve.setAcceleration(Swerve.getAcceleration() + stepSizes[stepIndex]);
                Swerve.setDeceleration(Swerve.getDeceleration() + stepSizes[stepIndex]);
                Swerve.setRotAcceleration(Swerve.getRotAcceleration() + stepSizes[stepIndex]);
                Swerve.setRotDeceleration(Swerve.getRotDeceleration() + stepSizes[stepIndex]);
            }
            if(gamepad2.xWasPressed()) {
                Swerve.setAcceleration(Swerve.getAcceleration() - stepSizes[stepIndex]);
                Swerve.setDeceleration(Swerve.getDeceleration() - stepSizes[stepIndex]);
                Swerve.setRotAcceleration(Swerve.getRotAcceleration() - stepSizes[stepIndex]);
                Swerve.setRotDeceleration(Swerve.getRotDeceleration() - stepSizes[stepIndex]);
            }
            if(Swerve.getAcceleration() <= 0) {
                Swerve.setAcceleration(Swerve.getAcceleration() + stepSizes[stepIndex]);
            }
            else if(Swerve.getDeceleration() <= 0) {
                Swerve.setDeceleration(Swerve.getDeceleration() + stepSizes[stepIndex]);
            }
            else if(Swerve.getRotAcceleration() <= 0) {
                Swerve.setRotAcceleration(Swerve.getRotAcceleration() + stepSizes[stepIndex]);
            }
            else if(Swerve.getRotDeceleration() <= 0) {
                Swerve.setRotDeceleration(Swerve.getRotDeceleration() + stepSizes[stepIndex]);
            }
            maxAccel = Swerve.getAcceleration();
            maxDecel = Swerve.getDeceleration();
            maxRotAccel = Swerve.getRotAcceleration();
            maxRotDecel = Swerve.getRotDeceleration();


            if (gamepad2.y && !lastA) {
                Movement.setFieldToggle(!Movement.getFieldToggle());
            }
            lastA = gamepad2.y;
            fieldToggle = Movement.getFieldToggle();

            if (fieldToggle) {
                axial = slewLimit(y * cos(-heading) - x * sin(-heading), axial, maxAccel, maxDecel, dt);
                lateral = slewLimit(y * sin(-heading) + x * cos(-heading), lateral, maxAccel, maxDecel,dt);
                telemetry.addLine("Field Centric Toggled: On");
            } else {
                axial = slewLimit(y, axial, maxAccel, maxDecel, dt);
                lateral = slewLimit(x, lateral, maxAccel, maxDecel, dt);
                telemetry.addLine("Field Centric Toggled: Off");
            }


            //------------------------------------Movement----------------------------------------


            if (gamepad1.dpad_left) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_right && !lastC) {
                angleSnapping = !angleSnapping;
            }
            lastC = gamepad1.dpad_right;

            if (angleSnapping) {
                if (Math.abs(0 - Degrees) <= 10 && Math.abs(rotCheck) < 0.05 && Math.abs(0 - Degrees) <= 1.5) {
                    yaw = (0 - Degrees) * 0.1;
                }
                else if (Math.abs(90 - Degrees) <= 10 && Math.abs(rotCheck) < 0.05 && Math.abs(90 - Degrees) <= 1.5) {
                    yaw = (90 - Degrees) * 0.1;
                }
                else if (Math.abs(-90 - Degrees) <= 10 && Math.abs(rotCheck) < 0.05 && Math.abs(-90 - Degrees) <= 1.5) {
                    yaw = (-90 - Degrees) * 0.1;
                }
                else if (Math.abs(180 - Degrees) <= 10 && Math.abs(rotCheck) < 0.05 && Math.abs(180 - Degrees) <= 1.5) {
                    yaw = (180 - Degrees) * 0.1;
                }
                else {
                    yaw = slewLimit(turn, yaw, maxRotAccel, maxRotDecel, dt);
                }
            telemetry.addLine("Angle snapping: On");
            }
            else {
                telemetry.addLine("Angle snapping: Off");
                yaw = gamepad1.right_stick_x;
            }

            frontRightPower = axial + lateral + yaw;
            frontLeftPower = axial - lateral - yaw;
            backRightPower = axial - lateral + yaw;
            backLeftPower = axial + lateral - yaw;

            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            if (Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05) {

                frontleft.setPower(0);
                frontright.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);
            }
            else {
            frontleft.setPower(frontLeftPower);
            frontright.setPower(frontRightPower);
            backleft.setPower(backLeftPower);
            backright.setPower(backRightPower);
            }


            //-----------------------------------Shooting---------------------------------------


            if (gamepad1.rightBumperWasPressed()) {
                targetFlywheelVel += 10;
            }
            if (gamepad1.leftBumperWasPressed()) {
                targetFlywheelVel -= 10;
            }

            flywheelIsReversed = gamepad1.left_trigger_pressed;

            if (gamepad1.a && !lastB) {
                isShooting = !isShooting;
            }
            lastB = gamepad1.a;

            if(isShooting && !flywheelIsReversed) {

                shootTimer.reset();

                flywheel.setVelocity(targetFlywheelVel);

                if (targetFlywheelVel - flywheelVel >= -80 && targetFlywheelVel - flywheelVel <= 10) {
                    windmill.setPower(1);
                }
                else {
                    windmill.setPower(0);
                }
            }
            else if (flywheelIsReversed){
                flywheel.setPower(-1);
                windmill.setPower(0);
            }
            else {
                flywheel.setPower(0);
                windmill.setPower(0);
            }


            //-------------------------------intake--------------------------------


            if (gamepad1.b && !lastD) {
                intakeIsOn = !intakeIsOn;
            }
            lastD = gamepad1.b;

            intakeIsReverse = gamepad1.right_trigger_pressed;

            if (intakeIsOn && !intakeIsReverse) {
                intake.setPower(1);
            }
            else if (intakeIsReverse) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            if(gamepad1.y && !lastE) {
                flowerServo = !flowerServo;
            }
            lastE = gamepad1.y;

            if (flowerServo) {
                leftServo.setPosition(0.0);
            }
            else {
                leftServo.setPosition(0.7);
            }


            //---------------------telemetry--------------------
            telemetry.addData("Testing Velocity", testingVel);
            telemetry.addData("PID Values","Proportion: %.5f, Feed-Forward: %.5f", LaunchPID.getkP(), LaunchPID.getkF());
            telemetry.addData("Swerve Drive Settings", "Max Acceleration: %.3f, Max Deceleration: %.3f", Swerve.getAcceleration(), Swerve.getDeceleration());
            telemetry.addData("Swerve Rotation Settings", "Max Rotation Acceleration: %.3f, Max Rotation Deceleration %.3f", Swerve.getRotAcceleration(), Swerve.getRotDeceleration());
            telemetry.addData("step size: ", stepSizes[stepIndex]);
            telemetry.addLine("---------------------");
            telemetry.addData("Current rotation: ", Degrees);
            telemetry.addData("Target flywheel velocity: ", targetFlywheelVel);
            telemetry.addData("Current flywheel velocity: ", flywheelVel);
            telemetry.addData("Run time", runtime);
            telemetry.update();
        }
    }
    public double pid(double proportion, double integral, double dampening, double distOld, double tOld, double distNew, double tNew){

        double p = distNew * proportion;

        double i = 0;

        r_integralSum += (tNew-tOld)*((distOld+distNew)/2);
        i = r_integralSum * integral;

        double vel = (distNew - distOld)/(tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }
    public double slewLimit(double target, double current, double maxAccelRate, double maxDecelRate, double dt) {
        double error = target - current;
        double rate = (Math.abs(target) < Math.abs(current)) ? maxDecelRate : maxAccelRate;
        double maxChange = rate * dt;

        if (Math.abs(error) <= maxChange) {
            return target;
        }
        return current + Math.signum(error) * maxChange;
    }
}

