package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "NathanOpModeAdvanced")
public class NathanOpModeAdvanced extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean isShooting = false;
    double flywheelVel;
    double targetFlywheelVel = 1240;
    double oldFlywheelDist = 0;
    double oldTime = 0;
    boolean isIntakeRunning = true;
    boolean lastB = false;
    boolean lastC = false;
    boolean angleSnapping = false;
    boolean lastD = false;
    boolean intakeIsOn;

    double f_integralSum;
    public static double f_kD = 0.05, f_kI = 0, f_kP = 0.009;

    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    double yaw;
    double axial;
    double lateral;
    boolean lastA;
    boolean fieldToggle;
    double y;
    double x;
    double rotCheck;
    double heading;
    double Degrees;
    double moveSpeed = 1;

    DcMotor frontleft;
    DcMotor backleft;
    DcMotor frontright;
    DcMotor backright;
    DcMotorEx flywheel;
    DcMotorEx intake;
    CRServo right_intake_servo;
    CRServo left_intake_servo;
    CRServo windmill;
    IMU imu;
    OctoQuad octo;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windmill = hardwareMap.get(CRServo.class, "windmill");
        right_intake_servo = hardwareMap.get(CRServo.class, "rightServo");
        left_intake_servo = hardwareMap.get(CRServo.class, "leftServo");
        octo = hardwareMap.get(OctoQuad.class, "octo");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        right_intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
        left_intake_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        if (Math.abs(gamepad1.left_stick_y) < 0.05 &&
                Math.abs(gamepad1.left_stick_x) < 0.05 && Math.abs(gamepad1.right_stick_x) < 0.05) {

            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        imu = hardwareMap.get(IMU.class, "imu");

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        imu.resetYaw();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
            heading = angle.getYaw(AngleUnit.RADIANS);
            Degrees = angle.getYaw(AngleUnit.DEGREES);
            flywheelVel = -flywheel.getVelocity();
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            rotCheck = gamepad1.right_stick_x;


            //------------------------------------Movement----------------------------------------


            telemetry.addLine("Press Left Stick or dpad down to change move speed");
            if (gamepad1.leftStickButtonWasPressed() || gamepad1.dpadDownWasPressed()) {
                if (moveSpeed == 1) {
                    moveSpeed = 0.5;
                } else {
                    moveSpeed = 1;
                }
            }

            telemetry.addLine("Press dpad left to reset imu");
            if (gamepad1.dpad_left) {
                imu.resetYaw();
            }

            telemetry.addLine("Press dpad up to toggle field centric");
            if (gamepad1.dpad_up && !lastA) {
                fieldToggle = !fieldToggle;
            }
            lastA = gamepad1.dpad_up;

            if (fieldToggle) {
                axial = y * cos(-heading) - x * sin(-heading);
                lateral = y * sin(-heading) + x * cos(-heading);
                telemetry.addLine("Field Centric Toggled: On");
            } else {
                axial = y;
                lateral = x;
                telemetry.addLine("Field Centric Toggled: Off");
            }

            telemetry.addLine("Press 'X' to toggle angle snapping");
            if (gamepad1.x && !lastC) {
                angleSnapping = !angleSnapping;
            }
            lastC = gamepad1.x;

            if (angleSnapping) {
                    if (AngleUnit.normalizeDegrees(0 - Degrees) >= 5 && AngleUnit.normalizeDegrees(0 - Degrees) <= 5 && rotCheck == 0) {
                    yaw = 0 - Degrees;
                }
                else if (AngleUnit.normalizeDegrees(90 - Degrees) >= 5 && AngleUnit.normalizeDegrees(90 - Degrees) <= 5 && rotCheck == 0) {
                    yaw = 90 - Degrees;
                }
                else if (AngleUnit.normalizeDegrees(-90 - Degrees) >= 5 && AngleUnit.normalizeDegrees(-90 - Degrees) <= 5 && rotCheck == 0) {
                    yaw = -90 - Degrees;
                }
                else if (AngleUnit.normalizeDegrees(180 - Degrees) >= 5 && AngleUnit.normalizeDegrees(180 - Degrees) <= 5 && rotCheck == 0) {
                    yaw = 180 - Degrees;
                }
            telemetry.addLine("Angle snapping: On");
            }
            else {
                telemetry.addLine("Angle snapping: Off");
            }

            frontRightPower = (axial + yaw + lateral) * moveSpeed;
            frontLeftPower = (axial - yaw - lateral) * moveSpeed;
            backRightPower = (axial + yaw - lateral) * moveSpeed;
            backLeftPower = (axial - yaw + lateral) * moveSpeed;

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


            telemetry.addLine("Press right and left bumper to increase or decrease target flywheel velocity");
            if (gamepad1.rightBumperWasPressed()) {
                targetFlywheelVel += 10;
            }
            if (gamepad1.leftBumperWasPressed()) {
                targetFlywheelVel -= 10;
            }

            telemetry.addLine("Press 'A' to shoot");
            if (gamepad1.a && !lastB) {
                isShooting = !isShooting;
            }
            lastB = gamepad1.a;

            if(isShooting) {

                shootTimer.reset();

                flywheel.setPower(pid(f_kP, f_kI, f_kD, oldFlywheelDist, oldTime, targetFlywheelVel - flywheelVel, runtime.milliseconds()));

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

            oldTime = runtime.milliseconds();
            oldFlywheelDist = targetFlywheelVel - flywheelVel;


            //-------------------------------intake--------------------------------


            telemetry.addLine("Press 'B' to power intake");
            if (gamepad1.b && !lastD && isIntakeRunning) {
            intakeIsOn = !intakeIsOn;
            }
            lastD = gamepad1.b;

            if (intakeIsOn) {
                left_intake_servo.setPower(1);
                right_intake_servo.setPower(1);
                intake.setPower(1);
            }
            else {
                left_intake_servo.setPower(0);
                right_intake_servo.setPower(0);
                intake.setPower(0);
            }


            //---------------------telemetry--------------------
            telemetry.addData("Movement speed: ", moveSpeed);
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

        f_integralSum += (tNew-tOld)*((distOld+distNew)/2);
        i = f_integralSum * integral;

        double vel = (distNew - distOld)/(tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }
}

