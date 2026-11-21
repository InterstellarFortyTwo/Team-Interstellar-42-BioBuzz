package org.firstinspires.ftc.teamcode.CompCode;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name = "CompAuto", group = "CompCodes")
public class CompAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean isShooting = false;
    double flywheelVel;
    double targetFlywheelVel = 1110;
    double oldFlywheelDist = 0;
    double oldTime = 0;
    boolean isIntakeRunning = true;
    boolean lastB = false;
    boolean lastC = false;
    boolean angleSnapping = false;
    boolean lastD = false;
    boolean intakeIsOn = false;

    // Edge detection tracking variables for buttons
    boolean lastLeftStickButton = false;
    boolean lastYButton = false;
    boolean lastRightBumper = false;
    boolean lastLeftBumper = false;

    double f_integralSum;
    public static double f_kD = 0.007, f_kI = 0, f_kP = 0.012;

    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    double yaw;
    double axial;
    double lateral;
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
    //CRServo right_intake_servo;
    Servo left_intake_servo; // Changed to standard Servo for position control
    CRServo windmill;
    IMU imu;

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
        //right_intake_servo = hardwareMap.get(CRServo.class, "rightServo");
        left_intake_servo = hardwareMap.get(Servo.class, "leftServo");

        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_intake_servo.setDirection(CRServo.Direction.REVERSE);

        if (Math.abs(gamepad1.left_stick_y) < 0.05 &&
                Math.abs(gamepad1.left_stick_x) < 0.05 && Math.abs(gamepad1.right_stick_x) < 0.05) {

            frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
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

            telemetry.addLine("Press Left Stick Button to change move speed");
            boolean currentLeftStick = gamepad1.left_stick_button;
            if (currentLeftStick && !lastLeftStickButton) {
                if (moveSpeed == 1) {
                    moveSpeed = 0.5;
                } else {
                    moveSpeed = 1;
                }
            }
            lastLeftStickButton = currentLeftStick;

            telemetry.addLine("Press dpad left to reset imu");
            if (gamepad1.dpad_left) {
                imu.resetYaw();
            }

            telemetry.addLine("Press 'Y' to toggle field centric");
            boolean currentY = gamepad1.y;
            if (currentY && !lastYButton) {
                fieldToggle = !fieldToggle;
            }
            lastYButton = currentY;

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
                if (rotCheck == 0) {
                    if (Math.abs(AngleUnit.normalizeDegrees(0 - Degrees)) <= 5) {
                        yaw = AngleUnit.normalizeDegrees(0 - Degrees);
                    } else if (Math.abs(AngleUnit.normalizeDegrees(90 - Degrees)) <= 5) {
                        yaw = AngleUnit.normalizeDegrees(90 - Degrees);
                    } else if (Math.abs(AngleUnit.normalizeDegrees(-90 - Degrees)) <= 5) {
                        yaw = AngleUnit.normalizeDegrees(-90 - Degrees);
                    } else if (Math.abs(AngleUnit.normalizeDegrees(180 - Degrees)) <= 5) {
                        yaw = AngleUnit.normalizeDegrees(180 - Degrees);
                    }
                }
                telemetry.addLine("Angle snapping: On");
            } else {
                telemetry.addLine("Angle snapping: Off");
            }

            frontRightPower = (axial + yaw + lateral);
            frontLeftPower = (axial - yaw - lateral);
            backRightPower = (axial + yaw - lateral);
            backLeftPower = (axial - yaw + lateral);

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
            } else {
                frontleft.setPower(frontLeftPower * moveSpeed);
                frontright.setPower(frontRightPower * moveSpeed);
                backleft.setPower(backLeftPower * moveSpeed);
                backright.setPower(backRightPower * moveSpeed);
            }

            //-----------------------------------Shooting---------------------------------------

            telemetry.addLine("Press right and left bumper to increase or decrease target flywheel velocity");
            boolean currentRightBumper = gamepad1.right_bumper;
            if (currentRightBumper && !lastRightBumper) {
                targetFlywheelVel += 10;
            }
            lastRightBumper = currentRightBumper;

            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                targetFlywheelVel -= 10;
            }
            lastLeftBumper = currentLeftBumper;

            telemetry.addLine("Press 'A' to shoot");
            if (gamepad1.a && !lastB) {
                isShooting = !isShooting;
            }
            lastB = gamepad1.a;

            if (isShooting) {
                shootTimer.reset();

                flywheel.setPower(pid(f_kP, f_kI, f_kD, oldFlywheelDist, oldTime, targetFlywheelVel - flywheelVel, runtime.milliseconds()));

                if (targetFlywheelVel - flywheelVel >= -80 && targetFlywheelVel - flywheelVel <= 10) {
                    windmill.setPower(1);
                } else {
                    windmill.setPower(0);
                }
            } else {
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
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            //-------------------------------Intake & Reverse--------------------------------

            telemetry.addLine("Press 'B' to toggle intake, hold 'Y' to reverse intake");

            // Intake control: 'Y' reverses intake, 'B' toggles forward intake, otherwise off
            if (gamepad1.y) {
                intake.setPower(-1.0); // Run intake in reverse
            } else if (intakeIsOn) {
                intake.setPower(1.0);  // Run intake forward
            } else {
                intake.setPower(0.0);  // Stop intake
            }

            //------------------Servo Control intake (dpad_up / dpad_down)---------------
            telemetry.addLine("Press dpad up/down to rotate left_intake_servo 180 deg");
            if (gamepad1.dpad_up) {
                left_intake_servo.setPosition(1.0); // 180 degrees clockwise position
            } else if (gamepad1.dpad_down) {
                left_intake_servo.setPosition(0.0); // 180 degrees counter-clockwise position
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

        f_integralSum += (tNew - tOld) * ((distOld + distNew) / 2);
        double i = f_integralSum * integral;

        double vel = (distNew - distOld) / (tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }
}


