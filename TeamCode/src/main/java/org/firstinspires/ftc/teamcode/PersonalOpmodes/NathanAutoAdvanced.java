package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name ="NathanAutoAdvanced")
public class NathanAutoAdvanced extends LinearOpMode {
    // I state my variables up here so they are within the scope of all my methods
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor intake;
    DcMotorEx flywheel;
    CRServo windmill;
    CRServo left_intake_servo;
    CRServo right_intake_servo;
    IMU imu;
    double heading;
    double Degrees;
    double yaw = 0;
    double axial = 0;
    double lateral = 0;
    boolean isMoving = false;
    boolean shooting = false;
    double frontLeftPower = 0;
    double frontRightPower = 0;
    double backLeftPower = 0;
    double backRightPower = 0;
    double currentVel;
    double f_integralSum;
    public static double f_kD = 0.05, f_kI = 0, f_kP = 0.009;
    double oldflywheelDist = 0;
    double oldtime = 0;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime shooterTimer = new ElapsedTime();
    ElapsedTime movetime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backright = hardwareMap.get(DcMotor.class, "backright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windmill = hardwareMap.get(CRServo.class, "windmill");
        right_intake_servo = hardwareMap.get(CRServo.class, "rightServo");
        left_intake_servo = hardwareMap.get(CRServo.class, "leftServo");
        imu = hardwareMap.get(IMU.class, "imu");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);

        if (frontRightPower == 0 && frontLeftPower == 0 && backLeftPower == 0 && backRightPower == 0) {

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

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        imu.resetYaw();
        waitForStart();

        shoot(1180, 5);
        sleep(100);
        move(1,1,0,0.1,1);
        sleep(100);
        move(-1,1,15,0.2,1);
        intake(1.5);
        sleep(100);
        move(6,1,0,0.1,1);
        sleep(100);
        move(5,1,-90,1,1);
        sleep(100);

        while (opModeIsActive()) {
            telemetry.addData("Runtime: ", runtime);
            telemetry.addData("flywheel Vel: ", currentVel);
            telemetry.update();
        }
    }
    public void move(double y, double x, double targetAngle, double rotateSpeed, double power) throws InterruptedException {

        movetime.reset();
        isMoving = true;

        double distance = Math.hypot(y,x) / 39.37;

// I need odometry pods for this, so this could be used in future iterations

        time = distance / currentVel;

        time *= (1/power);

        while (opModeIsActive() && isMoving) {

            YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
            heading = angle.getYaw(AngleUnit.RADIANS);
            Degrees = angle.getYaw(AngleUnit.DEGREES);

            axial = y * cos(-heading) - x * sin(-heading);
            lateral = y * sin(-heading) + x * cos(-heading);

            double error = AngleUnit.normalizeDegrees(targetAngle - Degrees);

            yaw = Math.max(-1.0, Math.min(1.0, error * rotateSpeed));

        /*
        This is where I state all the variables for movement in an array like pattern for mechanum wheels.
        Remember the forward and sideways movement are not controlled directly from the variables that can be
        stated in the method, but in the field centric math.
         */

            frontLeftPower = (axial - lateral + yaw) * power;
            frontRightPower = (axial + lateral - yaw) * power;
            backLeftPower = (axial + lateral + yaw) * power;
            backRightPower = (axial - lateral - yaw) * power;

            //Makes sure power does not exceed 1 or -1
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

            if (movetime.seconds() < time) {
                isMoving = false;
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
        }
    }
    public void intake(double time) {
        //sets power to motors for a given amount of time
        intake.setPower(1);
        right_intake_servo.setPower(-1);
        left_intake_servo.setPower(1);

        sleep((long) (time * 1000));

        intake.setPower(0);
        right_intake_servo.setPower(0);
        left_intake_servo.setPower(0);
    }
    public void shoot(double targetVel, double time) {

        shooting = true;
        shooterTimer.reset();

        while (opModeIsActive() && shooting) {

            currentVel = flywheel.getVelocity();

            flywheel.setPower(pid(f_kP, f_kI, f_kD, oldflywheelDist, oldtime, targetVel - currentVel, runtime.milliseconds()));

            if (targetVel - currentVel >= -80 && targetVel - currentVel <= 10) {
                windmill.setPower(1);
            }
            else {
                windmill.setPower(0);
            }

            oldtime = runtime.milliseconds();
            oldflywheelDist = targetVel - currentVel;

            if(shooterTimer.seconds() > time) {
                shooting = false;
                flywheel.setPower(0);
                windmill.setPower(0);
            }
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



