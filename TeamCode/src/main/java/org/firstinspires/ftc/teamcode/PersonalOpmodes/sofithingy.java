package org.firstinspires.ftc.teamcode.PersonalOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class sofithingy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo rightServo = hardwareMap.get(CRServo.class, "rightServo");
        CRServo leftServo = hardwareMap.get(CRServo.class, "leftServo");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {

            if(gamepad1.a) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

        }
    }
}
