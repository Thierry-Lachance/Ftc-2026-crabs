package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Teleop_test extends LinearOpMode {

    Servo c1;
    Servo c2;
    Servo c3;
    Servo angle;
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;

    DcMotor intake;
    DcMotor shooter;
   
    @Override
    public void runOpMode() throws InterruptedException {

        c1 = hardwareMap.get(Servo.class, Constant.chamber1Name);
        c2 = hardwareMap.get(Servo.class, Constant.chamber2Name);
        c3 = hardwareMap.get(Servo.class, Constant.chamber3Name);
        angle = hardwareMap.get(Servo.class, Constant.angleServoName);

        fl = hardwareMap.dcMotor.get(Constant.frontLeftMotorName);
        bl = hardwareMap.dcMotor.get(Constant.backLeftMotorName);
        fr = hardwareMap.dcMotor.get(Constant.frontRightMotorName);
        br = hardwareMap.dcMotor.get(Constant.backRightMotorName);
        intake = hardwareMap.dcMotor.get(Constant.intakeMotorName);
        shooter = hardwareMap.dcMotor.get(Constant.shooterMotorName);

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);


            if(gamepad1.x){
                c1.setPosition(Constant.chamber1ActivePos);
            }
            else {
                c1.setPosition(Constant.chamber1BasePos);
            }
            if(gamepad1.y){
                c2.setPosition(Constant.chamber2ActivePos);
            }
            else {
                c2.setPosition(Constant.chamber2BasePos);
            }
            if(gamepad1.b){
                c3.setPosition(Constant.chamber3ActivePos);
            }
            else {
                c3.setPosition(Constant.chamber3BasePos);
            }
            if(gamepad1.a) {
                angle.setPosition(Constant.angleServoActivePos);
            }
            else {
                angle.setPosition(Constant.angleServoBasePos);
            }
            shooter.setPower(gamepad1.right_trigger);
            intake.setPower(gamepad1.left_trigger);
           
        }
    }
}
