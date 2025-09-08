package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class Zero extends LinearOpMode {
    Servo gateServo;
    Servo angleServo;
    Servo chamber1Servo;
    Servo chamber2Servo;
    Servo chamber3Servo;

    @Override
    public void runOpMode() throws InterruptedException {
        gateServo = hardwareMap.get(Servo.class, Constant.gateServoName);
        angleServo = hardwareMap.get(Servo.class, Constant.angleServoName);
        chamber1Servo = hardwareMap.get(Servo.class, Constant.chamber1Name);
        chamber2Servo = hardwareMap.get(Servo.class, Constant.chamber2Name);
        chamber3Servo = hardwareMap.get(Servo.class, Constant.chamber3Name);

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
        gateServo.setPosition(Constant.gateServoBasePos);
        angleServo.setPosition(Constant.angleServoBasePos);
        chamber1Servo.setPosition(Constant.chamber1BasePos);
        chamber2Servo.setPosition(Constant.chamber2BasePos);
        chamber3Servo.setPosition(Constant.chamber3BasePos);

        }
    }
}
