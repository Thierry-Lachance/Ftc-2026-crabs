package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@TeleOp
public class Zero extends LinearOpMode {
    Servo gateServo;
   // Servo angleServo;
    Servo chamber1Servo;
    Servo chamber2Servo;
    Servo chamber3Servo;
    Servo shooterServo;


    @Override
    public void runOpMode() throws InterruptedException {

       // angleServo = hardwareMap.get(Servo.class, Constant.angleServoName);
        chamber1Servo = hardwareMap.get(Servo.class, Constant.chamber1Name);
      chamber2Servo = hardwareMap.get(Servo.class, Constant.chamber2Name);
       chamber3Servo = hardwareMap.get(Servo.class, Constant.chamber3Name);
        shooterServo = hardwareMap.get(Servo.class, Constant.angleServoName);
        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

      //  angleServo.setPosition(Constant.angleServoBasePos);

       //
            if(gamepad1.x){
                chamber1Servo.setPosition(Constant.chamber1ActivePos);
                chamber2Servo.setPosition(Constant.chamber2ActivePos);
                 chamber3Servo.setPosition(Constant.chamber3ActivePos);
                 shooterServo.setPosition(Constant.angleServoActivePos);
            }
else{
    chamber1Servo.setPosition(Constant.chamber1BasePos);
                chamber2Servo.setPosition(Constant.chamber2BasePos);
                 chamber3Servo.setPosition(Constant.chamber3BasePos);
                 shooterServo.setPosition(Constant.angleServoBasePos);
}


        }
    }
}
