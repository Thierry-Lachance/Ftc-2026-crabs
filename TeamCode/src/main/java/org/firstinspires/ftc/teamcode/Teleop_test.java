package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp
public class Teleop_test extends LinearOpMode {
    Servo led;
    Servo s1;
    Servo s2;
    Servo s3;
    Servo s4;
    NormalizedColorSensor c1;
    NormalizedColorSensor c2;
    NormalizedColorSensor c3;
    double hue1;
    double hue2;
    double hue3;
    @Override
    public void runOpMode() throws InterruptedException {
        led = hardwareMap.get(Servo.class, "led");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
        c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
        c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
        // Reading individual RGB values

        // Reading normalized colors (if using NormalizedColorSensor)


        // Converting to HSV for more reliable color detection

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("hue1", hue1);
            telemetry.addData("hue2", hue2);
            telemetry.addData("hue3", hue3);
            telemetry.update();
            NormalizedRGBA colors1 = c1.getNormalizedColors();
            NormalizedRGBA colors2 = c2.getNormalizedColors();
            NormalizedRGBA colors3 = c3.getNormalizedColors();
            hue1 = JavaUtil.colorToHue(colors1.toColor());
            hue2 = JavaUtil.colorToHue(colors2.toColor());
            hue3 = JavaUtil.colorToHue(colors3.toColor());

           if(hue1 > 100 && hue1 < 130){
                s1.setPosition(1);
              } else {
                s1.setPosition(0);
           }
           if(hue2 > 100 && hue2 < 130){
                s2.setPosition(1);
              } else {
               s2.setPosition(0);
           }
              if(hue3 > 80 && hue3 < 1101){
                 s3.setPosition(1);
                  } else {
                  s3.setPosition(0);
              }

            /*for(int i = 0; i <= 100; i++){

                s1.setPosition(i/100.0);
                s2.setPosition(i/100.0);
                s3.setPosition(i/100.0);
                s4.setPosition(i/100.0);
                sleep(10);
            }
            for(int i = 100; i >= 0; i--) {

                s1.setPosition(i / 100.0);
                s2.setPosition(i / 100.0);
                s3.setPosition(i / 100.0);
                s4.setPosition(i / 100.0);
                sleep(10);
            }*/

        }
    }
}
