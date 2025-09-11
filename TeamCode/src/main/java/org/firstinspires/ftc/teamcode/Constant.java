package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Constant {

    public enum ColorPattern {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }
    public static ColorPattern gameElementPattern = ColorPattern.UNKNOWN;

    //odo offsets
    public static double xOffset = 0;
    public static double yOffset = 0;

   //Motor Name
    public static String frontLeftMotorName = "fl";
    public static String backLeftMotorName = "bl";
    public static String frontRightMotorName = "fr";
    public static String backRightMotorName = "br";

    public static String shooterMotorName = "s1";
    public static String intakeMotorName = "i1";

    //Servo Name
    public static String gateServoName = "g1";
    public static String angleServoName = "a1";
    public static String chamber1Name = "c1";
    public static String chamber2Name = "c2";
    public static String chamber3Name = "c3";

    //Servo Base Position
    public static double gateServoBasePos = 0.0;
    public static double angleServoBasePos = 0.0;
    public static double chamber1BasePos = 0.0;
    public static double chamber2BasePos = 0.0;
    public static double chamber3BasePos = 0.0;
    //Servo Active Position
    public static double gateServoActivePos = 0.0;
    public static double angleServoActivePos = 0.0;
    public static double chamber1ActivePos = 0.0;
    public static double chamber2ActivePos = 0.0;
    public static double chamber3ActivePos = 0.0;
}
