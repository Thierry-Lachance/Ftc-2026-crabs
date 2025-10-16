package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class Base extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    Servo chamber1Servo, chamber2Servo, chamber3Servo;
    DcMotorEx shooter;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_A,
        DRIVE_TO_TARGET_B
    }

    static final Pose2D TARGET_A = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_B = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    boolean shooting = false;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get(Constant.frontLeftMotorName);
        backLeft = hardwareMap.dcMotor.get(Constant.backLeftMotorName);
        frontRight = hardwareMap.dcMotor.get(Constant.frontRightMotorName);
        backRight = hardwareMap.dcMotor.get(Constant.backRightMotorName);

        intake = hardwareMap.dcMotor.get(Constant.intakeMotorName);
        shooter = hardwareMap.get(DcMotorEx.class, Constant.shooterMotorName);

        chamber1Servo = hardwareMap.get(Servo.class, Constant.chamber1Name);
        chamber2Servo = hardwareMap.get(Servo.class, Constant.chamber2Name);
        chamber3Servo = hardwareMap.get(Servo.class, Constant.chamber3Name);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(Constant.xOffset, Constant.yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_TARGET;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            odo.update();
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (stateMachine == StateMachine.WAITING_FOR_TARGET) {
                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
            } else {
                telemetry.addData("Current Pos", odo.getPosition().toString());
               telemetry.addData("power", nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                telemetry.update();
                frontLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            if (gamepad1.b) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_B;
                nav.driveTo(odo.getPosition(), TARGET_B, (Math.abs(Math.sqrt((Math.abs(odo.getPosX()+odo.getPosY()))/2)/80))+0.5, 0);

            }
            else if(gamepad1.a){
                stateMachine = StateMachine.DRIVE_TO_TARGET_A;
                nav.driveTo(odo.getPosition(), TARGET_A, (Math.abs(Math.sqrt((Math.abs(odo.getPosX()+odo.getPosY()))/2)/80))+0.5, 0);
            }

            else {
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
            if(gamepad1.x){
                shooting = true;
            }
            else if(gamepad1.y) {
                shooting = false;
            }
            if(shooting){
                shooter.setVelocity(2650);//max velocity = 2800 at 12V according to motor spec
                if(shooter.getVelocity() > 2600){
                    chamber1Servo.setPosition(Constant.chamber1ActivePos);
                    chamber2Servo.setPosition(Constant.chamber2ActivePos);
                    chamber3Servo.setPosition(Constant.chamber3ActivePos);
                }
                else{
                    chamber1Servo.setPosition(Constant.chamber1BasePos);
                    chamber2Servo.setPosition(Constant.chamber2BasePos);
                    chamber3Servo.setPosition(Constant.chamber3BasePos);
                }
            }
            else{
                shooter.setVelocity(0);//max velocity = 2800 at 12V according to motor spec
            }

            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            if(!shooting){
                if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                    chamber1Servo.setPosition(Constant.chamber1BasePos);
                    chamber2Servo.setPosition(Constant.chamber2BasePos);
                    chamber3Servo.setPosition(Constant.chamber3BasePos);
                }
                else {
                    chamber1Servo.setPosition(Constant.chamber1EngagedPos);
                    chamber2Servo.setPosition(Constant.chamber2EngagedPos);
                    chamber3Servo.setPosition(Constant.chamber3EngagedPos);
                }
            }




        }
    }
}