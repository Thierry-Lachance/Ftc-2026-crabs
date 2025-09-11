package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
@TeleOp
public class TeleopRed extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class


    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor shooterMotor;
    DcMotor intakeMotor;

    Servo gateServo;
    Servo angleServo;
    Servo chamber1Servo;
    Servo chamber2Servo;
    Servo chamber3Servo;


    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_A,
        DRIVE_TO_TARGET_B
    }


    int ballCount = 0;
    Constant.ColorPattern shootingOrder = Constant.ColorPattern.UNKNOWN;

    static final Pose2D TARGET_A = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_B = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get(Constant.frontLeftMotorName);
        backLeftMotor = hardwareMap.dcMotor.get(Constant.backLeftMotorName);
        frontRightMotor = hardwareMap.dcMotor.get(Constant.frontRightMotorName);
        backRightMotor = hardwareMap.dcMotor.get(Constant.backRightMotorName);

        shooterMotor = hardwareMap.dcMotor.get(Constant.shooterMotorName);
        intakeMotor = hardwareMap.dcMotor.get(Constant.intakeMotorName);

        gateServo = hardwareMap.get(Servo.class, Constant.gateServoName);
        angleServo = hardwareMap.get(Servo.class, Constant.angleServoName);
        chamber1Servo = hardwareMap.get(Servo.class, Constant.chamber1Name);
        chamber2Servo = hardwareMap.get(Servo.class, Constant.chamber2Name);
        chamber3Servo = hardwareMap.get(Servo.class, Constant.chamber3Name);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(Constant.xOffset, Constant.yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_TARGET;

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();
            odo.update();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            double up = (double) (gamepad1.dpad_up ? 1 : 0) / 2;
            double down = (double) (gamepad1.dpad_down ? 1 : 0) / 2;
            double left = (double) (gamepad1.dpad_left ? 1 : 0) / 2;
            double right = (double) (gamepad1.dpad_right ? 1 : 0) / 2;

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
                frontLeftMotor.setPower(frontLeftPower + up - down - left + right);
                backLeftMotor.setPower(backLeftPower + up - down + left - right);
                frontRightMotor.setPower(frontRightPower + up - down + left - right);
                backRightMotor.setPower(backRightPower + up - down - left + right);
            } else {
                frontLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            //***********************DRIVING LOGIC************************//

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            if (gamepad1.b) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_B;
                nav.driveTo(odo.getPosition(), TARGET_B, 0.5, 0);

            } else if (gamepad1.a) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_A;
                nav.driveTo(odo.getPosition(), TARGET_A, 0.5, 0);
            } else {
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
            //***********************COLOR TRACKING LOGIC************************//
            if (gamepad2.right_bumper) {
                resetBallCount();
            }
            if (gamepad2.x) {
                incrementBallCount();
            }
            if (gamepad2.a) {
                decrementBallCount();
            }
            //***********************SHOOTING LOGIC************************//
            if (gamepad1.y) {
                calculateShootingOrder();
                shooterMotor.setPower(1.0);// full power
                //adjust angle
                //adjust position
                //shoot first ball
                //adjust pos etc
            } else {
                shooterMotor.setPower(0);
            }
            //************************INTAKE LOGIC***********************//
            if(gamepad1.right_trigger > 0.1){
                intakeMotor.setPower(1.0);
                gateServo.setPosition(Constant.gateServoActivePos);
            }
            else{
                intakeMotor.setPower(0);
                gateServo.setPosition(Constant.gateServoBasePos);
            }
        }
    }

    void resetBallCount() {
        ballCount = 0;
    }

    void incrementBallCount() {
        ballCount++;
    }

    void decrementBallCount() {
        if (ballCount > 0) {
            ballCount--;
        }
    }

    int findColorGreenInChamber() {
        return 0;
    }
    int findColorPurpleInChamber() {
        return 0;
    }

    void calculateShootingOrder() {
        int patternStart = ballCount % 3; // 0,1,2
        switch (Constant.gameElementPattern) {
            case GPP:
                switch (patternStart) {
                    case 0:
                        //shoot gpp
                        shootingOrder = Constant.ColorPattern.GPP;
                        break;
                    case 1:
                        //shoot ppg
                        shootingOrder = Constant.ColorPattern.PPG;
                        break;
                    case 2:
                        //shoot pgp
                        shootingOrder = Constant.ColorPattern.PGP;
                        break;
                }
                break;
            case PGP:
                switch (patternStart) {
                    case 0:
                        //shoot pgp
                        shootingOrder = Constant.ColorPattern.PGP;
                        break;
                    case 1:
                        //shoot gpp
                        shootingOrder = Constant.ColorPattern.GPP;
                        break;
                    case 2:
                        //shoot ppg
                        shootingOrder = Constant.ColorPattern.PPG;
                        break;
                }

                break;
            case PPG:
                switch (patternStart) {
                    case 0:
                        //shoot ppg
                        shootingOrder = Constant.ColorPattern.PPG;
                        break;
                    case 1:
                        //shoot pgp
                        shootingOrder = Constant.ColorPattern.PGP;
                        break;
                    case 2:
                        //shoot gpp
                        shootingOrder = Constant.ColorPattern.GPP;
                        break;
                }

                break;
            case UNKNOWN:
                shootingOrder = Constant.ColorPattern.UNKNOWN;

                break;
        }


    }
}   // end class