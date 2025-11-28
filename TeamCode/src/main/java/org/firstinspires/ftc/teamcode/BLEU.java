package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp
public class BLEU extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    Servo chamber1Servo, chamber2Servo, chamber3Servo;
    DcMotorEx shooter;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_X,
        DRIVE_TO_TARGET_Y,
        DRIVE_TO_TARGET_B
    }

    static final Pose2D TARGET_X = new Pose2D(DistanceUnit.INCH, 60, -20, AngleUnit.DEGREES, -150);
    static final Pose2D TARGET_Y = new Pose2D(DistanceUnit.INCH, 88.5, -92, AngleUnit.DEGREES, -130);
    static final Pose2D TARGET_B = new Pose2D(DistanceUnit.INCH, 61.8, -71, AngleUnit.DEGREES, -135);
    Pose2D CurrentTarget = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    boolean shooting = false;

    double shooterPower = -1000;

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
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                telemetry.update();
                frontLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeft.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRight.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 105, -119, AngleUnit.RADIANS, 2.477));
            }
            if (gamepad1.y) {
                shooter.setVelocityPIDFCoefficients(250, 2, 2, 0.0);
                shooterPower = Constant.shooterPowerY;
                stateMachine = BLEU.StateMachine.DRIVE_TO_TARGET_Y;
                CurrentTarget = TARGET_Y;
                nav.driveTo(odo.getPosition(), TARGET_Y, 0.4 + 3 * (gamepad1.right_trigger / 5), 0);

            } else if (gamepad1.x) {
                shooter.setVelocityPIDFCoefficients(250, 1, 2, 0.0);
                shooterPower = Constant.shooterPowerX;
                stateMachine = BLEU.StateMachine.DRIVE_TO_TARGET_X;
                CurrentTarget = TARGET_X;
                nav.driveTo(odo.getPosition(), TARGET_X, 0.4 + 3 * (gamepad1.right_trigger / 5), 0);

            } else if (gamepad1.b) {
                shooter.setVelocityPIDFCoefficients(250, 2, 2, 0.0);
                shooterPower = Constant.shooterPowerB;
                stateMachine = BLEU.StateMachine.DRIVE_TO_TARGET_B;
                CurrentTarget = TARGET_B;
                nav.driveTo(odo.getPosition(), TARGET_B, 0.4 + 3 * (gamepad1.right_trigger / 5), 0);
            } else {
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
            if (gamepad2.x) {
                shooting = true;


            } else if (gamepad2.y) {
                shooting = false;
                shooter.setVelocity(0);//max velocity = 2800 at 12V according to motor spec
                chamber1Servo.setPosition(Constant.chamber1EngagedPos);
                chamber2Servo.setPosition(Constant.chamber2EngagedPos);
                chamber3Servo.setPosition(Constant.chamber3EngagedPos);
            }

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            if (shooting) {
                shooter.setVelocity(shooterPower);//max velocity = 2800 at 12V according to motor spec
                if (shooter.getVelocity() <= shooterPower + 30 && shooter.getVelocity() >= shooterPower - 30 &&
                        Math.abs(odo.getPosX() - CurrentTarget.getX(DistanceUnit.MM)) < 30 &&
                        Math.abs(odo.getPosY() - CurrentTarget.getY(DistanceUnit.MM)) < 30 &&
                        Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - CurrentTarget.getHeading(AngleUnit.RADIANS)) < 0.1 &&
                        Math.abs(odo.getVelX()) < 75.0 &&
                        Math.abs(odo.getVelY()) < 75.0) {
                    telemetry.addLine("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH");
                    if (gamepad2.dpad_left) {
                        chamber1Servo.setPosition(Constant.chamber1ActivePos);

                    } else if (gamepad2.dpad_up) {
                        chamber2Servo.setPosition(Constant.chamber2ActivePos);

                    } else if (gamepad2.dpad_right) {
                        chamber3Servo.setPosition(Constant.chamber3ActivePos);

                    }


                }
            } else {
                if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {

                    chamber1Servo.setPosition(Constant.chamber1BasePos);
                    chamber2Servo.setPosition(Constant.chamber2BasePos);
                    chamber3Servo.setPosition(Constant.chamber3BasePos);
                } else {
                    chamber1Servo.setPosition(Constant.chamber1EngagedPos);
                    chamber2Servo.setPosition(Constant.chamber2EngagedPos);
                    chamber3Servo.setPosition(Constant.chamber3EngagedPos);
                }
            }
            if(gamepad1.dpadUpWasReleased()){
            odo.setPosition(new Pose2D(DistanceUnit.INCH, odo.getPosition().getX(DistanceUnit.INCH) - 5*Math.cos(odo.getPosition().getHeading(AngleUnit.RADIANS)), odo.getPosition().getY(DistanceUnit.INCH) - 5*Math.sin(odo.getPosition().getHeading(AngleUnit.RADIANS)), AngleUnit.RADIANS, odo.getPosition().getHeading(AngleUnit.RADIANS)));
            }
            if(gamepad1.dpadDownWasReleased()){
                odo.setPosition(new Pose2D(DistanceUnit.INCH, odo.getPosition().getX(DistanceUnit.INCH) + 5*Math.cos(odo.getPosition().getHeading(AngleUnit.RADIANS)), odo.getPosition().getY(DistanceUnit.INCH) + 5*Math.sin(odo.getPosition().getHeading(AngleUnit.RADIANS)), AngleUnit.RADIANS, odo.getPosition().getHeading(AngleUnit.RADIANS)));
            }
            if(gamepad1.dpadLeftWasReleased()){
                odo.setPosition(new Pose2D(DistanceUnit.INCH, odo.getPosition().getX(DistanceUnit.INCH) - 5*Math.cos(odo.getPosition().getHeading(AngleUnit.RADIANS)+Math.PI/2), odo.getPosition().getY(DistanceUnit.INCH) - 5*Math.sin(odo.getPosition().getHeading(AngleUnit.RADIANS)+Math.PI/2), AngleUnit.RADIANS, odo.getPosition().getHeading(AngleUnit.RADIANS)));
            }
            if(gamepad1.dpadRightWasReleased()){
                odo.setPosition(new Pose2D(DistanceUnit.INCH, odo.getPosition().getX(DistanceUnit.INCH) + 5*Math.cos(odo.getPosition().getHeading(AngleUnit.RADIANS)+Math.PI/2), odo.getPosition().getY(DistanceUnit.INCH) + 5*Math.sin(odo.getPosition().getHeading(AngleUnit.RADIANS)+Math.PI/2), AngleUnit.RADIANS, odo.getPosition().getHeading(AngleUnit.RADIANS)));
            }


            telemetry.addData("speed", shooter.getVelocity());
            telemetry.addData("current state:", stateMachine);
            telemetry.addData("s1", shooter.getVelocity() <= shooterPower + 30);
            telemetry.addData("s2", shooter.getVelocity() >= shooterPower - 30);
            telemetry.addData("x", Math.abs(odo.getPosX() - CurrentTarget.getX(DistanceUnit.MM)) < 30);
            telemetry.addData("y", Math.abs(odo.getPosY() - CurrentTarget.getY(DistanceUnit.MM)) < 30);
            telemetry.addData("deg", Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - CurrentTarget.getHeading(AngleUnit.RADIANS)) < 0.1);
            telemetry.addData("deg", Math.abs(odo.getPosition().getHeading(AngleUnit.RADIANS) - CurrentTarget.getHeading(AngleUnit.RADIANS)));
            telemetry.addData("deg 1", odo.getPosition().getHeading(AngleUnit.RADIANS));
            telemetry.addData("deg 2", CurrentTarget.getHeading(AngleUnit.RADIANS));
            telemetry.addData("vel X", odo.getVelX());
            telemetry.addData("vel Y", odo.getVelY());
            telemetry.addData("vel R", odo.getHeadingVelocity());



            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("ROBOT Position", data);
            String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", CurrentTarget.getX(DistanceUnit.INCH), CurrentTarget.getY(DistanceUnit.INCH), CurrentTarget.getHeading(AngleUnit.RADIANS));
            telemetry.addData("TARGET Position", data2);
            telemetry.update();


        }
    }
}