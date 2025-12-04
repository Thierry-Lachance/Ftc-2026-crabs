package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "AutoRed", group = "Autonomous")


public class AutoRed extends LinearOpMode {

    static final Pose2D[] TARGET = {
            new Pose2D(DistanceUnit.MM, -1000, -600, AngleUnit.DEGREES, -45), // shooting position
            new Pose2D(DistanceUnit.MM, -1000, -750, AngleUnit.DEGREES, 0),   // correct angle to pick ball 2
            new Pose2D(DistanceUnit.MM, -200, -700, AngleUnit.DEGREES, 0), // pos after picking ball 1
            new Pose2D(DistanceUnit.MM, -50, -900, AngleUnit.DEGREES, 0),         // open gate
            new Pose2D(DistanceUnit.MM, -1000, -1300, AngleUnit.DEGREES, 0), // pos before intake ball 2
            new Pose2D(DistanceUnit.MM, -200, -1300, AngleUnit.DEGREES, 0),  // pos after intake ball 2
            new Pose2D(DistanceUnit.MM, -1000, -1850, AngleUnit.DEGREES, 0),// pos before intake 3
            new Pose2D(DistanceUnit.MM, -200, -1850, AngleUnit.DEGREES, 0), // pos after intake ball 3
            new Pose2D(DistanceUnit.MM, -200, -1850, AngleUnit.DEGREES, 0)};// offside


    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_SHOOT_1,
        DRIVE_TO_PRE_INTAKE_1,
        DRIVE_TO_POST_INTAKE_1,
        DRIVE_TO_POST_GATE_1,
        DRIVE_TO_SHOOT_2,
        DRIVE_TO_PRE_INTAKE_2,
        DRIVE_TO_POST_INTAKE_2,
        DRIVE_TO_SHOOT_3,
        DRIVE_TO_PRE_INTAKE_3,
        DRIVE_TO_POST_INTAKE_3,
        DRIVE_TO_SHOOT_4,
        DRIVE_TO_OFFSIDE
    }

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this);
    double speed = 0.7;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.dcMotor.get(Constant.frontLeftMotorName);
        leftBackDrive = hardwareMap.dcMotor.get(Constant.backLeftMotorName);
        rightFrontDrive = hardwareMap.dcMotor.get(Constant.frontRightMotorName);
        rightBackDrive = hardwareMap.dcMotor.get(Constant.backRightMotorName);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(Constant.xOffset, Constant.yOffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START://up the slider
                    stateMachine = StateMachine.DRIVE_TO_SHOOT_1;
                    break;
                case DRIVE_TO_SHOOT_1:
                    if (nav.driveTo(odo.getPosition(), TARGET[0], speed, 1.0)) {
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_PRE_INTAKE_1;

                    }


                    break;
                case DRIVE_TO_PRE_INTAKE_1:
                    if (nav.driveTo(odo.getPosition(), TARGET[1], speed, 1.0)) {
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_POST_INTAKE_1;

                    }

                    break;
                case DRIVE_TO_POST_INTAKE_1:
                    if (nav.driveTo(odo.getPosition(), TARGET[2], speed, 1.0)) {
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_POST_GATE_1;

                    }

                    break;

                case DRIVE_TO_POST_GATE_1:
                    if (nav.driveTo(odo.getPosition(), TARGET[3], speed, 1.0)) {
                        telemetry.addLine("at position #4");

                        stateMachine = StateMachine.DRIVE_TO_SHOOT_2;
                    }

                    break;
                case DRIVE_TO_SHOOT_2:
                    if (nav.driveTo(odo.getPosition(), TARGET[0], speed, 1.0)) {
                        telemetry.addLine("at position #5");
                        stateMachine = StateMachine.DRIVE_TO_PRE_INTAKE_2;
                    }
                    break;

                case DRIVE_TO_PRE_INTAKE_2:
                    if (nav.driveTo(odo.getPosition(), TARGET[4], speed, 1.0)) {
                        telemetry.addLine("at position #6");

                        stateMachine = StateMachine.DRIVE_TO_POST_INTAKE_2;
                    }

                    break;
                case DRIVE_TO_POST_INTAKE_2:
                    if (nav.driveTo(odo.getPosition(), TARGET[5], speed, 1.0)) {// stop the arm
                        telemetry.addLine("at position #7");

                        stateMachine = StateMachine.DRIVE_TO_SHOOT_3;
                    }

                    break;

                case DRIVE_TO_SHOOT_3:
                    if (nav.driveTo(odo.getPosition(), TARGET[0], speed, 1.0)) {
                        telemetry.addLine("at position #8");

                        stateMachine = StateMachine.DRIVE_TO_PRE_INTAKE_3;

                    }

                    break;
                case DRIVE_TO_PRE_INTAKE_3:
                    if (nav.driveTo(odo.getPosition(), TARGET[6], speed, 1.0)) {
                        telemetry.addLine("at position #9");

                        stateMachine = StateMachine.DRIVE_TO_POST_INTAKE_3;
                    }
                    break;
                case DRIVE_TO_POST_INTAKE_3:
                    if (nav.driveTo(odo.getPosition(), TARGET[7], speed, 1.0)) {
                        telemetry.addLine("at position #10");

                        stateMachine = StateMachine.DRIVE_TO_SHOOT_4;


                    }
                    break;
                case DRIVE_TO_SHOOT_4:
                    if (nav.driveTo(odo.getPosition(), TARGET[0], speed, 1.0)) {// stop the arm
                        telemetry.addLine("at position #10");

                        stateMachine = StateMachine.DRIVE_TO_OFFSIDE;


                    }
                    break;
                case DRIVE_TO_OFFSIDE:
                    if (nav.driveTo(odo.getPosition(), TARGET[8], speed, 1.0)) {// stop the arm
                        telemetry.addLine("at position #10");

                        stateMachine = StateMachine.AT_TARGET;


                    }
                    break;
            }

            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            telemetry.addData("current state:", stateMachine);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
        }


    }

    public void stopWheel() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }


}