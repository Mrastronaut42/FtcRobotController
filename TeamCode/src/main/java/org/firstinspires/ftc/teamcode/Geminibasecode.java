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

@TeleOp(name = "TeleOp_Mecanum", group = "TeleOp")
public class Geminibasecode extends LinearOpMode {

    // Servo Constants
    private static final double FEEDER_REST_POS = 0.0;
    private static final double FEEDER_PUSH_POS = 0.5;
    private static final double GATE_CLOSED_POS = 0.0;
    private static final double GATE_OPEN_POS   = 0.5;

    // Dual Flywheel Target Velocity (Ticks/Second)
    private static final double SHOOTER_TARGET_VELOCITY = 2200.0;

    // Driver Control Variables
    private boolean fieldCentricMode = false;
    private boolean backButtonPrevious = false;

    // Hardware Declarations
    private DcMotorEx leftFrontDrive   = null;
    private DcMotorEx rightFrontDrive  = null;
    private DcMotorEx leftBackDrive    = null;
    private DcMotorEx rightBackDrive   = null;
    private DcMotorEx shooterMotorFront = null;
    private DcMotorEx shooterMotorBack  = null;
    private Servo     feederServo      = null;
    private Servo     gateServo        = null;
    private IMU       imu              = null;

    @Override
    public void runOpMode() {

        // Hardware Mapping
        leftFrontDrive   = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive  = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive    = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive   = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        try {
            shooterMotorFront = hardwareMap.get(DcMotorEx.class, "shooter_motor_front");
            shooterMotorBack  = hardwareMap.get(DcMotorEx.class, "shooter_motor_back");
            feederServo       = hardwareMap.get(Servo.class, "feeder_servo");
            gateServo         = hardwareMap.get(Servo.class, "gate_servo");
        } catch (Exception e) {
            telemetry.addData("Notice", "Optional mechanisms missing.");
        }

        // IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Drive Motor Directions
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Dual Flywheels: Reverse back motor relative to front motor so both push ball out together
        if (shooterMotorFront != null) shooterMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (shooterMotorBack != null)  shooterMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (shooterMotorFront != null) shooterMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (shooterMotorBack != null)  shooterMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initial Servo States
        if (gateServo != null)   gateServo.setPosition(GATE_CLOSED_POS);
        if (feederServo != null) feederServo.setPosition(FEEDER_REST_POS);

        telemetry.addData("Status", "TeleOp Ready for Dual Flywheel System.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // GAMEPAD 1: Driver Mobility Controls
            // -----------------------------------------------------------------

            if (gamepad1.options || gamepad1.back) {
                imu.resetYaw();
            }

            if (gamepad1.y && !backButtonPrevious) {
                fieldCentricMode = !fieldCentricMode;
            }
            backButtonPrevious = gamepad1.y;

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double speedScale = gamepad1.left_bumper ? 0.45 : 1.0;

            double rotX = x;
            double rotY = y;

            if (fieldCentricMode) {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower  = ((rotY + rotX + rx) / denominator) * speedScale;
            double backLeftPower   = ((rotY - rotX + rx) / denominator) * speedScale;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * speedScale;
            double backRightPower  = ((rotY + rotX - rx) / denominator) * speedScale;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // -----------------------------------------------------------------
            // GAMEPAD 2: Mechanism & Dual Flywheel Controls
            // -----------------------------------------------------------------

            // Spin both shooter motors together (Gamepad 2 - A Button or Right Trigger)
            if (shooterMotorFront != null && shooterMotorBack != null) {
                if (gamepad2.a || gamepad2.right_trigger > 0.1) {
                    shooterMotorFront.setVelocity(SHOOTER_TARGET_VELOCITY);
                    shooterMotorBack.setVelocity(SHOOTER_TARGET_VELOCITY);
                } else {
                    shooterMotorFront.setVelocity(0.0);
                    shooterMotorBack.setVelocity(0.0);
                }
            }

            // Gate Servo (Gamepad 2 - Right Bumper opens gate)
            if (gateServo != null) {
                if (gamepad2.right_bumper) {
                    gateServo.setPosition(GATE_OPEN_POS);
                } else {
                    gateServo.setPosition(GATE_CLOSED_POS);
                }
            }

            // Feeder Servo (Gamepad 2 - X Button or Left Bumper pushes ball)
            if (feederServo != null) {
                if (gamepad2.x || gamepad2.left_bumper) {
                    feederServo.setPosition(FEEDER_PUSH_POS);
                } else {
                    feederServo.setPosition(FEEDER_REST_POS);
                }
            }

            // Telemetry Diagnostics
            telemetry.addData("Drive Mode", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            if (shooterMotorFront != null && shooterMotorBack != null) {
                telemetry.addData("Front Velocity", "Target: %.0f | Current: %.0f", SHOOTER_TARGET_VELOCITY, shooterMotorFront.getVelocity());
                telemetry.addData("Back Velocity",  "Target: %.0f | Current: %.0f", SHOOTER_TARGET_VELOCITY, shooterMotorBack.getVelocity());
            }
            telemetry.update();
        }
    }
}