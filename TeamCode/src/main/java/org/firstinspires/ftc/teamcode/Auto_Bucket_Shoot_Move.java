package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto_Bucket_Shoot_Move_IMU", group = "Autonomous")
public class Auto_Bucket_Shoot_Move extends LinearOpMode {

    // Kinematic Calculations: goBILDA 13.7:1 (383.6 CPR) on 96mm Wheels
    private static final double TICKS_PER_MM = 1.2719;

    // Field & Motion Constants
    private static final double DRIVE_FORWARD_MM = 609.6; // 1 FTC Tile = 24 inches
    private static final double DRIVE_POWER      = 0.50;  // 50% max drive speed
    private static final double HEADING_KP       = 0.035; // Gyro correction gain

    // Dual-Flywheel Closed-Loop Velocity (Ticks/Second)
    private static final double SHOOTER_TARGET_VELOCITY = 2200.0;

    // Servo Positions
    private static final double FEEDER_REST_POS = 0.0;
    private static final double FEEDER_PUSH_POS = 0.5;
    private static final double GATE_CLOSED_POS = 0.0;
    private static final double GATE_OPEN_POS   = 0.5;

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

        // 1. Hardware Mapping
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
            telemetry.addData("Notice", "Optional hardware mapping skipped.");
        }

        // 2. Initialize Control Hub IMU Gyroscope
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        // 3. Motor Directions
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Dual Flywheels: Reverse back motor relative to front motor so both propel ball in same direction
        if (shooterMotorFront != null) shooterMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (shooterMotorBack != null)  shooterMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // 4. Configure Encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (shooterMotorFront != null) shooterMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (shooterMotorBack != null)  shooterMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial servo positions
        if (gateServo != null)   gateServo.setPosition(GATE_CLOSED_POS);
        if (feederServo != null) feederServo.setPosition(FEEDER_REST_POS);

        telemetry.addData("Status", "Dual Flywheel Hardware & Closed-Loop Control Ready.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Step 1: Launch pre-loads using synchronized dual flywheels
            shootAllBallsVelocity();

            // Step 2: Drive 1 tile forward with IMU active heading lock
            driveStraightGyro(DRIVE_POWER, DRIVE_FORWARD_MM, 0.0);

            telemetry.addData("Status", "Autonomous Routine Complete");
            telemetry.update();
        }
    }

    /**
     * Powers both front and back shooter motors to identical velocities for launch stability.
     */
    private void shootAllBallsVelocity() {
        if (shooterMotorFront != null && shooterMotorBack != null) {
            // Spin up both flywheels simultaneously
            shooterMotorFront.setVelocity(SHOOTER_TARGET_VELOCITY);
            shooterMotorBack.setVelocity(SHOOTER_TARGET_VELOCITY);

            // Wait for both flywheel speeds to stabilize
            double startTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - startTime < 1.2)) {
                telemetry.addData("Front Shooter Vel", shooterMotorFront.getVelocity());
                telemetry.addData("Back Shooter Vel",  shooterMotorBack.getVelocity());
                telemetry.update();
            }

            // Open safety gate
            if (gateServo != null) {
                gateServo.setPosition(GATE_OPEN_POS);
                sleep(250);
            }

            // Feed 3 balls into the dual flywheel channel
            for (int i = 0; i < 3; i++) {
                if (feederServo != null) {
                    feederServo.setPosition(FEEDER_PUSH_POS);
                    sleep(300);
                    feederServo.setPosition(FEEDER_REST_POS);
                    sleep(300);
                }
            }

            // Close gate & power down both shooter motors
            if (gateServo != null) gateServo.setPosition(GATE_CLOSED_POS);
            shooterMotorFront.setVelocity(0.0);
            shooterMotorBack.setVelocity(0.0);
        } else {
            sleep(1500);
        }
    }

    private void driveStraightGyro(double maxSpeed, double distanceMM, double targetHeading) {
        int targetTicks = (int) (distanceMM * TICKS_PER_MM);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(targetTicks);
        rightFrontDrive.setTargetPosition(targetTicks);
        leftBackDrive.setTargetPosition(targetTicks);
        rightBackDrive.setTargetPosition(targetTicks);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                        leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError   = getAngleError(targetHeading, currentHeading);

            double correction = headingError * HEADING_KP;

            double leftPower  = Math.min(Math.max(maxSpeed - correction, -1.0), 1.0);
            double rightPower = Math.min(Math.max(maxSpeed + correction, -1.0), 1.0);

            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Ticks", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Heading Error", "%.2f deg", headingError);
            telemetry.update();
        }

        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getAngleError(double target, double current) {
        double error = target - current;
        while (error > 180)  error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void setMotorPower(double power) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
}