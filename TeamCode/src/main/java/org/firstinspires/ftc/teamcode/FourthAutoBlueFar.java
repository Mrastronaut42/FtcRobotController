package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="BLUE FAR", group="Autonomous")
//@Disabled
public class FourthAutoBlueFar extends LinearOpMode {

    private VisionPortal visionPortal;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armmotor = null;
    private DcMotor slidemotor = null;
    private Servo clawmotor = null;


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        armmotor = hardwareMap.get(DcMotor.class, "am");
        slidemotor = hardwareMap.get(DcMotor.class, "sm");
        clawmotor = hardwareMap.get(Servo.class, "csm");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armmotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Preload");
        telemetry.update();
        clawmotor.setPosition(0.22);
        telemetry.addData("Status", "Preload Complete");
        telemetry.update();



        waitForStart();



        telemetry.addData("Status", "Begin Autonomous");    //




        telemetry.addData("status", "Pixle recognized");
        rightBackDrive.setPower(.25);
        rightFrontDrive.setPower(-.25);
        leftBackDrive.setPower(-.25);
        leftFrontDrive.setPower(.25);
        sleep(200);
        armmotor.setPower(-0.35);
        sleep(300);
        armmotor.setPower(0);
        sleep(1);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        sleep(1000);
        rightFrontDrive.setPower(0.25);
        rightBackDrive.setPower(0.25);
        leftFrontDrive.setPower(0.25);
        leftBackDrive.setPower(0.25);
        sleep(6500);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(1);
        armmotor.setPower(0.35);
        sleep(500);
        armmotor.setPower(0);
        sleep(100);
        rightBackDrive.setPower(0.25);
        rightFrontDrive.setPower(-0.25);
        leftBackDrive.setPower(-0.25);
        leftFrontDrive.setPower(0.25);
        sleep(1200);
        rightBackDrive.setPower(0.25);
        rightFrontDrive.setPower(0.25);
        leftBackDrive.setPower(0.25);
        leftFrontDrive.setPower(0.25);
        sleep(1000);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(1);
        slidemotor.setPower(0.5);
        sleep(800);
        slidemotor.setPower(0);
        sleep(1);
        armmotor.setPower(-0.25);
        sleep(700);
        armmotor.setPower(0);
        sleep(700);
        clawmotor.setPosition(0.45);
        sleep(1);
        slidemotor.setPower(-0.33);
        sleep(500);
        slidemotor.setPower(0);
        sleep(1);
        armmotor.setPower(0.50);
        sleep(500);
        armmotor.setPower(0);
        sleep(1);





        sleep(1000);
        telemetry.addData("status","strafe");
        rightFrontDrive.setPower(0.25);
        leftBackDrive.setPower(0.25);
        rightBackDrive.setPower(-0.25);
        leftFrontDrive.setPower(-0.25);
        sleep(1800);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }


}   // end method telemetryTfod()
