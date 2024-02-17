package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class secondauto {

//@Disabled
    public class FirstAuto extends LinearOpMode
    {
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
            telemetry.update();
            sleep(1000);
            telemetry.addData("Status","Arm Moving");
            telemetry.update();
            armmotor.setPower(-0.30);
            sleep(2000);
            armmotor.setPower(0);
            telemetry.addData("Status","Arm Stopped");
            telemetry.update();




        telemetry.addData("status", "Pixle recognized");

        sleep(10000);
                telemetry.addData("Status", "Drive Forward");
                telemetry.update();
                leftBackDrive.setPower(0.2);
                leftFrontDrive.setPower(0.2);
                rightFrontDrive.setPower(0.2);
                rightBackDrive.setPower(0.2);
        sleep(1500);
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                telemetry.addData("Status", "Drive complete");
                telemetry.update();


        sleep(1000);
                telemetry.addData("Status", "Claw Opening");
                telemetry.update();
                clawmotor.setPosition(0.45);
        sleep(1000);
                telemetry.addData("Status", "Claw Open pixle out");
                telemetry.update();


                telemetry.addData("Status", "Back UP");
                telemetry.update();
                leftBackDrive.setPower(-0.2);
                leftFrontDrive.setPower(-0.2);
                rightFrontDrive.setPower(-0.2);
                rightBackDrive.setPower(-0.2);
        sleep(1000);
                telemetry.addData("Status", "Drive complete");
                telemetry.update();
    }


}   // end method telemetryTfod()
}





