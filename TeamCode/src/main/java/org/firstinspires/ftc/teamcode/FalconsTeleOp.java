package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FalconsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, arm, rotator, climb;
    Servo clawRotator, clawClaw;
    // TODO: Uncomment the following line if you are using servos
    //Servo Claw;

    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        // Set the strings at the top of the MecanumDrive file; they are shared between TeleOp and Autonomous
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);

        rotator = (DcMotorEx) hardwareMap.dcMotor.get("rotator");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("parRight");
        clawClaw = hardwareMap.servo.get("claw");
        clawRotator = hardwareMap.servo.get("clawRotator");
        climb = (DcMotorEx) hardwareMap.dcMotor.get("perp");

        // Use the following line as a template for defining new servos
        //Claw = (Servo) hardwareMap.servo.get("claw");

        //Set them to the correct modes
        //This reverses the motor direction
        // This data is also set at the top of MecanumDrive, for the same reasons as above
        motorLF.setDirection(DRIVE_PARAMS.leftFrontDriveDirection);
        motorLB.setDirection(DRIVE_PARAMS.leftBackDriveDirection);
        motorRF.setDirection(DRIVE_PARAMS.rightFrontDriveDirection);
        motorRB.setDirection(DRIVE_PARAMS.rightBackDriveDirection);

        //This resets the encoder values when the code is initialized
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        // GET OUT OF MY HEAD
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The program will pause here until the Play icon is pressed on the Driver Station
        waitForStart();

        // opModeIsActive() returns "true" as long as the Stop button has not been pressed on the Driver Station
        while(opModeIsActive()) {
            telemetry.addData("Rotator", rotator.getCurrentPosition());

            // Mecanum drive code
            double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
            double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
            double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

            // Set the desired powers based on joystick inputs (-1 to 1)
            powerX = gamepad1.left_stick_x;
            powerY = -gamepad1.left_stick_y;
            powerAng = -gamepad1.right_stick_x;

            // Perform vector math to determine the desired powers for each wheel
            double powerLF = powerX + powerY - powerAng;
            double powerLB = -powerX + powerY - powerAng;
            double powerRF = -powerX + powerY + powerAng;
            double powerRB = powerX + powerY + powerAng;
            boolean slowMode = gamepad1.right_bumper;

            //these booleans mean that it detects if one of these is pressed and returns true or false depending on if it is or isn't
            boolean arrrrrrmUp = gamepad2.right_bumper;
            boolean arrrrrrmDown = gamepad2.left_bumper;
            boolean rotUp = gamepad2.a;
            boolean rotDown = gamepad2.b;
            boolean clawOpen = gamepad2.y;
            boolean clawClose = gamepad2.x;
            boolean isOpen = false; // boolean used to check if the claw is open
            boolean rotatorClawDown = gamepad2.dpad_down;
            boolean rotatorClawUp = gamepad2.dpad_up;
            boolean climbOut = gamepad2.dpad_right;
            boolean climbIn = gamepad2.dpad_left;

            // Determine the greatest wheel power and set it to max
            double max = Math.max(1.0, Math.abs(powerLF));
            max = Math.max(max, Math.abs(powerRF));
            max = Math.max(max, Math.abs(powerLB));
            max = Math.max(max, Math.abs(powerRB));

            // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
            powerLF /= max;
            powerLB /= max;
            powerRF /= max;
            powerRB /= max;

            double m1 = powerLB / 4;
            double m2 = powerLB / 4;
            double m3 = powerRF / 4;
            double m4 = powerRB / 4;

            if (!slowMode) {
                motorLF.setPower(powerLF);
                motorLB.setPower(powerLB);
                motorRF.setPower(powerRF);
                motorRB.setPower(powerRB);
            } else {
                motorLF.setPower(m1);
                motorLB.setPower(m2);
                motorRF.setPower(m3);
                motorRB.setPower(m4);
            }

            if (climbOut)
            {
                climb.setPower(1);
            }
            else if (climbIn)
            {
                climb.setPower(-1);
            }
            else
            {
                climb.setPower(0);
            }


            // If you want to print information to the Driver Station, use telemetry
            // addData() lets you give a string which is automatically followed by a ":" when printed
            //     the variable that you list after the comma will be displayed next to the label
            // update() only needs to be run once and will "push" all of the added data

            //telemetry.addData("Label", "Information");
            //telemetry.update()

            //checks if each boolean returns true and if it does it does something
            if (rotatorClawDown)
            {
                clawRotator.setPosition(.2);
            }

            if (rotatorClawUp)
            {
                clawRotator.setPosition(.9);
            }

            if (clawOpen)
            {
                clawClaw.setPosition(0);
            }

            if (clawClose)
            {
                clawClaw.setPosition(.75);
            }

            if (arrrrrrmUp)
            {
                arm.setPower(1); //Motors use setPower from a value 0-1
            }
            else if (arrrrrrmDown)
            {
                arm.setPower(-1);
            }
            else {
                arm.setPower(0);
            }

            if (rotUp)
            {
                rotator.setPower(1);
                //rotator.setTargetPosition(4644);
            }
            else if (rotDown)
            {
                rotator.setPower(-1);
            }
            else {
                rotator.setPower(0);
            }

        } // opModeActive loop ends
    }
} // end class