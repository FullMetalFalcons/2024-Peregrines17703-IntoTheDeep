package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.Main;

@TeleOp
public class BlueBotTeleOp_HRI extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB, Slide, SlideRotator;
    Servo MainClaw, MainWrist, WallClaw, WallWrist, WallArmL, WallArmR;

    // Multiplication factor for slow drive mode
    final double SLOW_MODE_FACTOR = 0.001;

    // Test mode variables
    boolean testModeActive = false;
    boolean testModeToggleRequested;
    boolean lastTestModeToggleRequested = false;

    int testModeIndexMin = 1;
    int testModeIndexMax = 8;
    int testModeIndex = testModeIndexMin;

    boolean testModeIncrement;
    boolean lastTestModeIncrement = false;
    boolean testModeDecrement;
    boolean lastTestModeDecrement = false;

    double testModeControlValue;
    double testModeControlStick;

    double testModeReportedPosition;
    String testModeMotorName;


    // Servo position constants
    final double MAIN_CLAW_OPEN = 0.5;
    final double MAIN_CLAW_CLOSED = 0.8;
    final double MAIN_WRIST_HAND_OFF_POSITION = 0.7;
    final double MAIN_WRIST_SCORE_POSITION = 0.5;
    final double MAIN_WRIST_FLOOR_POSITION = 0.1;

    final double WALL_CLAW_OPEN = 0.8;
    final double WALL_CLAW_CLOSED = 1.0;
    final double WALL_WRIST_HAND_OFF_POSITION = 0.6;
    final double WALL_WRIST_PICK_UP_POSITION = 0.9;

    //final double WALL_ARM_LEFT_DOWN = 0.7;
    //final double WALL_ARM_LEFT_UP = 0.5;
    //final double WALL_ARM_RIGHT_DOWN = 0.0;
    //final double WALL_ARM_RIGHT_UP = 0.0;


    // Other control variables
    boolean lastWristPressed = false;
    boolean wristAtFloor = false;

    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    public void runOpMode() {

        // Define drive motors
        //The string should be the name on the Driver Hub
        // Set the strings at the top of the MecanumDrive file; they are shared between TeleOp and Autonomous
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);

        // Define other motors and servos
        Slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        SlideRotator = (DcMotorEx) hardwareMap.dcMotor.get("slideRotator");

        MainClaw = (Servo) hardwareMap.servo.get("mainClaw");
        MainWrist = (Servo) hardwareMap.servo.get("mainWrist");
        WallClaw = (Servo) hardwareMap.servo.get("wallClaw");
        WallWrist = (Servo) hardwareMap.servo.get("wallWrist");
        WallArmL = (Servo) hardwareMap.servo.get("wallArmLeft");
        WallArmR = (Servo) hardwareMap.servo.get("wallArmRight");

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

        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The program will pause here until the Play icon is pressed on the Driver Station
        waitForStart();

        // opModeIsActive() returns "true" as long as the Stop button has not been pressed on the Driver Station
        while(opModeIsActive()) {

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

            if (gamepad1.right_bumper) {
                powerLF *= SLOW_MODE_FACTOR;
                powerLB *= SLOW_MODE_FACTOR;
                powerRF *= SLOW_MODE_FACTOR;
                powerRB *= SLOW_MODE_FACTOR;
            }

            motorLF.setPower(powerLF);
            motorLB.setPower(powerLB);
            motorRF.setPower(powerRF);
            motorRB.setPower(powerRB);


            // Test Mode code
            testModeToggleRequested = (gamepad2.start && gamepad2.back);
            if (testModeToggleRequested && !lastTestModeToggleRequested) {
                // Switch between test mode and regular mode using "Start + Back"
                testModeActive = !testModeActive;
            }
            lastTestModeToggleRequested = testModeToggleRequested;

            if (testModeActive) {
                // Joystick down becomes 0.0, up becomes 1.0
                testModeControlStick = -gamepad2.right_stick_y;
                testModeControlValue = (1 + testModeControlStick) /2;

                switch (testModeIndex) {
                    case 1:
                        testModeMotorName = "Main Claw";
                        MainClaw.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 2:
                        testModeMotorName = "Main Wrist";
                        MainWrist.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 3:
                        testModeMotorName = "Wall Claw";
                        WallClaw.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 4:
                        testModeMotorName = "Wall Wrist";
                        WallWrist.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 5:
                        testModeMotorName = "Left Wall Arm";
                        WallArmL.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 6:
                        testModeMotorName = "Right Wall Arm";
                        WallArmR.setPosition(testModeControlValue);
                        testModeReportedPosition = testModeControlValue;
                        break;
                    case 7:
                        testModeMotorName = "Viper Slide";
                        Slide.setPower(testModeControlStick);
                        testModeReportedPosition = Slide.getCurrentPosition();
                        break;
                    case 8:
                        testModeMotorName = "Slide Rotator";
                        SlideRotator.setPower(testModeControlStick);
                        testModeReportedPosition = SlideRotator.getCurrentPosition();
                        break;
                }

                // Change controlled motor
                testModeIncrement = gamepad2.dpad_up;
                testModeDecrement = gamepad2.dpad_down;

                if (testModeIncrement && !lastTestModeIncrement) {
                    testModeIndex += 1;
                    // (condition ? true value : false value)
                    testModeIndex = (testModeIndex > testModeIndexMax ? testModeIndexMin : testModeIndex);
                }
                if (testModeDecrement && !lastTestModeDecrement) {
                    testModeIndex -= 1;
                    // (condition ? true value : false value)
                    testModeIndex = (testModeIndex < testModeIndexMin ? testModeIndexMax : testModeIndex);
                }
                lastTestModeIncrement = testModeIncrement;
                lastTestModeDecrement = testModeDecrement;


                // Write the name of the controlled motor/servo and its position to telemetry
                telemetry.addData(testModeMotorName + " position", testModeReportedPosition);

            } else {

                // Manually control the viper slide
                SlideRotator.setPower(-gamepad2.left_stick_y);
                Slide.setPower(-gamepad2.right_stick_y);

                // Control the main claw
                if (gamepad2.right_bumper) {
                    MainClaw.setPosition(MAIN_CLAW_OPEN);
                } else {
                    MainClaw.setPosition(MAIN_CLAW_CLOSED);
                }

                // Control the main wrist
                if (gamepad2.right_trigger > 0 && !lastWristPressed) {
                    // Toggle the position from "floor" to "score"
                    wristAtFloor = !wristAtFloor;
                    if (wristAtFloor) {
                        MainWrist.setPosition(MAIN_WRIST_SCORE_POSITION);
                    } else {
                        MainWrist.setPosition(MAIN_WRIST_FLOOR_POSITION);
                    }
                }
                lastWristPressed = (gamepad2.right_trigger > 0);

                // Initiate the wall pick-up sequence
                if (gamepad2.y) {
                    wallGrabHandOffRoutine();
                }





            }



            // If you want to print information to the Driver Station, use telemetry
            // addData() lets you give a string which is automatically followed by a ":" when printed
            //     the variable that you list after the comma will be displayed next to the label
            // update() only needs to be run once and will "push" all of the added data

            telemetry.update();


        } // opModeActive loop ends
    }

    // Functions are defined here
    public void wallGrabHandOffRoutine() {
        // Prepare the wall attachment for picking up a specimen
        //WallArmL.setPosition(WALL_ARM_LEFT_DOWN);
        //WallArmR.setPosition(WALL_ARM_RIGHT_DOWN);
        WallWrist.setPosition(WALL_WRIST_PICK_UP_POSITION);
        WallClaw.setPosition(WALL_CLAW_OPEN);
        Slide.setPower(0);
        SlideRotator.setPower(0);

        // Slowly drive backwards into the wall until the button is released
        while (gamepad2.y) {
            powerAllDriveMotors(-0.05);
        }

        // Stop driving and grab the specimen
        powerAllDriveMotors(0.0);
        WallClaw.setPosition(WALL_CLAW_CLOSED);
        sleep(1000);

        // Move everything into hand-off positions
        //WallArmL.setPosition(WALL_ARM_LEFT_UP);
        //WallArmR.setPosition(WALL_ARM_RIGHT_UP);
        powerAllDriveMotors(0.08);
        WallWrist.setPosition(WALL_WRIST_HAND_OFF_POSITION);
        MainClaw.setPosition(MAIN_CLAW_OPEN);
        MainWrist.setPosition(MAIN_WRIST_HAND_OFF_POSITION);
        Slide.setTargetPosition(0);
        SlideRotator.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Pause to drive away from the wall
        sleep(500);
        powerAllDriveMotors(0.0);

        // Wait for the motors to finish moving
        while (Slide.isBusy() || SlideRotator.isBusy()) {
            Slide.setPower(1);
            SlideRotator.setPower(1);
        }
        Slide.setPower(0);
        SlideRotator.setPower(0);
        sleep(1000);

        // Perform the hand-off
        MainClaw.setPosition(MAIN_CLAW_CLOSED);
        sleep(1000);
        WallClaw.setPosition(WALL_CLAW_OPEN);
        sleep(500);
        MainWrist.setPosition(MAIN_WRIST_SCORE_POSITION);

        // Set slide motors back to manual mode
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void powerAllDriveMotors(double desiredPower) {
        motorLF.setPower(desiredPower);
        motorLB.setPower(desiredPower);
        motorRF.setPower(desiredPower);
        motorRB.setPower(desiredPower);
    }


} // end class