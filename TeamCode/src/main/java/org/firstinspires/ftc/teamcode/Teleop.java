package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1,m2,m3,m4,arm, rotator;

    Servo clawRotator, clawClaw;

    public void runOpMode() {
        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left_motor");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right_motor");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left_motor");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right_motor");
        rotator = (DcMotorEx) hardwareMap.dcMotor.get("rotator");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("parRight");

        //servos
        //clawRotator = hardwareMap.servo.get("clawRotator");
        clawClaw = hardwareMap.servo.get("claw");
        clawRotator = hardwareMap.servo.get("clawRotator");

        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawClaw.setPosition(.75); //sets the position of the claw to closed before teleop starts
        clawRotator.setPosition(1); //sets the position of the claw hinge before teleop starts

        //All the code above this will begin when you press INIT on the Driver Hub
        //This waits until you press the play button
        waitForStart();

        //The code inside this loop will run once you press the play button
        //For TeleOp, we want the code to run continuously until you press stop
        //For auto, you would want the code to run once IF the play button is pressed
        while(opModeIsActive()) {
            //MecanumDrive: Left stick moves up/down/strafe, right stick turns
//Requires Mecanum wheels
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;
            //these booleans mean that it detects if one of these is pressed and returns true or false depending on if it is or isn't
            boolean arrrrrrmUp = gamepad2.right_bumper;
            boolean arrrrrrmDown = gamepad2.left_bumper;
            boolean rotUp = gamepad2.a;
            boolean rotDown = gamepad2.b;
            boolean clawOpen = gamepad2.y;
            boolean rotatorclaw = gamepad2.x;

            //math stuff prebuilt into mecanum
            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            //checks if each boolean returns true and if it does it does something
            if (rotatorclaw)
            {
                clawRotator.setPosition(.25); //servos use setPosition instead of setPower from a value 0-1
            }
            else
            {
                clawRotator.setPosition(1);
            }

            if (clawOpen)
            {
                clawClaw.setPosition(0);
            }
            else {
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
            }
            else if (rotDown)
            {
                rotator.setPower(-1);
            }
            else {
                rotator.setPower(0);
            }
        }
    }
}