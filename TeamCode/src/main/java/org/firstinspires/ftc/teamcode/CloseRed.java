package org.firstinspires.ftc.teamcode;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@Autonomous
public class CloseRed extends LinearOpMode {
    public void runOpMode() {

        DcMotorEx arm, rotator;
        PeregrinesArm peregrinesArm = new PeregrinesArm(hardwareMap, telemetry);

        rotator = (DcMotorEx) hardwareMap.dcMotor.get("rotator");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("parRight");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(34.6, 63.3, Math.toRadians(-90)));

        Action closeBlue;

        closeBlue = drive.actionBuilder(drive.pose)
                .waitSeconds(3)
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(55))
                .waitSeconds(2)
                .turn(Math.toRadians(-55))
                .waitSeconds(3)
                .turn(Math.toRadians(75))
                .waitSeconds(2)
                .turn(Math.toRadians(-75))
                .strafeToLinearHeading(new Vector2d(24, 0), Math.toRadians(180))
                .build();


        Action trajectoryActionChosen = closeBlue;

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(-54.5, -53), Math.toRadians(225)).build(), //moves to the position -54.5, - 53 and aligns it so the back is facing the baskets
                        peregrinesArm.clawRotatorToPosition(.25), //sets the claw position to
                        peregrinesArm.clawToPosition(.75), //closes claw
                        //peregrinesArm.rotatorToPower(1, 1.9), //moves arm up perpendicular to ground
                        peregrinesArm.armToPower(1, 3), //moves the actuator to the top position
                        peregrinesArm.waitSeconds(1), //waits 1 second
                        peregrinesArm.clawRotatorToPosition(1), //moves the claw hinge into the basket
                        peregrinesArm.waitSeconds(.75), //waits .75 seconds
                        peregrinesArm.clawToPosition(0), //opens the claw
                        peregrinesArm.waitSeconds(1), //waits 1 second
                        peregrinesArm.clawRotatorToPosition(.25), //moves claw out of basket
                        peregrinesArm.waitSeconds(1), //waits 1 second
                        //peregrinesArm.rotatorToPower(-1, 1.3), //moves the arm down but the actuator is still out
                        drive.actionBuilder(drive.pose).turn(Math.toRadians(29)).build(), //turns the robot 32 degrees
                        peregrinesArm.clawToPosition(0), //opens the claw
                        peregrinesArm.clawRotatorToPosition(0), //flops the claw rotator onto the ground while its open
                        peregrinesArm.waitSeconds(.1), //waits .1 seconds
                        peregrinesArm.clawToPosition(.75), //closes the claw
                        //peregrinesArm.rotatorToPower(1, 1.3), //moves the arm up perpendicular to the ground
                        drive.actionBuilder(drive.pose).turn(Math.toRadians(-29)).build(), //turns the robot back
                        peregrinesArm.waitSeconds(1), //waits 1 seconds
                        peregrinesArm.clawRotatorToPosition(1), //moves claw into basket
                        peregrinesArm.clawToPosition(0), //opens claw and hopefully deposits sample
                        peregrinesArm.clawRotatorToPosition(.25), //moves claw out of basket
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(-21, -8.7), Math.toRadians(0)).build() //moves into observation zone
                )
        );
    }
}