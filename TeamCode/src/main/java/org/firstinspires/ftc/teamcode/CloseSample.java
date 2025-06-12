package org.firstinspires.ftc.teamcode;

// RoadRunner Specific Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
public class CloseSample extends LinearOpMode {
    public void runOpMode() {

        PeregrinesArm peregrinesArm = new PeregrinesArm(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(38.5, 63.3, Math.toRadians(-90)));

        Action entireAuto;

        entireAuto = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(63, 56), Math.toRadians(265))
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(52, 51), Math.toRadians(265))//move to right sample
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(63, 57), Math.toRadians(265)) // move to basket
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(63, 51), Math.toRadians(265)) //move out a few inches (middle sample)
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(63, 56), Math.toRadians(265)) //move back a few inches
                .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(63, 55), Math.toRadians(275))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(27, 10), Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        peregrinesArm.clawRotatorToPosition(.4),
                        new SequentialAction(
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.rotatorToPower(4664, 0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.armToPower(1, 3, 0),
                                peregrinesArm.waitSeconds(1.3),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(.6),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.rotatorToPower(600, 0),
                                peregrinesArm.waitSeconds(3),
                                peregrinesArm.clawRotatorToPosition(.2),
                                peregrinesArm.waitSeconds(.5),
                                peregrinesArm.clawToPosition(.75),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(.5),
                                peregrinesArm.rotatorToPower(4664, 0),
                                peregrinesArm.waitSeconds(2.5),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.rotatorToPower(600, 0),
                                peregrinesArm.waitSeconds(3),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.waitSeconds(.5),
                                peregrinesArm.clawToPosition(.75),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(.5),
                                peregrinesArm.rotatorToPower(4664, 0),
                                peregrinesArm.waitSeconds(2.5),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1)
                        ),
                        entireAuto
                )
        );
    }
}
