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


@Config
@Autonomous
public class CloseSpecimen extends LinearOpMode {
    public void runOpMode() {

        PeregrinesArm peregrinesArm = new PeregrinesArm(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(34.6, 63.3, Math.toRadians(-90)));

        Action entireAuto;

        entireAuto = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(6.1, 46.7), Math.toRadians(265)) //move to bar
                .waitSeconds(5.5)
                .strafeToLinearHeading(new Vector2d(51, 53), Math.toRadians(265))//move to right sample
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(63, 56), Math.toRadians(265)) // move to basket
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(64.2, 52), Math.toRadians(270)) //move out a few inches
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(63, 56), Math.toRadians(265)) //move back a few inches
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(39.7, 25.8), Math.toRadians(0))
                .waitSeconds(7)
                .strafeToLinearHeading(new Vector2d(63, 56), Math.toRadians(265)) //move back a few inches
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        peregrinesArm.clawRotatorToPosition(.4),
                        new SequentialAction(
                                new ParallelAction(
                                        peregrinesArm.rotatorToPower(2355, 0), //arm up
                                        peregrinesArm.clawRotatorToPosition(0.4)
                                ),
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.armToPower(1, 1.1, 0),
                                peregrinesArm.waitSeconds(.8),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.rotatorToPower(2000, 0),
                                peregrinesArm.waitSeconds(1.2),
                                peregrinesArm.armToPower(-1, 1.1, 0),
                                peregrinesArm.clawRotatorToPosition(.45),
                                peregrinesArm.waitSeconds(4),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.rotatorToPower(600, 0),
                                peregrinesArm.armToPower(1, .8, 0),
                                peregrinesArm.waitSeconds(.75),
                                peregrinesArm.clawToPosition(.75),
                                peregrinesArm.waitSeconds(1),
                                new ParallelAction(
                                        peregrinesArm.rotatorToPower(4664, 0),
                                        peregrinesArm.clawRotatorToPosition(.45)
                                ),
                                peregrinesArm.waitSeconds(2.5),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.rotatorToPower(600, 0),
                                peregrinesArm.waitSeconds(3),
                                peregrinesArm.clawToPosition(.75),
                                peregrinesArm.waitSeconds(1),
                                new ParallelAction(
                                        peregrinesArm.rotatorToPower(4664, 0),
                                        peregrinesArm.clawRotatorToPosition(.45)
                                ),
                                peregrinesArm.waitSeconds(2.5),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.waitSeconds(3),
                                peregrinesArm.rotatorToPower(700, 0),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.clawRotatorToPosition(.55),
                                peregrinesArm.waitSeconds(1.5),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.clawToPosition(.75),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.armToPower(-1, .2, 0),
                                peregrinesArm.waitSeconds(.3),
                                peregrinesArm.armToPower(1, .25, 0),
                                peregrinesArm.waitSeconds(.2),
                                peregrinesArm.rotatorToPower(4664, 0),
                                peregrinesArm.waitSeconds(3),
                                peregrinesArm.clawRotatorToPosition(.45),
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.clawToPosition(.0),
                                peregrinesArm.waitSeconds(2)

                        ),
                        entireAuto
                )
        );
    }
}
