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
public class CloseBlue extends LinearOpMode {
    public void runOpMode() {

        PeregrinesArm peregrinesArm = new PeregrinesArm(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(34.6, 63.3, Math.toRadians(-90)));

        Action entireAuto;

        entireAuto = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(64.5, 55.6), Math.toRadians(265))
                .waitSeconds(7)
                .turn(Math.toRadians(-30))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        peregrinesArm.clawRotatorToPosition(.4),
                        new SequentialAction(
                                peregrinesArm.rotatorToPower(4664, 0),
                                peregrinesArm.clawRotatorToPosition(.4),
                                peregrinesArm.waitSeconds(2),
                                peregrinesArm.armToPower(1, .8, 2),
                                peregrinesArm.clawRotatorToPosition(1),
                                peregrinesArm.waitSeconds(.3),
                                peregrinesArm.clawToPosition(0),
                                peregrinesArm.waitSeconds(1),
                                peregrinesArm.clawRotatorToPosition(.4)
                        ),
                        entireAuto
                )
        );
    }
}
