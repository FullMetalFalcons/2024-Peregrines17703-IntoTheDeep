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
public class FarAuto extends LinearOpMode {
    public void runOpMode() {

        PeregrinesArm peregrinesArm = new PeregrinesArm(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(34.6, 63.3, Math.toRadians(-90)));

        Action entireAuto;

        entireAuto = drive.actionBuilder(drive.pose)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                peregrinesArm.waitSeconds(2),
                                new ParallelAction(
                                        peregrinesArm.clawRotatorToPosition(.2),
                                        peregrinesArm.rotatorToPower(2600, 0)
                                ),

                                peregrinesArm.waitSeconds(3)
                        )
                       // entireAuto
                )
        );
    }
}
