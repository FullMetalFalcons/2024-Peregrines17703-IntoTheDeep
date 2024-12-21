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
public class CloseBlue extends LinearOpMode {
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
                        drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(54.5, 53), Math.toRadians(225)).build(),
                        peregrinesArm.clawRotatorToPosition(.25), //sets the claw position to
                        peregrinesArm.clawToPosition(.75),
                        peregrinesArm.rotatorToPower(1, 1.9),
                        peregrinesArm.armToPower(1, 3),
                        peregrinesArm.waitSeconds(1),
                        peregrinesArm.clawRotatorToPosition(1),
                        peregrinesArm.waitSeconds(.75),
                        peregrinesArm.clawToPosition(0),
                        peregrinesArm.waitSeconds(1),
                        peregrinesArm.armToPower(-1, 3)
                )
        );
    }
}