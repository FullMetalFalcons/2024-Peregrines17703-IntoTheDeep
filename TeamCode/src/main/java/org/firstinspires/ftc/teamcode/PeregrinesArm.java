package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.NumberFormat;
import java.util.concurrent.TimeUnit;

public class PeregrinesArm {
    // Encoder storage variables for arm limit

    private DcMotorEx Arm;
    private DcMotorEx Slide;
    private Servo Claw;
    public PeregrinesArm(HardwareMap hardwareMap, Telemetry telemetry1) {
        // Set up motors using MecanumDrive constants
        Arm = (DcMotorEx) hardwareMap.dcMotor.get("parRight");

        Slide = (DcMotorEx) hardwareMap.dcMotor.get("rotator");

        Claw = hardwareMap.servo.get("claw");

        // The arm will hold its position when given 0.0 power
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setClawPosition(double desiredPosition) {
        Claw.setPosition(desiredPosition);
    }


    public class ClawToPosition implements Action {
        // Use constructor parameter to set target position
        private double targetClawPosition;
        public ClawToPosition(double clawPos) {
            super();
            targetClawPosition = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(targetClawPosition);
            return false;
        }
    }
    public ClawToPosition clawToPosition(double clawPos) {
        return new ClawToPosition(clawPos);
    }


    public class ArmToPower implements Action {
        // Use constructor parameter to set target position
        private double armPower;
        private long armTime;
        private boolean armInitizalized = false;
        private long startInNS;
        public ArmToPower(double armPower1, long ArmTime) {
            super();
            armPower = armPower1;
            armTime = ArmTime;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!armInitizalized){
                startInNS = System.nanoTime();
                armInitizalized = true;
            }

            if (System.nanoTime() > startInNS + TimeUnit.SECONDS.toNanos(armTime))
            {
                Arm.setPower(0);
                return false;
            }
            else {
                Arm.setPower(armPower);
                return true;
            }
        }
    }

    public class RotatorToPower implements Action {
        private double rotatorPower;
        private double rotatorTime;
        private boolean isInitialized = false;
        private long startTimeNS;
        public RotatorToPower(double rotPower, double rotTime) {
            super();
            rotatorPower = rotPower;
            rotatorTime = rotTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!isInitialized){
                startTimeNS = System.nanoTime();
                isInitialized = true;
            }

            if (System.nanoTime() > startTimeNS + TimeUnit.MILLISECONDS.toNanos((long)(rotatorTime*1000))) {
                Slide.setPower(0);
                return false;
            } else {
                Slide.setPower(rotatorPower);
                return true;
            }
        }
    }
    public ArmToPower armToPower(double armPower, long armTime) {
        return new ArmToPower(armPower, armTime);
    }

    public RotatorToPower rotatorToPower(double rotatorPower, double rotatorTime) {
        return new RotatorToPower(rotatorPower, rotatorTime);
    }
}