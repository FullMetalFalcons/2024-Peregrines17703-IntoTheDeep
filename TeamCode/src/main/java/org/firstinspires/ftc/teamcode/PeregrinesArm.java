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
    private Servo Claw, clawRotator;

    public PeregrinesArm(HardwareMap hardwareMap, Telemetry telemetry1) {
        // Set up motors using MecanumDrive constants
        Arm = (DcMotorEx) hardwareMap.dcMotor.get("parRight");
        Slide = (DcMotorEx) hardwareMap.dcMotor.get("rotator");

        Claw = hardwareMap.servo.get("claw");
        clawRotator = hardwareMap.servo.get("clawRotator");

        // The arm will hold its position when given 0.0 power
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //listen man, idk what half of this means but it works. and i dont have the time to do it right now but its basically just different Actions for Roadrunner 1x which is a lot harder for some reason
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

    public class ClawRotatorToPosition implements Action {
        // Use constructor parameter to set target position
        private double targetClawPosition;

        public ClawRotatorToPosition(double rotatorPos) {
            super();
            targetClawPosition = rotatorPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawRotator.setPosition(targetClawPosition);
            return false;
        }
    }

    public ClawRotatorToPosition clawRotatorToPosition(double rotatorPos) {
        return new ClawRotatorToPosition(rotatorPos);
    }


    public class ArmToPower implements Action {
        // Use constructor parameter to set target position
        private double armPower;
        private double armTime;
        private boolean armInitizalized = false;
        private long startInNS;

        public ArmToPower(double armPower1, double ArmTime) {
            super();
            armPower = armPower1;
            armTime = ArmTime;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!armInitizalized) {
                startInNS = System.nanoTime();
                armInitizalized = true;
            }

            if (System.nanoTime() > startInNS + TimeUnit.MILLISECONDS.toNanos((long) (armTime * 1000))) {
                Arm.setPower(0);
                return false;
            } else {
                Arm.setPower(armPower);
                return true;
            }
        }
    }

    public class RotatorToPower implements Action {
        private int rotatorPower;
        public RotatorToPower(int rotPower) {
            super();
            rotatorPower = rotPower;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Slide.setTargetPosition(rotatorPower);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
            return true;
        }
    }

    /*public class ClawOpen implements Action {

    }*/
    public ArmToPower armToPower(double armPower, double armTime) {
        return new ArmToPower(armPower, armTime);
    }

    public RotatorToPower rotatorToPower(int rotatorPower) {
        return new RotatorToPower(rotatorPower);
    }

    public class Wait implements Action {
        private double seconds;
        private boolean isInitialized = false;
        private long startTimeNs;

        public Wait(double time) {
            seconds = time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!isInitialized) {
                startTimeNs = System.nanoTime();
                isInitialized = true;
            }
            return System.nanoTime() - startTimeNs < TimeUnit.MILLISECONDS.toNanos((long) (seconds * 1000));
        }
    }

    // Add to PeregrinesArm:
    public Wait waitSeconds(double seconds) {
        return new Wait(seconds);
    }

}
