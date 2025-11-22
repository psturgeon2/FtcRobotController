package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Intake {
    private final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    private final double FULL_SPEED = 1.0;
    private DcMotor Intake;

    public void init (@NonNull HardwareMap hwMap) {
        Intake = hwMap.get(DcMotor.class, "intake");

        // Set launcher motor to RUN_USING_ENCODER and BRAKE to slow down faster than coasting.
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopIntake();
    }

    public void stopIntake() {
        Intake.setPower(STOP_SPEED);
    }

    public void startIntake () {
        Intake.setPower(FULL_SPEED);
    }
}
/*
        intake.setDirection(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                300, 0, 0, 10));
*/