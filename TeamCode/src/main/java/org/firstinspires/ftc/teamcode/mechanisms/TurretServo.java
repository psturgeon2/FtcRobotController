package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretServo {
    private Servo turretServo;

    private final double TURRET_START_POSITION = 0.0; // nominally 0 degrees, may need to be tuned based on mounting angle of servo

    private double currentTurretAngle;

    public void init (HardwareMap hwMap) {
        turretServo = hwMap.get(Servo.class,"turret_servo");
        currentTurretAngle = TURRET_START_POSITION; // default to "0"
        resetTurret(); // default it to "0" degrees
    }

    /// Set launch feeder server back to "0" position
    public void resetTurret() {
        // Set feeders to a preset value to stop the servos.
        turretServo.setPosition(TURRET_START_POSITION);
    }

    /// Set launch feeder server back to "0" position
    public void changeTurretByDegrees(double deltaDegrees) {

        // TODO: change currentTurretAngle (range: 0 - 1.0) based on input delta angle as measured by the camera (range: -180 - 180)
        // set the turret to the new angle
        turretServo.setPosition(currentTurretAngle);
    }
}
