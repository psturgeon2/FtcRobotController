package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDIndicator {
    private Servo LEDindicator;

    private final double LEDStartValue = 0.0; // nominally 0 degrees, may need to be tuned based on mounting angle of servo
    private double currentLEDValue;

    public void init (HardwareMap hwMap) {
        LEDindicator = hwMap.get(Servo.class,"blinkin");
        currentLEDValue = LEDStartValue;
    }

    public void incrementLEDValue() {
        currentLEDValue = currentLEDValue + .05;
        LEDindicator.setPosition(currentLEDValue);
    }

    public void decrementLEDValue() {
        currentLEDValue = currentLEDValue - .05;
        LEDindicator.setPosition(currentLEDValue);
    }

    public void setLEDRed(){
        currentLEDValue = .30;
        LEDindicator.setPosition(currentLEDValue);
    }

    public void setLEDBlue(){
        currentLEDValue = .60;
        LEDindicator.setPosition(currentLEDValue);
    }

    public void setLEDGreen(){
        currentLEDValue = .50;
        LEDindicator.setPosition(currentLEDValue);
    }

    public double getLEDValue(){
        return currentLEDValue;
    }

    /* SERVO POSITIONS TO COLORS

        .50 = GREEN
        .30 = RED (maybe tune a little redder if we want)
        .60 = BLUE

         */


}
