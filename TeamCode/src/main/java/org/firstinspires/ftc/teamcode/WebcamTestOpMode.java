package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagsWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp
public class WebcamTestOpMode  extends OpMode {
    AprilTagsWebcam aprilTagWebcam = new AprilTagsWebcam();
    Launcher launcher = new Launcher();


    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        launcher.init(hardwareMap);
    }

    @Override
    public void loop() {
        //Update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);

        if (gamepad1.yWasPressed()) {
            launcher.incrementLaunchSpeed();
        }
        else if (gamepad1.bWasPressed()) {
            launcher.decrementLaunchSpeed();
        }
        telemetry.addLine("Target Velocity: " + launcher.LaunchSpeed);
        telemetry.addLine("Lower Velocity: " + launcher.getLowerVelocity());
        telemetry.addLine("Upper Velocity: " + launcher.getUpperVelocity());


        launcher.setMotorVelocity();

    }
}
