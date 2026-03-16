package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.doclint.Entity;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagsWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LEDIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp
public class RedTeleOp  extends TeleOpBase {

    private final Pose openGatePose = new Pose(128, 66, Math.toRadians(90)); // Open gate
    private final Pose parkPose = new Pose(38.5, 33.5, Math.toRadians(90)); // Open gate
    @Override
    protected int getTagid() {
        return 24;
    }

    @Override
    protected Pose getGatePose() {
        return openGatePose;
    }

    @Override
    protected Pose getParkPose() {
        return parkPose;
    }
}