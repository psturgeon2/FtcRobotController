package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagsWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LEDIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous
public class SquareAuto extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    private Follower follower;


    private final Pose pose1 = new Pose(0, 0, Math.toRadians(0));
    private final Pose pose2 = new Pose(0, 36, Math.toRadians(0));
    private final Pose pose3 = new Pose(36, 36, Math.toRadians(0));
    private final Pose pose4 = new Pose(36, 0, Math.toRadians(0));

    private PathChain path1, path2, path3, path4;


    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose1))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose1.getHeading())
                .build();
    }

    public static enum State {
        PATH1,
        PATH2,
        PATH3,
        PATH4,
        FINISHED
    }

    SquareAuto.State state;


    @Override
    public void init() {
        drive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);
        follower.setMaxPower(1);

        state = State.PATH1;
    }

    @Override
    public void loop() {

        follower.update();

        telemetry.addData("Current state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        switch (state) {
            case PATH1:
                if (!follower.isBusy()) {
                    follower.followPath(path1);
                    state = State.PATH2;
                    break;
                }


            case PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    state = State.PATH3;
                    break;
                }

            case PATH3:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    state = State.PATH4;
                    break;
                }

            case PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(path4);
                    state = State.FINISHED;
                    break;
                }

            case FINISHED:
                break;

            default:
                break;

        }
    }
}