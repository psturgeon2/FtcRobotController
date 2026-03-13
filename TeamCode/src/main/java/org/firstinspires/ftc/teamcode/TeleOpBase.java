package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagsWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.LEDIndicator;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class TeleOpBase extends OpMode {
    AprilTagsWebcam aprilTagWebcam = new AprilTagsWebcam();
    Launcher launcher = new Launcher();
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    TurretServo turret = new TurretServo();
    LEDIndicator led = new LEDIndicator();
    private Follower follower;
    int numMissingTagReads = 0;
    Boolean runningAutoPath = false;
    protected abstract int getTagid();

    private final Pose pose2 = new Pose(0, -36, Math.toRadians(90));



    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        launcher.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        led.init(hardwareMap);
        // turret.init(hardwareMap);

        Object EndPoseValue = blackboard.get("EndPose");
        telemetry.addData("EndPose Loaded", EndPoseValue);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose((Pose)EndPoseValue);
        follower.setMaxPower(1);
    }

    @Override
    public void loop() {

        follower.update();

        double distanceToGoalCM = -1;
        double angleToTag = 0;
        if (!gamepad2.b && !gamepad2.right_bumper && !gamepad2.left_bumper) {
            //Update the vision portal
            aprilTagWebcam.update();
            AprilTagDetection tagId = aprilTagWebcam.getTagBySpecificId(getTagid()); // TAG ID 24 is the red goal
            aprilTagWebcam.displayDetectionTelemetry(tagId);
            // NOTE: we will need a separate OPMODE (otherwise identical) that sets the target TAGID to BLUE (#20)
            if (tagId != null && tagId.ftcPose != null) {
                numMissingTagReads = 0;
                angleToTag = tagId.ftcPose.bearing + 2;
                turret.changeTurretByDegrees(angleToTag);

                distanceToGoalCM = tagId.ftcPose.range - 20;
                launcher.setMotorVelocityForDistance(distanceToGoalCM);
                // NOTE: use this after distance vs speed has been measured and calibrated
            } else if (numMissingTagReads < 100) {
                numMissingTagReads++;
            } else {
                // if we can't see the target
                // default back to neutral/default
                // and turn launch motors off
                launcher.stopLauncher();
                turret.resetTurret();
            }

            if (numMissingTagReads >= 100) {
                led.setLEDRed();
            } else if (tagId != null && tagId.ftcPose != null) {
                double speedError = launcher.getLaunchSpeedError();
                double angleError = turret.getAngleError();
                if (speedError < 50 && angleError < 2) {
                    led.setLEDGreen();
                } else {
                    led.setLEDBlue();
                }
            }
            // set LED to yellow? Or something else to indicate we don't have 100 missed reads, but aren't facing the tag now
            // if we turn quick enough, no guarantee that we will get an angle error...
            // maybe just > 10 missedTagReads? that would indicate that the tag reads are sketchy even if facing it
            else if (numMissingTagReads > 10) { // || angleError > 5
                led.setLEDRed();
            }
        } else if (gamepad2.right_bumper) {
            launcher.presetMotorVelocity(1000);
            telemetry.addLine("Preset 1000");
        } else if (gamepad2.left_bumper) {
            launcher.presetMotorVelocity(1400);
            telemetry.addLine("preset 1400");
        } else if (gamepad2.b) {
            launcher.stopLauncher();
            telemetry.addLine("skip april tag");
            //This skips the april tag reading and math
        }


// Added a way for Game Controller 1 to do everything for testing
        if (gamepad2.right_trigger != 0 || gamepad1.a) {
            launcher.loadBall();
        } else if (gamepad2.x || gamepad1.x) {
            launcher.unloadBall();
        } else {
            launcher.resetFeeder();
        }

        //For Intake (test if same buttons works)
        if (gamepad1.right_trigger != 0 || gamepad2.right_trigger != 0) {
            intake.startIntake();
        } else if (gamepad1.left_trigger != 0) {
            intake.reverseIntake();
        } else {
            intake.stopIntake();
        }

        if(!runningAutoPath) {
            // slow mode
            if (gamepad2.y) {
                drive.drive(-gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x * 0.5);
            } else {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                // Note: pushing left stick forward gives negative value
                drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }

        if(gamepad1.yWasPressed()){
            runningAutoPath = true;
            //Heading is in radians
            Pose Current = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
            PathChain TestPath = follower.pathBuilder()
                    .addPath(new BezierLine(Current, pose2))
                    .setLinearHeadingInterpolation(Current.getHeading(), pose2.getHeading())
                    .build();
            follower.followPath(TestPath);
        }
        if((runningAutoPath && !follower.isBusy()) || gamepad1.leftBumperWasPressed()){
            follower.breakFollowing();
            runningAutoPath = false;
        }


        launcher.setMotorVelocity();

        String robotX = String.format("%.2f", follower.getPose().getX());
        String robotY = String.format("%.2f", follower.getPose().getY());
        String robotHeading = String.format("%.2f", follower.getPose().getHeading());
        telemetry.addLine("Robot X,Y: " + robotX + ", " + robotY);
        telemetry.addLine("Robot X,Y: " + robotHeading);
        telemetry.addLine("Distance/angle to goal: " + distanceToGoalCM + "/" + angleToTag);
        telemetry.addLine("Missed Tag Reads: " + numMissingTagReads);
        telemetry.addLine("Target Velocity: " + launcher.getTargetLaunchSpeed());
        telemetry.addLine("Right Velocity: " + launcher.getLowerVelocity());
        telemetry.addLine("Left Velocity: " + launcher.getUpperVelocity());
        telemetry.addData("State: ", launcher.getState());
        String turretPositionStr = String.format("%.2f",turret.getCurrentPosition());
        telemetry.addLine("Turret Position: " + turretPositionStr);


    }
}
