package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private boolean isRobotCentric = true;

    protected abstract int getTagid();
    protected abstract Pose getGatePose();
    protected abstract Pose getParkPose();

    protected abstract int getDriverDirection();



    private final Pose pose2 = new Pose(0, -36, Math.toRadians(90));


    private Boolean followerInitialized = false;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        launcher.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        led.init(hardwareMap);
        // turret.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        if(blackboard.containsKey("EndPose")) {
            Object EndPoseValue = blackboard.get("EndPose");
            telemetry.addData("EndPose Loaded", EndPoseValue);
            follower.setStartingPose((Pose) EndPoseValue);
            followerInitialized = true;
            isRobotCentric = false;
        }
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
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

        // launch feeder
        if (gamepad2.right_trigger != 0 || gamepad1.a) {
            launcher.loadBall();
        } else if (gamepad2.x) {
            launcher.unloadBall();
        } else {
            launcher.resetFeeder();
        }

        //For Intake
        if (gamepad1.right_trigger != 0 || gamepad2.right_trigger != 0) {
            intake.startIntake();
        } else if (gamepad1.left_trigger != 0) {
            intake.reverseIntake();
            launcher.unloadBall();
        } else {
            intake.stopIntake();
        }

        double speedMultiplier = 1;
        if(gamepad2.y){
            speedMultiplier = .5;
        }


        if(gamepad1.dpadDownWasPressed() && followerInitialized) {
            // followerInitialized must be true in order to run field centric
            isRobotCentric = false; //field centric
        }
        if(gamepad1.dpadUpWasPressed()) {
            // can always run robot centric
            isRobotCentric = true;
        }
        if(!runningAutoPath) {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                // Note: pushing left stick forward gives negative value
                //drive.drive(-gamepad1.left_stick_y * speedMultiplier, gamepad1.left_stick_x * speedMultiplier, gamepad1.right_stick_x * speedMultiplier);
            if(!isRobotCentric) {
                // leftY * getMultiplier()
                int redVsBlueDirection = getDriverDirection();
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speedMultiplier * redVsBlueDirection, -gamepad1.left_stick_x * speedMultiplier * redVsBlueDirection, -gamepad1.right_stick_x * speedMultiplier, false);
                telemetry.addLine("FC Right Stick X,Y" + gamepad1.left_stick_x + ", " + gamepad1.left_stick_y);
                telemetry.addLine("FC Left Stick X" + gamepad1.right_stick_x);
            } else {
                //drive.drive(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier);
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier, true);
                telemetry.addLine("RC Right Stick X,Y" + gamepad1.left_stick_x + ", " + gamepad1.left_stick_y);
                telemetry.addLine("RC Left Stick X" + gamepad1.right_stick_y);
            }
        }

        if(gamepad1.dpadLeftWasPressed()){
            runningAutoPath = true;
            //Heading is in radians
            Pose Current = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
            Pose gatePose = getGatePose();
            PathChain GatePath = follower.pathBuilder()
                    .addPath(new BezierLine(Current, gatePose))
                    .setLinearHeadingInterpolation(Current.getHeading(), gatePose.getHeading())
                    .build();
            follower.followPath(GatePath);
        } else if(gamepad1.dpadRightWasPressed()) {
            runningAutoPath = true;
            //Heading is in radians
            Pose Current = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
            Pose parkPose = getParkPose();
            PathChain ParkPath = follower.pathBuilder()
                    .addPath(new BezierLine(Current, parkPose))
                    .setLinearHeadingInterpolation(Current.getHeading(), parkPose.getHeading())
                    .build();
            follower.followPath(ParkPath);
        }
        if((runningAutoPath && !follower.isBusy()) || gamepad1.leftBumperWasPressed()){
            follower.breakFollowing();
            follower.startTeleOpDrive(true);
            runningAutoPath = false;
            telemetry.addLine("Path Broken");
        }



        launcher.setMotorVelocity();

        String robotX = String.format("%.2f", follower.getPose().getX());
        String robotY = String.format("%.2f", follower.getPose().getY());
        String robotHeading = String.format("%.2f", follower.getPose().getHeading());
        telemetry.addLine("Robot X,Y: " + robotX + ", " + robotY);
        telemetry.addLine("Robot X,Y: " + robotHeading);
        if(!isRobotCentric){
            telemetry.addLine("Field Centric");
        }
        else {
            telemetry.addLine("Robot Centric");
        }
        if (follower.isBusy()){
            telemetry.addLine("Follower is Busy");
        } else {
            telemetry.addLine("Follower is not Busy");
        }
        if (runningAutoPath){
            telemetry.addLine("running auto path");
        } else {
            telemetry.addLine("not running auto path");
        }
        if (gamepad1.left_bumper){
            telemetry.addLine("left bumper pressed");
        } else {
            telemetry.addLine("left bumper not pressed");
        }
        telemetry.addLine("Distance/angle to goal: " + distanceToGoalCM + "/" + angleToTag);
        telemetry.addLine("Missed Tag Reads: " + numMissingTagReads);
        telemetry.addLine("Target Velocity: " + launcher.getTargetLaunchSpeed());
        telemetry.addLine("Right Velocity: " + launcher.getLowerVelocity());
        telemetry.addLine("Left Velocity: " + launcher.getUpperVelocity());
        telemetry.addData("State: ", launcher.getState());
        String turretPositionStr = String.format("%.2f",turret.getCurrentPosition());
        telemetry.addLine("Turret Position: " + turretPositionStr);


    }

    @Override
    public void stop() {
        Pose endPose = follower.getPose();
        blackboard.put("EndPose", endPose);
        telemetry.addData("EndPose", endPose);
        super.stop();
    }
}
