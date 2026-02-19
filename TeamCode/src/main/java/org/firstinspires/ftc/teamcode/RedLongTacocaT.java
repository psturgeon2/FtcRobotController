package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class RedLongTacocaT extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    Launcher launcher = new Launcher();
    AprilTagsWebcam aprilTagWebcam = new AprilTagsWebcam();
    TurretServo turret = new TurretServo();
    LEDIndicator led = new LEDIndicator();
    int numMissingTagReads = 0;
    private Follower follower;


    private final Pose startPose = new Pose(88.8, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose launchingPose = new Pose(83.5, 21.5, Math.toRadians(65)); // Where our robot launches from
    private final Pose pickup1_1Pose = new Pose(100, 8, Math.toRadians(0)); // Ready to pick up balls
    private final Pose pickup1_2Pose = new Pose(123, 8, Math.toRadians(0)); // Pickup balls away from wall
    private final Pose pickup1_3Pose = new Pose(133, 8, Math.toRadians(0)); // pickup balls close to wall
    private Path startToLaunching;
    private PathChain launchingToPickupReady1, pickup1ToPickup2, pickup2ToPickup3, pickup3ToPickup2, pickup3ToLaunching, launchingToPickup2;

    public void buildPaths() {

        //This sets up our first path where we back up
        startToLaunching = new Path(new BezierLine(startPose, launchingPose));
        startToLaunching.setLinearHeadingInterpolation(startPose.getHeading(), launchingPose.getHeading());

        //these next several section set up the rest of the paths, created in the PathChain
        launchingToPickupReady1 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, pickup1_1Pose))
                .setLinearHeadingInterpolation(launchingPose.getHeading(), pickup1_1Pose.getHeading())
                .build();

        pickup1ToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_1Pose, pickup1_2Pose))
                .setLinearHeadingInterpolation(pickup1_1Pose.getHeading(), pickup1_2Pose.getHeading())
                .build();

        pickup2ToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_2Pose, pickup1_3Pose))
                .setLinearHeadingInterpolation(pickup1_2Pose.getHeading(), pickup1_3Pose.getHeading())
                .build();

        pickup3ToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_3Pose, pickup1_2Pose))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), pickup1_2Pose.getHeading())
                .build();

        pickup3ToLaunching = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_3Pose, launchingPose))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), launchingPose.getHeading())
                .build();
        launchingToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(launchingPose, pickup1_2Pose))
                .setLinearHeadingInterpolation( launchingPose.getHeading(), pickup1_2Pose.getHeading())
                .build();
    }

    enum State {
        GO_TO_LAUNCH_1,
        WAIT_TO_FINISH_PATH_1,
        FIND_TAG_1,
        SPIN_UP_1,
        LAUNCHING_1,
        INTAKE_POSE_1_1,
        INTAKE_1_2,
        INTAKE_1_3,
        INTAKE_1_4,
        INTAKE_1_5,
        INTAKE_1_6,
        INTAKE_1_7,
        GO_TO_LAUNCH_2,
        WAIT_TO_FINISH_PATH_2,
        FIND_TAG_2,
        SPIN_UP_2,
        LAUNCHING_2,
        INTAKE_POSE_2_1,
        INTAKE_2_2,
        INTAKE_2_3,
        INTAKE_2_4,
        INTAKE_2_5,
        INTAKE_2_6,
        INTAKE_2_7,
        GO_TO_LAUNCH_3,
        WAIT_TO_FINISH_PATH_3,
        FIND_TAG_3,
        SPIN_UP_3,
        LAUNCHING_3,
        GO_TO_END_POSE,
        FINISHED
    }

    RedLongTacocaT.State state;
    ElapsedTime driveTimer = new ElapsedTime();


    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);
        led.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        state = State.GO_TO_LAUNCH_1;
    }

    public void loop() {

        follower.update();


        // Feedback to Driver Hub for debugging
        telemetry.addData("Current state", state);
        telemetry.addLine("Target Velocity: " + launcher.getTargetLaunchSpeed());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        // if (in spin up, launch, find tag, etc
        if(state == State.FIND_TAG_1 ||
                state == State.SPIN_UP_1 ||
                state == State.LAUNCHING_1 ||
                state == State.FIND_TAG_2 ||
                state == State.SPIN_UP_2 ||
                state == State.LAUNCHING_2 ||
                state == State.FIND_TAG_3 ||
                state == State.SPIN_UP_3 ||
                state == State.LAUNCHING_3 )
        {
            doAprilTag();
        }
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        switch (state) {
            case GO_TO_LAUNCH_1:
                follower.followPath(startToLaunching);
                state = State.WAIT_TO_FINISH_PATH_1;
                break;
            case WAIT_TO_FINISH_PATH_1:
                if(!follower.isBusy()){
                    state = State.FIND_TAG_1;
                }
                break;
            case FIND_TAG_1:
                if(id24 != null){
                    state = State.SPIN_UP_1;
                }
                break;
            case SPIN_UP_1:
                double speedError = launcher.getLaunchSpeedError();
                double angleError = turret.getAngleError();
                if (speedError < 50){
                    state = State.LAUNCHING_1;
                    driveTimer.reset();
                }
                break;
            case LAUNCHING_1:
                if (driveTimer.seconds() < 2) {
                    intake.startIntake();
                    launcher.loadBall();
                }
                else {
                    intake.stopIntake();
                    //launcher.setMotorVelocity();
                    launcher.stopLauncher();
                    launcher.resetFeeder();
                    //Launcher.LaunchState = Launcher.LaunchState.IDLE;
                    state = State.INTAKE_POSE_1_1;
                    driveTimer.reset();
                }
                break;
            case INTAKE_POSE_1_1:
                if(!follower.isBusy()){
                    intake.startIntake();
                    follower.followPath(launchingToPickupReady1, true);
                    state = State.INTAKE_1_2;
                }
                break;
            case INTAKE_1_2:
                if(!follower.isBusy()){
                    follower.followPath(pickup1ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_1_3;
                }
                break;
            case INTAKE_1_3:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_1_4;
                }
                break;
            case INTAKE_1_4:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_1_5;
                }
                break;
            case INTAKE_1_5:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.GO_TO_LAUNCH_2;
                }
                break;
            case INTAKE_1_6:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_1_7;
                }
                break;
            case INTAKE_1_7:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.GO_TO_LAUNCH_2;
                }
                break;
            case GO_TO_LAUNCH_2:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToLaunching);
                    state = State.WAIT_TO_FINISH_PATH_2;
                }
                break;
            case WAIT_TO_FINISH_PATH_2:
                if(!follower.isBusy()){
                    state = State.FIND_TAG_2;
                }
                break;
            case FIND_TAG_2:
                if(id24 != null){
                    intake.stopIntake();
                    state = State.SPIN_UP_2;
                }
                break;
            case SPIN_UP_2:
                speedError = launcher.getLaunchSpeedError();
                angleError = turret.getAngleError();
                if (speedError < 100){
                    state = State.LAUNCHING_2;
                    driveTimer.reset();
                }
                break;
            case LAUNCHING_2:
                if (driveTimer.seconds() < 3) {
                    intake.startIntake();
                    launcher.loadBall();
                }
                else {
                    intake.stopIntake();
                    launcher.resetFeeder();
                    Launcher.LaunchState = Launcher.LaunchState.IDLE;
                    launcher.stopLauncher();
                    state = State.INTAKE_POSE_2_1;
                    driveTimer.reset();
                }
                break;
            case INTAKE_POSE_2_1:
                if(!follower.isBusy()){
                    intake.startIntake();
                    follower.followPath(launchingToPickupReady1, true);
                    state = State.INTAKE_2_2;
                }
                break;
            case INTAKE_2_2:
                if(!follower.isBusy()){
                    follower.followPath(pickup1ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_2_3;
                }
                break;
            case INTAKE_2_3:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_2_4;
                }
                break;
            case INTAKE_2_4:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_2_5;
                }
                break;
            case INTAKE_2_5:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.GO_TO_LAUNCH_3;
                }
                break;
            case INTAKE_2_6:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToPickup2, .7, false);
                    intake.startIntake();
                    state = State.INTAKE_2_7;
                }
                break;
            case INTAKE_2_7:
                if(!follower.isBusy()){
                    follower.followPath(pickup2ToPickup3, .7, false);
                    intake.startIntake();
                    state = State.GO_TO_LAUNCH_3;
                }
                break;
            case GO_TO_LAUNCH_3:
                if(!follower.isBusy()){
                    follower.followPath(pickup3ToLaunching);
                    state = State.WAIT_TO_FINISH_PATH_3;
                }
                break;
            case WAIT_TO_FINISH_PATH_3:
                if(!follower.isBusy()){
                    state = State.FIND_TAG_3;
                }
                break;
            case FIND_TAG_3:
                if(id24 != null){
                    intake.stopIntake();
                    state = State.SPIN_UP_3;
                }
                break;
            case SPIN_UP_3:
                speedError = launcher.getLaunchSpeedError();
                angleError = turret.getAngleError();
                if (speedError < 100){
                    state = State.LAUNCHING_3;
                    driveTimer.reset();
                }
                break;
            case LAUNCHING_3:
                if (driveTimer.seconds() < 3) {
                    intake.startIntake();
                    launcher.loadBall();
                }
                else {
                    intake.stopIntake();
                    launcher.resetFeeder();
                    Launcher.LaunchState = Launcher.LaunchState.IDLE;
                    launcher.stopLauncher();
                    state = State.GO_TO_END_POSE;
                    driveTimer.reset();
                }
                break;
            case GO_TO_END_POSE:
                if(!follower.isBusy()){
                    intake.startIntake();
                    follower.followPath(launchingToPickup2);
                    driveTimer.reset();
                    state = State.FINISHED;
                }
                break;
            case FINISHED:
                if(!follower.isBusy()) {
                    if(driveTimer.seconds() > 2)
                        intake.stopIntake();
                }
                break;

            default:
                break;

        }


    }


    private void doAprilTag() {
        //Update the vision portal
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24); // TAG ID 24 is the red goal
        //aprilTagWebcam.displayDetectionTelemetry(id24);
        // NOTE: we will need a separate OPMODE (otherwise identical) that sets the target TAGID to BLUE (#20)
        if (id24 != null && id24.ftcPose != null) {
            numMissingTagReads = 0;
            double angleToTag = id24.ftcPose.bearing;
            //turret.changeTurretByDegrees(angleToTag);

            double distanceToGoalCM = id24.ftcPose.range;
            launcher.setMotorVelocityForDistance(distanceToGoalCM);
            led.setLEDGreen();
            // NOTE: use this after distance vs speed has been measured and calibrated
            //launcher.setMotorVelocityForDistance(distanceToGoalCM);
        } else if (numMissingTagReads < 100) {
            numMissingTagReads++;
            led.setLEDBlue();
        } else {
            // if we can't see the target/            // default back to neutral/default
            //turret.resetTurret();
            // and turn launch motors off
            launcher.stopLauncher();
            turret.resetTurret();
            led.setLEDRed();
        }
    }
}



