package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

public class RobotAuto extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    Launcher launcher = new Launcher();

    enum State {
        MOVE_FORWARD,
        FINISHED
    }
    State state = State.MOVE_FORWARD;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.init(hardwareMap);

        state = State.MOVE_FORWARD;
    }

    @Override
    public void loop() {
        telemetry.addData("Current state", state);

        switch (state) {
            case MOVE_FORWARD:
                break;
            case FINISHED:
                break;
            default:

        }

    }
}
