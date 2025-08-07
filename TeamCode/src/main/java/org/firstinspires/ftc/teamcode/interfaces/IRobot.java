package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public interface IRobot {
    public enum State {
        INVALID,
        INITIAL,
        DRIVE_TEST
    }

    void initialize(Robot robot, IRobot prevState);
    void execute(Robot robot, Telemetry telemetry);
    IRobot.State getState();
}