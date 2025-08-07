package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class InitialState extends BaseState {

    public InitialState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        if(prevState == null) {
            // Initialize robot to starting position
            // robot.setServoPosition("servo1", 0.5);
            // robot.setMotorTargetPosition("motor1", 0);
        } else {
            RobotTaskSeries resetSeries = new RobotTaskSeries();
            // resetSeries.add(createServoTask(robot, "servo1", 0.5, 1000, "Reset Servo", true));
            // resetSeries.add(createMotorTask(robot, "motor1", 0, 1000, "Reset Motor", false));
            taskArrayList.add(resetSeries);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.INITIAL;
    }
}
