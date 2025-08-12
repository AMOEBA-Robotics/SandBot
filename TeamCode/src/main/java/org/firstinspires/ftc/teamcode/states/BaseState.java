package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.Task;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import java.util.ArrayList;

public abstract class BaseState implements IRobot {

    protected final JoystickWrapper joystick;
    ArrayList<IRobotTask> taskArrayList = new ArrayList<IRobotTask>();

    protected BaseState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

//    public static IRobotTask createServoTask(Robot robot, String servoName, double position, int duration, String name, boolean steps) {
//        return new Task(new CallBackTask.CallBackListener() {
//            @Override
//            public void setPosition(double value) {
//            }
//
//            @Override
//            public double getPosition() {
//                return 0;
//            }
//        }, position, duration, name, steps);
//    }
//
//    public static IRobotTask createMotorTask(Robot robot, String motorName, int position, int duration, String name, boolean steps) {
//        return new CallBackTask(new CallBackTask.CallBackListener() {
//            @Override
//            public void setPosition(double value) {
//            }
//
//            @Override
//            public double getPosition() {
//                return 0;
//            }
//        }, position, duration, name, steps);
//    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        // Default implementation
    }

    public void executeTasks(Telemetry telemetry) {
        if(!taskArrayList.isEmpty()) {
            boolean isStarted = taskArrayList.get(0).hasStarted();
            boolean isRunning = taskArrayList.get(0).isRunning();
            boolean isComplete = taskArrayList.get(0).isComplete();

            taskArrayList.get(0).execute(telemetry);

            if(isComplete){
                taskArrayList.remove(0);
            }
        }
    }
}
