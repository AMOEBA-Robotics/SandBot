package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

public class ParallelTask extends RobotTaskImpl {

    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    public void add(IRobotTask task) {
        tasks.add(task);
    }

    @Override
    public void execute(Telemetry telemetry) {
        for (IRobotTask task : tasks) {
            task.execute(telemetry);
        }
    }

    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public boolean hasStarted() {
        if (tasks.size() > 0) {
            return tasks.get(0).hasStarted();
        } else {
            return true;
        }
    }

    @Override
    public boolean isRunning() {
        if (!hasStarted()) {
            return false;
        } else {
            return !isComplete();
        }
    }

    @Override
    public boolean isComplete() {
        boolean isComplete = true;
        for (IRobotTask task : tasks) {
            if (!task.isComplete()) {
                isComplete = false;
                break;
            }
        }
        return isComplete;
    }
}
