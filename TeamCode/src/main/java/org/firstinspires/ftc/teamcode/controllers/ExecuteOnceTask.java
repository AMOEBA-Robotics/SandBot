package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExecuteOnceTask extends RobotTaskImpl {

    public interface ExecuteListener {
        void execute();
    }

    private final ExecuteListener _listener;
    String _name;
    boolean _executed = false;

    public ExecuteOnceTask(ExecuteListener inListener) {
        _listener = inListener;
    }

    public boolean isBlocking() {
        return true;
    }

    @Override
    public boolean hasStarted() {
        return isComplete();
    }

    public boolean isRunning() {
        return false;
    }

    @Override
    public void execute(Telemetry telemetry) {
        if (isComplete()) {
            _listener.execute();
        }
        _executed = true;
    }

    @Override
    public boolean isComplete() {
        return _executed;
    }
}
