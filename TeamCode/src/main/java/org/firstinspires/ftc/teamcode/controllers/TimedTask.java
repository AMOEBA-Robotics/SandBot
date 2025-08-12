package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TimedTask extends RobotTaskImpl {

    TimedTaskListener _listener;

    public interface TimedTaskListener {
        void execute();
    }

    public TimedTask(long inDuration, TimedTaskListener _listener) {
        _duration = inDuration;
        this._listener = _listener;
    }

    long _startTime = 0;
    long _duration = 0;

    @Override
    public void execute(Telemetry telemetry) {
        if (_startTime == 0) {
            _startTime = System.currentTimeMillis();
        }

        if (!isComplete()) {
            _listener.execute();
        }
    }

    @Override
    public boolean isBlocking() {
        return true; // Default blocking behavior
    }

    @Override
    public boolean isRunning() {
        if (isComplete()) {
            return false;
        } else {
            return hasStarted();
        }
    }

    @Override
    public boolean hasStarted() {
        return _startTime > 0;
    }

    @Override
    public boolean isComplete() {
        if (!hasStarted()) {
            return false;
        } else {
            long currentTime = System.currentTimeMillis();
            long diff = currentTime - _startTime;
            return diff >= _duration;
        }
    }
}
