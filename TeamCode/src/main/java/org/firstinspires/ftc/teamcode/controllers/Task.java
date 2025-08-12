package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Task extends RobotTaskImpl {
  TaskListener _listener;

  private Task(TaskListener _listener) {
    this._listener = _listener;
  }

  public interface TaskListener {
    void execute();

    boolean endCondition();
  }

  boolean _hasStarted = false;

  @Override
  public void execute(Telemetry telemetry) {
    if (!_hasStarted)
      _hasStarted = true;
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
    return _hasStarted;
  }

  @Override
  public boolean isComplete() {
    if (!hasStarted()) {
      return false;
    }

    return _listener.endCondition();
  }
}
