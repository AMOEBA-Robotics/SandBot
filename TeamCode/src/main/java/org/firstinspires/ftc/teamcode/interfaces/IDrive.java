package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public interface IDrive {
  public enum DriveMode {
    MECANUM,
    TANK,
    SWERVE
  }

  void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed);

  void updateRaw(
      Telemetry telemetry,
      boolean isLeftStickPressed,
      double leftStickX,
      double leftStickY,
      double rightStickX,
      double rightStickY,
      double speed,
      double rotSpeed);

  String getDriveModeName();

  DriveMode getDriveMode();
}
