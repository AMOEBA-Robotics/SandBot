package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Minimal IMU wrapper for the Control Hub's universal IMU (SDK 8.1+).
 *
 * <p>
 * Core functionality: retrieves the robot's heading (yaw) with a fixed offset. Defaults: - headingOffset
 * = PI/2 radians (90°) - getHeading() returns radians normalized to [-PI, PI] - getHeadingDegrees()
 * returns degrees normalized to [-180, 180]
 */
public class IMUWrapper {

  private final IMU imu;
  private double headingOffset = Math.PI / 2; // default 90° offset

  /**
   * Initializes the IMU with logo up / USB forward orientation.
   *
   * @param hardwareMap the OpMode's hardwareMap
   * @param imuName     the configured name of the IMU
   */

  public IMUWrapper(HardwareMap hardwareMap) {
    this(hardwareMap, "imu");
  }

  public IMUWrapper(HardwareMap hardwareMap, String imuName) {
    imu = hardwareMap.get(IMU.class, imuName);
    RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    IMU.Parameters params = new IMU.Parameters(orientation);
    imu.initialize(params);
  }

  /**
   * Returns the robot's heading in radians, applying the fixed offset and normalizing to [-PI, PI].
   *
   * @return heading in radians
   */
  public double getHeading() {
    double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    return normalizeRadians(raw + headingOffset);
  }

  /**
   * Returns the robot's heading in degrees, applying the fixed offset and normalizing to [-180, 180].
   *
   * @return heading in degrees
   */
  public double getHeadingDegrees() {
    double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    return normalizeDegrees(raw + Math.toDegrees(headingOffset));
  }

  /**
   * Sets a custom heading offset in radians.
   *
   * @param offsetRadians new offset to add to raw heading
   */
  public void setHeadingOffset(double offsetRadians) {
    this.headingOffset = offsetRadians;
  }

  /** Resets the heading offset back to the default PI/2 radians (90°). */
  public void resetHeadingOffset() {
    this.headingOffset = Math.PI / 2;
  }

  // Normalize angle to [-PI, PI]
  private double normalizeRadians(double angle) {
    while (angle <= -Math.PI)
      angle += 2 * Math.PI;
    while (angle > Math.PI)
      angle -= 2 * Math.PI;
    return angle;
  }

  // Normalize angle to [-180, 180]
  private double normalizeDegrees(double angle) {
    while (angle <= -180)
      angle += 360;
    while (angle > 180)
      angle -= 360;
    return angle;
  }
}
