package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class SwervePod {
  // --- Hardware ---
  private final AnalogInput turnEncoder; // analog absolute encoder
  private final CRServo turnServo; // continuous rotation servo for azimuth
  private final DcMotor driveMotor; // wheel drive motor

  // --- Control ---
  private final PIDEx turnPID; // same family as in the first file

  // --- Geometry / calibration ---
  private final double angleOffsetDeg; // electrical/mechanical zero offset in degrees
  private final double xOffset;
  private final double yOffset;

  // REV hubs use 0–3.3V on analog inputs
  private static final double ANALOG_REF_V = 3.3;

  // ------------ Constructors ------------
  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean driveReversed, double angleOffsetDeg,
      double[] offsets) {
    this(servoName, inputName, motorName, hardwareMap, pidCoefficients, driveReversed, angleOffsetDeg,
        offsets[0], offsets[1]);
  }

  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean driveReversed, double angleOffsetDeg, double xOffset,
      double yOffset) {
    this.turnServo = hardwareMap.get(CRServo.class, servoName);
    this.turnEncoder = hardwareMap.get(AnalogInput.class, inputName);
    this.driveMotor = hardwareMap.get(DcMotor.class, motorName);

    this.turnPID = new PIDEx(pidCoefficients);
    this.angleOffsetDeg = angleOffsetDeg;
    this.xOffset = xOffset;
    this.yOffset = yOffset;

    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (driveReversed)
      driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  // ------------ Public API ------------
  /**
   * Command this pod to a wheel heading (degrees) and drive power [-1, 1].
   * 
   * @param targetAngleDeg desired wheel azimuth in DEGREES (field or robot frame handled by caller)
   * @param drivePower     drive power [-1, 1]
   */
  public void move(double targetAngleDeg, double drivePower) {
    // Current absolute angle from encoder (deg), corrected by calibration offset
    double actualDeg = normalize0To360(getRawAngleDeg() + angleOffsetDeg);
    double desiredDeg = normalize0To360(targetAngleDeg);

    // Shortest-path error in [-180, 180]
    double error = shortestAngleToTarget(actualDeg, desiredDeg);

    // If we'd need to turn > 90°, flip 180° and reverse the wheel drive to minimize
    // rotation
    if (Math.abs(error) > 90.0) {
      desiredDeg = normalize0To360(desiredDeg + 180.0);
      drivePower = -drivePower;
      error = shortestAngleToTarget(actualDeg, desiredDeg); // recompute after flip
    }

    // Setpoint close to current so PID follows shortest path, mirroring the first
    // file's pattern
    double setpointDeg = actualDeg + error;

    // PIDEx.calculate(setpoint, measurement) -> servo command; clamp to [-1, 1]
    double servoCmd = clamp(turnPID.calculate(setpointDeg, actualDeg), -1.0, 1.0);
    turnServo.setPower(servoCmd);

    // Drive motor last, after finalizing direction/flip logic
    driveMotor.setPower(drivePower);
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  // ------------ Helpers (mirroring the first file’s style) ------------
  private double getRawAngleDeg() {
    // Map 0–3.3 V -> 0–360°
    return (turnEncoder.getVoltage() / ANALOG_REF_V) * 360.0;
  }

  /** Normalize angle to [0, 360). */
  private static double normalize0To360(double deg) {
    deg = deg % 360.0;
    if (deg < 0)
      deg += 360.0;
    return deg;
  }

  /**
   * Smallest signed delta from current to target in [-180, 180], matching the logic from your first
   * file.
   */
  private static double shortestAngleToTarget(double current, double target) {
    current = normalize0To360(current);
    target = normalize0To360(target);

    double delta = target - current;

    if (delta > 180)
      delta -= 360;
    else if (delta <= -180)
      delta += 360;

    // If exactly 180, choose -180 consistently to avoid dithering
    if (Math.abs(delta) == 180)
      return -180;

    return delta;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(v, hi));
  }
}
