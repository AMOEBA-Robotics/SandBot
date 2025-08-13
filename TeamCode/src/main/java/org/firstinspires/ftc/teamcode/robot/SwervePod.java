package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class SwervePod {
  // --- Hardware ---
  private final AnalogInput turnEncoder; // absolute analog encoder for azimuth
  private final CRServo turnServo; // CR servo that steers the pod
  private final DcMotor driveMotor; // wheel drive motor

  // --- Control ---
  private final PIDEx turnPID; // PID for azimuth control

  // --- Geometry / calibration ---
  private final double angleOffsetDeg; // mechanical/electrical zero offset (deg)
  private final double xOffset; // pod x offset from robot center (m or in)
  private final double yOffset; // pod y offset from robot center (m or in)

  private Telemetry telemetry;
  private final String servoLabel;

  // REV analog reference voltage (0–3.3 V)
  private static final double ANALOG_REF_V = 3.3;

//  Telemetry telemetry;

  // ------------ Constructors ------------
  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean driveReversed, double angleOffsetDeg,
      double[] offsets, Telemetry telemetry) {
    this(servoName, inputName, motorName, hardwareMap, pidCoefficients, driveReversed, angleOffsetDeg,
        offsets[0], offsets[1], telemetry);
  }

  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean driveReversed, double angleOffsetDeg, double xOffset,
      double yOffset, Telemetry telemetry) {
    this.turnServo = hardwareMap.get(CRServo.class, servoName);
    this.turnEncoder = hardwareMap.get(AnalogInput.class, inputName);
    this.driveMotor = hardwareMap.get(DcMotor.class, motorName);

    this.turnPID = new PIDEx(pidCoefficients);
    this.angleOffsetDeg = angleOffsetDeg;
    this.xOffset = xOffset;
    this.yOffset = yOffset;

    this.telemetry = telemetry;

    this.servoLabel = servoName;

    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (driveReversed)
      driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  // ------------ Public API ------------
  /**
   * Command this pod to a wheel heading (degrees) and drive power [-1, 1]. Adds
   */
  public void move(double targetAngleDeg, double drivePower) {
    // Current absolute angle from encoder (deg), corrected by calibration offset
    telemetry.addData("Raw Input Angle ", Math.toDegrees(targetAngleDeg));
    double actualDeg = normalize0To360(getRawAngleDeg() + angleOffsetDeg);
    double desiredDeg = normalize0To360(Math.toDegrees(targetAngleDeg));

    // Shortest-path error in [-180, 180]
    double error = shortestAngleToTarget(actualDeg, desiredDeg);

    // Minimize rotation: flip + invert drive if > 90°
    if (Math.abs(error) > 90.0) {
      desiredDeg = normalize0To360(desiredDeg + 180.0);
      drivePower = -drivePower;
      error = shortestAngleToTarget(actualDeg, desiredDeg);
    }

    // Setpoint close to current so PID follows shortest path
    double setpointDeg = actualDeg + error;

    // PIDEx.calculate(setpoint, measurement) -> servo command; clamp to [-1, 1]
    double turnPower = -clamp(turnPID.calculate(setpointDeg, actualDeg), -1.0, 1.0);
    turnServo.setPower(turnPower);

    // Drive motor after finalizing direction/flip logic
    driveMotor.setPower(drivePower);

    if (telemetry != null) {
      telemetry.addData(servoLabel + " Angle", actualDeg);
      telemetry.addData(servoLabel + " Target", desiredDeg);
      telemetry.addData(servoLabel + " Turn Power", turnPower);
    }
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  // ------------ Helpers ------------
  private double getRawAngleDeg() {
    // Map 0–3.3 V -> 0–360°
    return (turnEncoder.getVoltage() / ANALOG_REF_V) * 360.0;
  }

  private static double normalize0To360(double deg) {
    deg = deg % 360.0;
    if (deg < 0)
      deg += 360.0;
    return deg;
  }

  /** Smallest signed delta from current to target in [-180, 180]. */
  private static double shortestAngleToTarget(double current, double target) {
    current = normalize0To360(current);
    target = normalize0To360(target);

    double delta = target - current;
    if (delta > 180)
      delta -= 360;
    else if (delta <= -180)
      delta += 360;

    if (Math.abs(delta) == 180)
      return -180;
    return delta;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(v, hi));
  }
}