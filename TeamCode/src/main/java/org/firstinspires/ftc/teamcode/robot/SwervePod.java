package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class SwervePod {
  private final AnalogInput servoPodInput;
  private final CRServo servoPod;
  private final DcMotor motor;

  private final PIDEx anglePID;

  private final double angleOffset;

  private final double xOffset;
  private final double yOffset;

  private boolean powerReversed = false;

  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean reversed, double angleOffset, double[] offsets) {
    this(servoName, inputName, motorName, hardwareMap, pidCoefficients, reversed, angleOffset,
        offsets[0], offsets[1]);
  }

  public SwervePod(String servoName, String inputName, String motorName, HardwareMap hardwareMap,
      PIDCoefficientsEx pidCoefficients, boolean reversed, double angleOffset, double xOffset,
      double yOffset) {
    this.xOffset = xOffset;
    this.yOffset = yOffset;

    this.servoPod = hardwareMap.get(CRServo.class, servoName);
    this.servoPodInput = hardwareMap.get(AnalogInput.class, inputName);
    this.motor = hardwareMap.get(DcMotor.class, motorName);

    this.angleOffset = angleOffset;

    this.anglePID = new PIDEx(pidCoefficients);

    this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (reversed)
      this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void move(double targetAngle, double power) {
    double position = (servoPodInput.getVoltage() / 3.3 * 360) + angleOffset;

    double pidf = anglePID.calculate(targetAngle, position);
    servoPod.setPower(Math.max(-1.0, Math.min(pidf, 1.0)));

    if (position >= 90 && position <= 270) {
      if (!powerReversed) {
        power *= -1;
        powerReversed = true;
      }
    } else {
      powerReversed = false;
    }

    motor.setPower(power);
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getXOffset() {
    return xOffset;
  }
}
