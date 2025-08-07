package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwervePod {
  private final AnalogInput servoPodInput;
  private final CRServo servoPod;
  private final DcMotor motor;

  private final PIDFController anglePIDF;

  private final double angleOffset;

  private final double xOffset;
  private final double yOffset;

  private boolean powerReversed = false;

  public SwervePod(String[] hwNames, HardwareMap hardwareMap, PIDFCoefficients pidfCoefficients, boolean reversed, double angleOffset, double xOffset, double yOffset) {
    this.xOffset = xOffset;
    this.yOffset = yOffset;

    this.servoPod = hardwareMap.get(CRServo.class, hwNames[0]);
    this.servoPodInput = hardwareMap.get(AnalogInput.class, hwNames[1]);
    this.motor = hardwareMap.get(DcMotor.class, hwNames[2]);

    this.angleOffset = angleOffset;

    this.anglePIDF = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);

    this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if(reversed) this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void move(double targetAngle, double power) {
    double position = (servoPodInput.getVoltage() / 3.3 * 360) + angleOffset;

    double pidf = anglePIDF.calculateAngular(position, targetAngle);
    servoPod.setPower(Math.max(-1.0, Math.min(pidf, 1.0)));


    if (position >= 90 && position <= 270) {
      if(!powerReversed) {
        power *= -1;
        powerReversed = true;
      }
    } else {
      powerReversed = false;
    }

    motor.setPower(power);
  }

  public double getYOffset() {return yOffset;}
  public double getXOffset() {return xOffset;}
}
