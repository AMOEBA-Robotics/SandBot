package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class MecanumDrive implements IDrive {

  final DcMotor frontLeftMotor;
  final DcMotor backLeftMotor;
  final DcMotor frontRightMotor;
  final DcMotor backRightMotor;

  public MecanumDrive(HardwareMap hardwareMap) {
    frontLeftMotor = hardwareMap.dcMotor.get("fL");
    backLeftMotor = hardwareMap.dcMotor.get("bL");
    frontRightMotor = hardwareMap.dcMotor.get("fR");
    backRightMotor = hardwareMap.dcMotor.get("bR");

    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  @Override
  public void update(
      Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
    updateRaw(
        telemetry,
        joystickWrapper.gamepad1GetLeftStickDown(),
        joystickWrapper.gamepad1GetLeftStickX(),
        joystickWrapper.gamepad1GetLeftStickY(),
        joystickWrapper.gamepad1GetRightStickX(),
        joystickWrapper.gamepad1GetRightStickY(),
        speed,
        rotSpeed);
  }

  @Override
  public void updateRaw(
      Telemetry telemetry,
      boolean isLeftStickPressed,
      double leftStickX,
      double leftStickY,
      double rightStickX,
      double rightStickY,
      double speed,
      double rotSpeed) {
    double y = -leftStickY;
    double x = leftStickX * 1.1;
    double rx = rightStickX * rotSpeed;

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower * speed);
    backLeftMotor.setPower(backLeftPower * speed);
    frontRightMotor.setPower(frontRightPower * speed);
    backRightMotor.setPower(backRightPower * speed);
  }

  @Override
  public String getDriveModeName() {
    return "Mecanum Drive";
  }

  @Override
  public DriveMode getDriveMode() {
    return DriveMode.MECANUM;
  }
}
