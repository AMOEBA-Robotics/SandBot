package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.interfaces.IDrive.DriveMode;
import org.firstinspires.ftc.teamcode.robot.SwervePod;
import org.firstinspires.ftc.teamcode.robot.Vector;
import org.firstinspires.ftc.teamcode.wrappers.IMUWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SwerveDrive implements IDrive {
  private SwervePod[] pods;
  private IMUWrapper imu;

  PIDCoefficientsEx swervePid = new PIDCoefficientsEx(.1, 0, 0, 99999, 0, 1);

  /**
   * Constructs a SwerveDrive with the specified pods.
   *
   * @param imu         The IMUWrapper used for orientation and heading.
   * @param pod1Offsets Offsets for the first pod (xOffset, yOffset).
   * @param pod2Offsets Offsets for the second pod (xOffset, yOffset).
   * @param pod3Offsets Offsets for the third pod (xOffset, yOffset).
   * @param pod4Offsets Offsets for the fourth pod (xOffset, yOffset).
   */
  public SwerveDrive(HardwareMap hardwareMap, IMUWrapper imu, double[] pod1Offsets, double[] pod2Offsets,
      double[] pod3Offsets, double[] pod4Offsets) {
    pods = new SwervePod[4];
    pods[0] = new SwervePod("swerveServo1", "swerveInput1", "swerveMotor1", hardwareMap, swervePid,
        false, 0, pod1Offsets);
    pods[1] = new SwervePod("swerveServo2", "swerveInput2", "swerveMotor2", hardwareMap, swervePid,
        false, 0, pod2Offsets);
    pods[2] = new SwervePod("swerveServo3", "swerveInput3", "swerveMotor3", hardwareMap, swervePid,
        false, 0, pod3Offsets);
    pods[3] = new SwervePod("swerveServo4", "swerveInput4", "swerveMotor4", hardwareMap, swervePid,
        false, 0, pod4Offsets);
    this.imu = imu;
  }

  @Override
  public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed,
      double rotSpeed) {
    updateRaw(telemetry, joystickWrapper.gamepad1GetLeftStickDown(),
        joystickWrapper.gamepad1GetLeftStickX(), joystickWrapper.gamepad1GetLeftStickY(),
        joystickWrapper.gamepad1GetRightStickX(), joystickWrapper.gamepad1GetRightStickY(), speed,
        rotSpeed);
  }

  public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX,
      double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {
    double y = -leftStickY;
    double x = leftStickX;
    double rx = (Math.abs(rightStickX) < 0.05) ? 0 : rightStickX * rotSpeed;

    Vector[] podVectors = new Vector[pods.length];

    for (int podNum = 0; podNum < pods.length; podNum++) {
      SwervePod pod = pods[podNum];
      Vector rawTrans = new Vector(x, y);
      Vector translation = rawTrans.getMagnitude() < 0.05 ? new Vector(0, 0)
          : rawTrans.rotate(-imu.getHeading()).multiply(speed);
      Vector rotation = (new Vector(-pod.getYOffset(), pod.getXOffset())).multiply(rx); // Right stick
                                                                                        // input
      podVectors[podNum] = translation.add(rotation);
    }
    double maxMagnitude = 1;
    for (int i = 0; i < podVectors.length; i++) {
      maxMagnitude = Math.max(maxMagnitude, podVectors[i].getMagnitude());
    }
    if (maxMagnitude > 1) {
      maxMagnitude = 1;
    }
    for (int podNum = 0; podNum < pods.length; podNum++) {
      Vector finalVector = podVectors[podNum].divide(maxMagnitude); // Normalize vectors
      pods[podNum].move(finalVector.getArgument(), finalVector.getMagnitude());
    }
  }

  @Override
  public String getDriveModeName() {
    return "Swerve Drive";
  }

  @Override
  public DriveMode getDriveMode() {
    return DriveMode.SWERVE;
  }
}
