package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.SwerveDrive;
import org.firstinspires.ftc.teamcode.wrappers.IMUWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;


@TeleOp
public class SwerveDriveTest extends LinearOpMode {
  JoystickWrapper joystickWrapper;
  SwerveDrive swerveDrive;

  @Override
  public void runOpMode() {
    swerveDrive = new SwerveDrive(hardwareMap, new IMUWrapper(hardwareMap), new double[] { 5, 6 },
        new double[] { -5, 6 }, new double[] { 5, -6 }, new double[] { -5, -6 });

    waitForStart();

    while (opModeIsActive()) {
      swerveDrive.update(telemetry, new JoystickWrapper(gamepad1, null), 1.0, 1.0);
      telemetry.update();
    }
  }
}
