package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class TankDrive implements IDrive {

    final DcMotor leftMotor1;
    final DcMotor leftMotor2;
    final DcMotor rightMotor1;
    final DcMotor rightMotor2;

    public TankDrive(HardwareMap hardwareMap) {
        leftMotor1 = hardwareMap.dcMotor.get("fL");
        leftMotor2 = hardwareMap.dcMotor.get("bL");
        rightMotor1 = hardwareMap.dcMotor.get("fR");
        rightMotor2 = hardwareMap.dcMotor.get("bR");

        leftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        updateRaw(telemetry, joystickWrapper.gamepad1GetLeftStickDown(),
                joystickWrapper.gamepad1GetLeftStickX(), joystickWrapper.gamepad1GetLeftStickY(),
                joystickWrapper.gamepad1GetRightStickX(), joystickWrapper.gamepad1GetRightStickY(),
                speed, rotSpeed);
    }

    @Override
    public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {
        double leftPower = -leftStickY * speed;
        double rightPower = -rightStickY * speed;

        leftMotor1.setPower(leftPower);
        leftMotor2.setPower(leftPower);
        rightMotor1.setPower(rightPower);
        rightMotor2.setPower(rightPower);
    }

    @Override
    public String getDriveModeName() {
        return "Tank Drive";
    }

    @Override
    public DriveMode getDriveMode() {
        return DriveMode.TANK;
    }
}