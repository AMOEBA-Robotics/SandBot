package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.IMUWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SwerveDrive implements IDrive {
    private SwervePod[] pods;
    private IMUWrapper imu;

    /**
     * Constructs a SwerveDrive with the specified pods.
     *
     * @param imu The IMUWrapper used for orientation and heading.
     * @param pods The array of SwervePods that make up the swerve drive system.
     */
    public SwerveDrive(IMUWrapper imu, SwervePod[] pods) {
        this.pods = pods;
        this.imu = imu;
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
        double x = leftStickX;
        double rx = (Math.abs(rightStickX) < 0.05) ? 0 : rightStickX * rotSpeed;

        Vector[] podVectors = new Vector[pods.length];

        for (int podNum = 0; podNum < pods.length; podNum++) {
            SwervePod pod = pods[podNum];
            Vector rawTrans = new Vector(x, y);
            Vector translation =
                    rawTrans.getMagnitude() < 0.05
                            ? new Vector(0, 0)
                            : rawTrans.rotate(-imu.getHeading()).multiply(speed);
            Vector rotation =
                    (new Vector(-pod.getYOffset(), pod.getXOffset()))
                            .multiply(rx); // Right stick input
            podVectors[podNum] = translation.add(rotation);
        }

        double maxMagnitude = 1;
        for (int i = 0; i < podVectors.length; i++) {
            maxMagnitude = Math.max(maxMagnitude, podVectors[i].getMagnitude());
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
