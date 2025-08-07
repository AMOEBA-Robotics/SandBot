package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobotPidMechanism;

abstract class RobotPid implements IRobotPidMechanism {

    private double kP, kI, kD;
    private double minOutput, maxOutput;
    private int targetPosition;
    private int maxPosition;
    private int minPosition;

    private double integral = 0;
    private double previousError = 0;
    private long lastTime = 0;

    public boolean showTelemetry = true;

    public RobotPid(double kp, double ki, double kd, double minOutput, double maxOutput, int inMaxPosition, int inMinPosition) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.targetPosition = 0;
        this.maxPosition = inMaxPosition;
        this.minPosition = inMinPosition;
        this.lastTime = System.currentTimeMillis();
    }

    public void update(Telemetry telemetry) {
        int currentPosition = getCurrentPosition();
        double power = calculatePID(currentPosition, targetPosition);

        if(showTelemetry && telemetry != null) {
            telemetry.addData(getName() + ": currentPosition", currentPosition);
            telemetry.addData(getName() + ": targetPosition", targetPosition);
            telemetry.addData(getName() + ": Power", power);
        }
        onSetPower(power);
    }

    private double calculatePID(int currentPosition, int targetPosition) {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;
        double error = targetPosition - currentPosition;

        integral += error * deltaTime;
        double derivative = (deltaTime > 0) ? (error - previousError) / deltaTime : 0;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        output = Math.max(minOutput, Math.min(maxOutput, output));

        previousError = error;
        lastTime = currentTime;

        return output;
    }

    public int evaluateConstraints(int position) {
        if(position < minPosition) {
            position = minPosition;
        } else if(position > maxPosition) {
            position = maxPosition;
        }
        return position;
    }

    public void setTargetPosition(int position) {
        targetPosition = evaluateConstraints(position);
    }

    public void increaseTargetPosition(int offset) {
        int newTargetPosition = targetPosition + offset;
        targetPosition = evaluateConstraints(newTargetPosition);
    }

    public void resetPID() {
        integral = 0;
        previousError = 0;
        lastTime = System.currentTimeMillis();
    }


    public int getTargetPosition() {
        return targetPosition;
    }
}