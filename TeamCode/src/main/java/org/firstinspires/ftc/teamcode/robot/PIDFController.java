package org.firstinspires.ftc.teamcode.robot;

public class PIDFController {

    private double kP, kI, kD, kF;
    private double setPoint;
    private double currentPoint;
    private double minIntegral, maxIntegral;

    private double pError;
    private double vError;

    private double totalError;
    private double prevError;

    private double pErrorTolerance = 0.05;
    private double vErrorTolerance = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0, 0);
    }

    public PIDFController(double kP, double kI, double kD, double kF, double setPoint, double currentPoint) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.setPoint = setPoint;
        this.currentPoint = currentPoint;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        pError = setPoint - currentPoint;
        reset();
    }

    public void reset() {
        totalError = 0;
        prevError = 0;
        lastTimeStamp = 0;
    }

    public void setSetPoint(double sp) {
        setPoint = sp;
    }

    public void setIntegralBounds(double min, double max) {
        minIntegral = min;
        maxIntegral = max;
    }

    public void setPositionErrorTolerance(double tol) {
        pErrorTolerance = tol;
    }

    public boolean atSetPoint() {
        return Math.abs(pError) < pErrorTolerance && Math.abs(vError) < vErrorTolerance;
    }

    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    public double calculate(double cp, double sp) {
        setSetPoint(sp);
        return calculate(cp);
    }

    public double calculate(double cp) {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) {
            lastTimeStamp = currentTimeStamp;
            currentPoint = cp;
            return 0;
        }

        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        currentPoint = cp;
        pError = setPoint - currentPoint;

        double deltaError = pError - prevError;
        vError = period > 1E-6 ? deltaError / period : 0;
        prevError = pError;

        totalError += pError * period;
        totalError = Math.max(minIntegral, Math.min(maxIntegral, totalError));

        return kP * pError + kI * totalError + kD * vError + kF * setPoint;
    }

    public double calculateAngular(double cp, double sp) {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) {
            lastTimeStamp = currentTimeStamp;
            currentPoint = cp;
            return 0;
        }

        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        cp = ((cp % 360) + 360) % 360;
        sp = ((sp % 360) + 360) % 360;

        double delta = sp - cp;
        if (delta > 180) delta -= 360;
        else if (delta <= -180) delta += 360;

        pError = delta;
        vError = period > 1E-6 ? (pError - prevError) / period : 0;
        prevError = pError;

        totalError += pError * period;
        totalError = Math.max(minIntegral, Math.min(maxIntegral, totalError));

        return kP * pError + kI * totalError + kD * vError + kF * sp;
    }

}