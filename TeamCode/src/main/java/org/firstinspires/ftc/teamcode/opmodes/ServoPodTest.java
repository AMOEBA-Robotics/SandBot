package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.PIDFController;
import org.opencv.core.Mat;

import com.acmerobotics.dashboard.config.Config;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

@Config
@TeleOp(name="Servo Pod Test", group="Test")
public class ServoPodTest extends OpMode {
    public static double angle = 90.0;

    CRServo servoPod;
    AnalogInput servoPodInput;


    public static double Kp = 0.5;
    public static double Ki = 0.0;
    public static double Kd = 1;
    public static double Kf = 0.0;

    //PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);



    PIDEx pidController;
    AngleController controller;

    @Override
    public void init() {


        PIDCoefficientsEx pidCoefs = new PIDCoefficientsEx(
                Kp,     // Proportional gain
                Ki,    // Integral gain
                Kd,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidController = new PIDEx(pidCoefs);

        controller = new AngleController(pidController);



        servoPodInput = hardwareMap.get(AnalogInput.class, "servoPodInput");
        servoPod = hardwareMap.get(CRServo.class, "servoPod");
        servoPod.setPower(0);
    }

    public double shortestAngleToTarget(double current, double target) {
        current = ((current % 360) + 360) % 360;
        target = ((target % 360) + 360) % 360;

        double delta = target - current;

        if (delta > 180) delta -= 360;
        else if(delta <= -180) delta += 360;

        if (Math.abs(delta) == 180) return -180;

        return delta;
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }



    @Override
    public void loop() {
        double actual = servoPodInput.getVoltage() / 3.3 * 360;

        double headingError = angleWrap((angleWrap(Math.toRadians(actual) - angleWrap(Math.toRadians(angle)))));
        double power = pidController.calculate(actual+headingError, actual);

        servoPod.setPower(power);

        telemetry.addData("Encoder Voltage:", servoPodInput.getVoltage());
        telemetry.addData("Servo Actual:", actual);
        telemetry.addData("Servo error:", headingError);
        telemetry.addData("Servo Target:", angle);
        telemetry.addData("Servo Power", power);
        telemetry.update();
    }
}
