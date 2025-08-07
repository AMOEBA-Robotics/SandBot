package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.PIDFController;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name="Servo Pod Test", group="Test")
public class ServoPodTest extends OpMode {
    public static double angle = 90.0;

    CRServo servoPod;
    AnalogInput servoPodInput;

    public static double Kp = 0.03;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.0;

    PIDFController pidfController = new PIDFController(Kp, Ki, Kd, Kf);

    @Override
    public void init() {
        servoPodInput = hardwareMap.get(AnalogInput.class, "servoPodInput");
        servoPod = hardwareMap.get(CRServo.class, "servoPod");
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

    @Override
    public void loop() {
        double position = servoPodInput.getVoltage() / 3.3 * 360;
//        position += shortestAngleToTarget(position, angle);
        double power = Math.max(-1.0, Math.min(pidfController.calculateAngular(position, angle), 1.0));

        telemetry.addData("Encoder Voltage:", servoPodInput.getVoltage());
        telemetry.addData("Servo Current:", position);
        telemetry.addData("Servo Target:", angle);
        telemetry.addData("Servo Power", power);
        telemetry.update();
    }
}
