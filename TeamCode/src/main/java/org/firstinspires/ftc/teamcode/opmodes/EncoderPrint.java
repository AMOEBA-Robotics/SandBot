package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class EncoderPrint extends OpMode {

    AnalogInput servoPodInput;

    @Override
    public void init() {
        servoPodInput = hardwareMap.get(AnalogInput.class, "servoPodInput");
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Voltage:", servoPodInput.getVoltage());
        telemetry.update();
    }
}
