package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {

    private IRobot currentState;
    private final JoystickWrapper joystick;
    private final HardwareMap hardwareMap;

    private final Map<State, Supplier<IRobot>> instanceStateMap = new HashMap<>();
    private final Map<IDrive.DriveMode, IDrive> driveMap = new HashMap<>();
    private IDrive.DriveMode currentDriveMode = IDrive.DriveMode.MECANUM;
    private IDrive currentDrive;
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        this.hardwareMap = hardwareMap;

        initializeHardware();

        instanceStateMap.put(State.INITIAL, () -> new InitialState(joystick));
//        instanceStateMap.put(State.DRIVE_TEST, () -> new DriveState(joystick));
        switchState(State.INITIAL);
    }

    private void initializeHardware() {
        driveMap.put(IDrive.DriveMode.MECANUM, new MecanumDrive(hardwareMap));
        driveMap.put(IDrive.DriveMode.TANK, new TankDrive(hardwareMap));
    }

    public void execute(Telemetry telemetry) {
        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        telemetry.update();
    }

    public State getCurrentState() {
        if (currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }
    public void updateDrive(Telemetry telemetry, double speed, double rotSpeed) {
        currentDrive.update(telemetry, joystick, speed, rotSpeed);
    }

    public void switchDriveMode(IDrive.DriveMode newMode) {
        if (driveMap.containsKey(newMode)) {
            currentDriveMode = newMode;
            currentDrive = driveMap.get(newMode);
        }
    }

    public String getCurrentDriveModeName() {
        return currentDrive.getDriveModeName();
    }


}