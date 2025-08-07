package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class JoystickWrapper {

    public Gamepad gamepad1;
    Gamepad gamepad2;

    boolean gamepad1xPressed = false;
    boolean gamepad1yPressed = false;
    boolean gamepad1aPressed = false;
    boolean gamepad1bPressed = false;

    boolean gamepad1dUpPressed = false;
    boolean gamepad1dDownPressed = false;
    boolean gamepad1dRightPressed = false;
    boolean gamepad1dLeftPressed = false;

    boolean gamepad1leftBumperPressed = false;
    boolean gamepad1rightBumperPressed = false;

    boolean gamepad1leftStickPressed = false;
    boolean gamepad1rightStickPressed = false;

    public JoystickWrapper(Gamepad inGamepad1, Gamepad inGamepad2) {
        gamepad1 = inGamepad1;
        gamepad2 = inGamepad2;
    }

    public boolean gamepad1GetX() {
        if (!gamepad1xPressed && gamepad1.x) {
            gamepad1xPressed = true;
            return true;
        } else if (gamepad1xPressed && !gamepad1.x) {
            gamepad1xPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetY() {
        if (!gamepad1yPressed && gamepad1.y) {
            gamepad1yPressed = true;
            return true;
        } else if (gamepad1yPressed && !gamepad1.y) {
            gamepad1yPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetA() {
        if (!gamepad1aPressed && gamepad1.a) {
            gamepad1aPressed = true;
            return true;
        } else if (gamepad1aPressed && !gamepad1.a) {
            gamepad1aPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetB() {
        if (!gamepad1bPressed && gamepad1.b) {
            gamepad1bPressed = true;
            return true;
        } else if (gamepad1bPressed && !gamepad1.b) {
            gamepad1bPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDUp() {
        if (!gamepad1dUpPressed && gamepad1.dpad_up) {
            gamepad1dUpPressed = true;
            return true;
        } else if (gamepad1dUpPressed && !gamepad1.dpad_up) {
            gamepad1dUpPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDDown() {
        if (!gamepad1dDownPressed && gamepad1.dpad_down) {
            gamepad1dDownPressed = true;
            return true;
        } else if (gamepad1dDownPressed && !gamepad1.dpad_down) {
            gamepad1dDownPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDRight() {
        if (!gamepad1dRightPressed && gamepad1.dpad_right) {
            gamepad1dRightPressed = true;
            return true;
        } else if (gamepad1dRightPressed && !gamepad1.dpad_right) {
            gamepad1dRightPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDLeft() {
        if (!gamepad1dLeftPressed && gamepad1.dpad_left) {
            gamepad1dLeftPressed = true;
            return true;
        } else if (gamepad1dLeftPressed && !gamepad1.dpad_left) {
            gamepad1dLeftPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftBumperDown() {
        if (!gamepad1leftBumperPressed && gamepad1.left_bumper) {
            gamepad1leftBumperPressed = true;
            return true;
        } else if (gamepad1leftBumperPressed && !gamepad1.left_bumper) {
            gamepad1leftBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightBumperDown() {
        if (!gamepad1rightBumperPressed && gamepad1.right_bumper) {
            gamepad1rightBumperPressed = true;
            return true;
        } else if (gamepad1rightBumperPressed && !gamepad1.right_bumper) {
            gamepad1rightBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftStickDown() {
        if (!gamepad1leftStickPressed && gamepad1.left_stick_button) {
            gamepad1leftStickPressed = true;
            return true;
        } else if (gamepad1leftStickPressed && !gamepad1.left_stick_button) {
            gamepad1leftStickPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightStickDown() {
        if (!gamepad1rightStickPressed && gamepad1.right_stick_button) {
            gamepad1rightStickPressed = true;
            return true;
        } else if (gamepad1rightStickPressed && !gamepad1.right_stick_button) {
            gamepad1rightStickPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightBumperRaw() {
        return gamepad1.right_bumper;
    }

    public boolean gamepad1GetLeftBumperRaw() {
        return gamepad1.left_bumper;
    }

    public double gamepad1GetRightTrigger() {
        return gamepad1.right_trigger;
    }

    public double gamepad1GetLeftTrigger() {
        return gamepad1.left_trigger;
    }

    public double gamepad1GetRightStickX() {
        return gamepad1.right_stick_x;
    }

    public double gamepad1GetRightStickY() {
        return gamepad1.right_stick_y;
    }

    public double gamepad1GetLeftStickX() {
        return gamepad1.left_stick_x;
    }

    public double gamepad1GetLeftStickY() {
        return gamepad1.left_stick_y;
    }
}