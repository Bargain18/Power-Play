package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    public Gamepad gamepad;
    public Gamepad current = new Gamepad();
    public Gamepad previous = new Gamepad();

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        try {
            previous.copy(current);
            current.copy(gamepad);
        } catch (Exception e) {
            telemetry.addData("Error with gamepads", e);
        }
    }
}
