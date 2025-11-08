package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OperatorInputs {
    public Gamepad gamepad;
    public Telemetry telemetry;
    private boolean prevLeftTrigger = false;
    public boolean intakeToggle = false;
    public double shooterSpeed = 1500.0;
    public boolean isShooterOn = false;
    public boolean shootActive = false;
    public double drive, turn;

    public OperatorInputs(Gamepad gamepad, Telemetry telemetry) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void update() {
        drive = -gamepad.left_stick_y * 0.72;
        turn = gamepad.right_stick_x;

        if (gamepad.left_trigger > 0.05 && !prevLeftTrigger) intakeToggle = !intakeToggle;
        shootActive = gamepad.right_trigger > 0.05;

        prevLeftTrigger = gamepad.left_trigger > 0.05;

        if (gamepad.dpadDownWasPressed()) isShooterOn = !isShooterOn;

        // just for tweaks during tests
        if (gamepad.dpadLeftWasPressed()) shooterSpeed -= 50.0;
        if (gamepad.dpadRightWasPressed()) shooterSpeed += 50.0;
        shooterSpeed = Math.max(600.0, Math.min(2850.0, shooterSpeed));
    }
}
