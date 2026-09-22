package org.firstinspires.ftc.teamcode.buzzball.controls;

import com.technototes.library.control.CommandButton;
import com.technototes.library.control.CommandGamepad;
import org.firstinspires.ftc.teamcode.buzzball.Robot;
import org.firstinspires.ftc.teamcode.buzzball.commands.TeleCommands;

public class TestingController {

    public Robot robot;
    public CommandGamepad gamepad;

    public CommandButton motorPowerButton;
    public CommandButton motorVelocityButton;
    public CommandButton TurretMoveToPose;

    public TestingController(CommandGamepad g, Robot r) {
        robot = r;
        gamepad = g;
        AssignNamedControllerButton();
        bindButtonControls();
    }

    private void AssignNamedControllerButton() {
        // motorPowerButton = gamepad.ps_circle;
        // motorVelocityButton = gamepad.ps_triangle;
        TurretMoveToPose = gamepad.ps_cross;
    }

    private void bindButtonControls() {
        motorPowerButton.whenPressed(TeleCommands.MotorPowerTest(robot));
        motorVelocityButton.whenPressed(TeleCommands.MotorVelocityTest(robot));
    }
}
