package org.firstinspires.ftc.teamcode.twenty403.controls;

import com.technototes.library.control.CommandGamepad;
import org.firstinspires.ftc.teamcode.twenty403.Robot;

public class OperatorController {

    public Robot robot;
    public CommandGamepad gamepad;

    public OperatorController(CommandGamepad g, Robot r) {
        robot = r;
        gamepad = g;
        AssignNamedControllerButton();
        BindControls();
    }

    private void AssignNamedControllerButton() {}

    public void BindControls() {}
}
