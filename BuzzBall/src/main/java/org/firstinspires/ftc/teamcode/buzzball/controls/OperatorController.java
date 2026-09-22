package org.firstinspires.ftc.teamcode.buzzball.controls;

import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.CommandButton;
import com.technototes.library.control.CommandGamepad;
import com.technototes.library.control.Stick;
import org.firstinspires.ftc.teamcode.buzzball.Hardware;
import org.firstinspires.ftc.teamcode.buzzball.Robot;
import org.firstinspires.ftc.teamcode.buzzball.Setup;
import org.firstinspires.ftc.teamcode.buzzball.commands.PedroDriver;
import org.firstinspires.ftc.teamcode.buzzball.commands.driving.DrivingCommands;

public class OperatorController {

    public Robot robot;
    public Hardware hardware;
    public CommandGamepad gamepad;

    public Stick driveLeftStick, driveRightStick;
    public CommandButton resetGyroButton;
    public CommandButton snailButton;

    public CommandButton override;

    public PedroDriver pedroDriver;

    public static double triggerThreshold = 0.1;

    public OperatorController(CommandGamepad g, Robot r) {
        this.robot = r;
        gamepad = g;
        override = g.leftTrigger.getAsButton(0.5);
        override = g.rightTrigger.getAsButton(0.5);

        AssignNamedControllerButton();
        if (Setup.Connected.DRIVEBASE) {
            bindDriveControls();
        }
    }

    private void AssignNamedControllerButton() {
        resetGyroButton = gamepad.ps_options;
        driveLeftStick = gamepad.leftStick;
        driveRightStick = gamepad.rightStick;
    }

    public void bindDriveControls() {
        pedroDriver = new PedroDriver(
            robot.follower,
            driveLeftStick,
            driveRightStick,
            robot.limelightSubsystem
        );
        CommandScheduler.scheduleJoystick(pedroDriver);

        // turboButton.whenPressed(DrivingCommands.TurboDriving(robot.drivebase));
        // turboButton.whenReleased(DrivingCommands.NormalDriving(robot.drivebase));
        snailButton.whenPressedReleased(
            DrivingCommands.SnailDriving(pedroDriver),
            DrivingCommands.NormalDriving(pedroDriver)
        );
        resetGyroButton.whenPressed(DrivingCommands.ResetGyro(pedroDriver));
        //MotorDecrease.whenPressed(TeleCommands.DecreaseMotor(robot));
        //MotorIncrease.whenPressed(TeleCommands.IncreaseMotor(robot));

        // autoAim.whilePressed(new LLPipelineChangeCommand(hardware.limelight, Setup.HardwareNames.AprilTag_Pipeline));
    }
}
