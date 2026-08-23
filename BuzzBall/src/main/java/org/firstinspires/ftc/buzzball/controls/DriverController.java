package org.firstinspires.ftc.buzzball.controls;

import com.technototes.library.command.CommandScheduler;
import com.technototes.library.control.CommandAxis;
import com.technototes.library.control.CommandButton;
import com.technototes.library.control.CommandGamepad;
import com.technototes.library.control.Stick;
import com.technototes.library.logger.Loggable;
import org.firstinspires.ftc.buzzball.Hardware;
import org.firstinspires.ftc.buzzball.Robot;
import org.firstinspires.ftc.buzzball.Setup;
import org.firstinspires.ftc.buzzball.commands.PedroDriver;
import org.firstinspires.ftc.buzzball.commands.SequentialCommands;
import org.firstinspires.ftc.buzzball.commands.TeleCommands;
import org.firstinspires.ftc.buzzball.commands.driving.DrivingCommands;

public class DriverController implements Loggable {

    public Robot robot;
    public Hardware hardware;
    public CommandGamepad gamepad;

    public Stick driveLeftStick, driveRightStick;
    public CommandButton resetGyroButton;
    public CommandButton snailButton;
    public CommandButton launchButton;
    public CommandButton spitButton;
    public CommandButton gateButton;
    public CommandButton brakeButton;
    public CommandButton override;
    public CommandButton RelocButton;
    public CommandButton holdButton;
    public CommandAxis intakeTrigger;
    public CommandAxis autoAim;

    public PedroDriver pedroDriver;
    public CommandButton depositHighBasketButton;
    public CommandButton depositLowBasketButton;
    public CommandButton depositRetractButton;
    public CommandButton clawOpenButton;
    public CommandButton turretTrackToggleButton;
    public CommandButton turretRelocalizeButton;

    public static double triggerThreshold = 0.1;

    public DriverController(CommandGamepad g, Robot r) {
        this.robot = r;
        gamepad = g;
        override = g.leftTrigger.getAsButton(0.5);
        override = g.rightTrigger.getAsButton(0.5);

        AssignNamedControllerButton();
        if (Setup.Connected.DRIVEBASE) {
            bindDriveControls();
        }
        if (Setup.Connected.LAUNCHERSUBSYSTEM) {
            bindLaunchControls();
        }
        if (Setup.Connected.INTAKESUBSYSTEM) {
            bindIntakeControls();
        }
        if (Setup.Connected.BRAKESUBSYSTEM) {
            bindBrakeControls();
        }
        if (Setup.Connected.AIMINGSUBSYSTEM) {
            bindAimControls();
        }
        if (Setup.Connected.DEPOSITSUBSYSTEM) {
            bindDepositControls();
        }
        if (Setup.Connected.TURRETSUBSYSTEM) {
            bindTurretControls();
        }
    }

    public void AssignNamedControllerButton() {
        resetGyroButton = gamepad.ps_options;
        driveLeftStick = gamepad.leftStick;
        driveRightStick = gamepad.rightStick;
        intakeTrigger = gamepad.rightTrigger;
        autoAim = gamepad.leftTrigger;
        depositHighBasketButton = gamepad.ps_triangle;
        depositLowBasketButton = gamepad.ps_circle;
        depositRetractButton = gamepad.rightBumper;
        clawOpenButton = gamepad.leftBumper;
        snailButton = gamepad.leftBumper;
        launchButton = gamepad.rightBumper;
        spitButton = gamepad.ps_square;
        brakeButton = gamepad.ps_triangle;
        gateButton = gamepad.ps_cross;
        holdButton = gamepad.ps_circle;
        turretTrackToggleButton = gamepad.ps_share;
        turretRelocalizeButton = gamepad.dpadUp;
    }

    public void bindDriveControls() {
        pedroDriver = new PedroDriver(
            robot.follower,
            driveLeftStick,
            driveRightStick,
            robot.limelightSubsystem
        );
        CommandScheduler.scheduleJoystick(pedroDriver);

        snailButton.whenPressedReleased(
            DrivingCommands.SnailDriving(pedroDriver),
            DrivingCommands.NormalDriving(pedroDriver)
        );

        resetGyroButton.whenPressed(DrivingCommands.ResetGyro(pedroDriver));

        if (Setup.Connected.LIMELIGHTSUBSYSTEM) {
            autoAim.whenPressed(DrivingCommands.AutoOrient(pedroDriver));
            autoAim.whenReleased(DrivingCommands.NoAutoOrient(pedroDriver));
            RelocButton.whenPressed(TeleCommands.LLRelocCommand(robot));
        }
    }

    public void bindLaunchControls() {
        launchButton.whilePressed(TeleCommands.Launch(robot));
        launchButton.whileReleased(TeleCommands.IdleLaunch(robot));
    }

    public void bindIntakeControls() {
        spitButton.whenPressed(TeleCommands.Spit(robot));
        spitButton.whenReleased(TeleCommands.IntakeStop(robot));
        intakeTrigger.whilePressed(TeleCommands.Intake(robot));
        intakeTrigger.whenPressed(TeleCommands.GobbleGulp(robot));
        intakeTrigger.whenReleased(TeleCommands.IThinkIAteTooMuch(robot));
        intakeTrigger.whenReleased(TeleCommands.IntakeStop(robot));
    }

    public void bindBrakeControls() {
        brakeButton.whilePressed(TeleCommands.EngageBrake(robot));
        brakeButton.whilePressed(TeleCommands.StopLaunch(robot));
        brakeButton.whenReleased(TeleCommands.DisengageBrake(robot));
    }

    public void bindDepositControls() {
        depositHighBasketButton.whenPressed(SequentialCommands.HighBasketScore(robot));
        depositLowBasketButton.whenPressed(SequentialCommands.LowBasketScore(robot));
        depositRetractButton.whenPressed(SequentialCommands.PlaceAndRetract(robot));

        clawOpenButton.whilePressed(TeleCommands.DepositClawOpen(robot));
        clawOpenButton.whenReleased(TeleCommands.DepositClawClose(robot));
    }

    public void bindAimControls() {
        gateButton.whenPressed(TeleCommands.GateDown(robot));
        gateButton.whenPressed(TeleCommands.Feed(robot));
        gateButton.whenReleased(TeleCommands.IntakeStop(robot));
        gateButton.whenReleased(TeleCommands.GateUp(robot));

        //
        holdButton.whilePressed(TeleCommands.Intake(robot));
        holdButton.whenPressed(TeleCommands.GateDown(robot));
        holdButton.whenReleased(TeleCommands.IntakeStop(robot));
        holdButton.whenReleased(TeleCommands.GateUp(robot));
    }

    public void bindTurretControls() {
        turretTrackToggleButton.toggle(
            TeleCommands.TurretTrackDefault(robot),
            TeleCommands.TurretDisableTracking(robot)
        );

        turretRelocalizeButton.whenPressed(TeleCommands.TurretRelocalizeToPosition1(robot));
    }
}
