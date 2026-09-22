package org.firstinspires.ftc.teamcode.buzzball.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.control.CommandButton;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.teamcode.buzzball.Hardware;
import org.firstinspires.ftc.teamcode.buzzball.Robot;
import org.firstinspires.ftc.teamcode.buzzball.Setup;
import org.firstinspires.ftc.teamcode.buzzball.commands.TeleCommands;
import org.firstinspires.ftc.teamcode.buzzball.helpers.StartingPosition;

/**
 * Deposit-only teleop for bench testing the DepositSubsystem in isolation.
 *
 * uponInit() forces every OTHER *mutable* Setup.Connected flag to false before Hardware/Robot
 * get built, so this opmode is deposit-only regardless of whatever Setup.java currently has
 * configured for competition. NOTE: TESTSUBSYSTEM and LIMELIGHTSUBSYSTEM are declared `final`
 * in Setup.java, so they can't be flipped at runtime - comment those out in Setup.java directly
 * if you need them off too.
 *
 * Controls (driver gamepad):
 *   Triangle      - go to high basket (live compensation: arm/wrist track the real slide height)
 *   Circle        - go to low basket (live compensation)
 *   Cross         - stow (slides down, arm down, claw closed)
 *   Right Bumper  - claw open while held, closes on release
 *   D-pad Up      - bench test: high basket, arm/wrist compensate off the TARGET height only
 *   D-pad Down    - bench test: low basket, arm/wrist compensate off the TARGET height only
 *   D-pad Left    - toggle the slide motor's output on/off (servo-only bench testing)
 *   D-pad Right   - dump (tips the wrist to WRIST_HORIZ_DEPO)
 *   PS/Options    - zero the slide encoder wherever the slides currently physically sit
 */
@TeleOp(name = "DepoOnlyTele")
@SuppressWarnings("unused")
public class Depoonlytele extends CommandOpMode {

    public Robot robot;
    public Hardware hardware;
    private PanelsTelemetry panelsTelemetry;

    public CommandButton highBasketButton;
    public CommandButton lowBasketButton;
    public CommandButton stowButton;
    public CommandButton clawOpenButton;
    public CommandButton testHighFromTargetButton;
    public CommandButton testLowFromTargetButton;
    public CommandButton toggleSlideOutputButton;
    public CommandButton dumpButton;
    public CommandButton resetEncoderButton;

    private boolean slideOutputOn = false;

    @Override
    public void uponInit() {
        // Deposit-only: force every other mutable subsystem flag off, no matter what Setup.java
        // currently has set for competition.
        Setup.Connected.DRIVEBASE = false;
        Setup.Connected.ODOSUBSYSTEM = false;
        Setup.Connected.SAFETYSUBSYSTEM = false;
        Setup.Connected.EXTERNAL_IMU = false;
        Setup.Connected.OTOS = false;
        Setup.Connected.INTAKESUBSYSTEM = false;
        Setup.Connected.SMARTINTAKE = false;
        Setup.Connected.LAUNCHERSUBSYSTEM = false;
        Setup.Connected.AIMINGSUBSYSTEM = false;
        Setup.Connected.BRAKESUBSYSTEM = false;
        Setup.Connected.DEPOSITSUBSYSTEM = true;
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware, Alliance.NONE, StartingPosition.Unspecified);
        lowBasketButton = driverGamepad.ps_circle;
        stowButton = driverGamepad.ps_cross;
        clawOpenButton = driverGamepad.rightBumper;
        testHighFromTargetButton = driverGamepad.dpadUp;
        testLowFromTargetButton = driverGamepad.dpadDown;
        toggleSlideOutputButton = driverGamepad.dpadLeft;
        dumpButton = driverGamepad.dpadRight;
        resetEncoderButton = driverGamepad.ps_options;

        //lowBasketButton.whenPressed(TeleCommands.DepositLowBasket(robot));
        // stowButton.whenPressed(TeleCommands.DepositSto(robot));

        clawOpenButton.whilePressed(TeleCommands.DepositClawOpen(robot));
        clawOpenButton.whenReleased(TeleCommands.DepositClawClose(robot));

        dumpButton.whenPressed(robot.claudeSubsystem::dump);
        resetEncoderButton.whenPressed(robot.claudeSubsystem::resetSlideEncoder);
    }

    @Override
    public void runLoop() {
        panelsTelemetry
            .getTelemetry()
            .addData("heightM", String.valueOf(robot.claudeSubsystem.slideHeightMeters));
        panelsTelemetry
            .getTelemetry()
            .addData(
                "targetHeightM",
                String.valueOf(robot.claudeSubsystem.targetSlideHeightMeters)
            );
        panelsTelemetry
            .getTelemetry()
            .addData("ticks", String.valueOf(robot.claudeSubsystem.slideTicks));
        panelsTelemetry
            .getTelemetry()
            .addData("power", String.valueOf(robot.claudeSubsystem.slidePower));
        panelsTelemetry
            .getTelemetry()
            .addData("armPos", String.valueOf(robot.claudeSubsystem.armTargetPos));
        panelsTelemetry
            .getTelemetry()
            .addData("armAngleDeg", String.valueOf(robot.claudeSubsystem.armAngleDeg));
        panelsTelemetry
            .getTelemetry()
            .addData("wristPos", String.valueOf(robot.claudeSubsystem.wristTargetPos));
        panelsTelemetry
            .getTelemetry()
            .addData("clawPos", String.valueOf(robot.claudeSubsystem.clawTargetPos));
        panelsTelemetry
            .getTelemetry()
            .addData(
                "targetBasketHeightM",
                String.valueOf(robot.claudeSubsystem.targetBasketHeightMeters)
            );

        panelsTelemetry.getTelemetry().update(telemetry);
    }
}
