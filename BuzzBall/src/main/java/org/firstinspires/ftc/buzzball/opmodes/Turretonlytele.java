package org.firstinspires.ftc.buzzball.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.control.CommandButton;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.buzzball.Hardware;
import org.firstinspires.ftc.buzzball.Robot;
import org.firstinspires.ftc.buzzball.Setup;
import org.firstinspires.ftc.buzzball.commands.TeleCommands;
import org.firstinspires.ftc.buzzball.helpers.StartingPosition;
import org.firstinspires.ftc.buzzball.subsystems.TurretSubsystem;

/**
 * Turret-only teleop for bench testing the TurretSubsystem in isolation.
 *
 * uponInit() forces every OTHER *mutable* Setup.Connected flag to false before Hardware/Robot
 * get built, so this opmode is turret-only regardless of whatever Setup.java currently has
 * configured for competition. NOTE: TESTSUBSYSTEM and LIMELIGHTSUBSYSTEM are declared `final`
 * in Setup.java, so they can't be flipped at runtime - comment those out in Setup.java directly
 * if you need them off too.
 *
 * DRIVEBASE is also forced off here, same as the rest - which means follower will be null and
 * the turret has no live pose to track with. Tracking mode will just sit there doing nothing in
 * that case. If you want to bench test actual field-point tracking (not just raw servo/angle
 * behavior), flip DRIVEBASE back on below and drive the bot around while watching the turret
 * track the target through the telemetry.
 *
 * Controls (driver gamepad):
 *   Triangle      - toggle: track TRACK_TARGET_DEFAULT / stop tracking and return to forward
 *   Circle        - relocalize robot pose to RELOCALIZE_POSITION_1
 *   Cross         - relocalize robot pose to RELOCALIZE_POSITION_2
 *   Right Bumper  - one-shot aim at TRACK_TARGET_DEFAULT right now (no continuous tracking)
 *   D-pad Up      - turret to forward (0deg) - quick known-good reset position
 *   D-pad Down    - turret to straight back (180deg) - range-of-motion sanity check
 *   D-pad Left    - jog turret angle -5deg from wherever it currently is
 *   D-pad Right   - jog turret angle +5deg from wherever it currently is
 *   PS/Options    - abort: stop tracking and return to forward
 */
@TeleOp(name = "TurretOnlyTele")
@SuppressWarnings("unused")
public class Turretonlytele extends CommandOpMode {

    public Robot robot;
    public Hardware hardware;
    private PanelsTelemetry panelsTelemetry;

    public CommandButton trackToggleButton;
    public CommandButton relocalize1Button;
    public CommandButton relocalize2Button;
    public CommandButton trackNowButton;
    public CommandButton toForwardButton;
    public CommandButton toBackButton;
    public CommandButton jogNegButton;
    public CommandButton jogPosButton;
    public CommandButton abortButton;

    private static final double TURRET_JOG_DEG = 5.0;

    @Override
    public void uponInit() {
        // Turret-only: force every other mutable subsystem flag off, no matter what Setup.java
        // currently has set for competition.
        Setup.Connected.DRIVEBASE = false;
        Setup.Connected.ODOSUBSYSTEM = true;
        Setup.Connected.SAFETYSUBSYSTEM = false;
        Setup.Connected.EXTERNAL_IMU = false;
        Setup.Connected.OTOS = false;
        Setup.Connected.INTAKESUBSYSTEM = false;
        Setup.Connected.SMARTINTAKE = false;
        Setup.Connected.LAUNCHERSUBSYSTEM = false;
        Setup.Connected.AIMINGSUBSYSTEM = false;
        Setup.Connected.BRAKESUBSYSTEM = false;
        Setup.Connected.DEPOSITSUBSYSTEM = false;
        Setup.Connected.TURRETSUBSYSTEM = true;
        panelsTelemetry = PanelsTelemetry.INSTANCE;

        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware, Alliance.NONE, StartingPosition.Unspecified);

        trackToggleButton = driverGamepad.ps_triangle;
        relocalize1Button = driverGamepad.ps_circle;
        relocalize2Button = driverGamepad.ps_cross;
        trackNowButton = driverGamepad.rightBumper;
        toForwardButton = driverGamepad.dpadUp;
        toBackButton = driverGamepad.dpadDown;
        jogNegButton = driverGamepad.dpadLeft;
        jogPosButton = driverGamepad.dpadRight;
        abortButton = driverGamepad.ps_options;

        trackToggleButton.toggle(
            TeleCommands.TurretTrackDefault(robot),
            TeleCommands.TurretDisableTracking(robot)
        );

        relocalize1Button.whenPressed(TeleCommands.TurretRelocalizeToPosition1(robot));
        relocalize2Button.whenPressed(TeleCommands.TurretRelocalizeToPosition2(robot));

        trackNowButton.whilePressedContinuous(() ->
            robot.turretSubsystem.trackFieldPointNow(TurretSubsystem.TRACK_TARGET_DEFAULT)
        );

        toForwardButton.whenPressed(robot.turretSubsystem::turretToForward);
        toBackButton.whenPressed(() -> robot.turretSubsystem.setTurretAngleCW(180));

        jogNegButton.whenPressed(() ->
            robot.turretSubsystem.setTurretAngleCW(
                robot.turretSubsystem.turretAngleCWDeg - TURRET_JOG_DEG
            )
        );
        jogPosButton.whenPressed(() ->
            robot.turretSubsystem.setTurretAngleCW(
                robot.turretSubsystem.turretAngleCWDeg + TURRET_JOG_DEG
            )
        );

        abortButton.whenPressed(TeleCommands.TurretDisableTracking(robot));
    }

    @Override
    public void runLoop() {
        panelsTelemetry
            .getTelemetry()
            .addData("servoPos", String.valueOf(robot.turretSubsystem.turretServoPos));
        panelsTelemetry
            .getTelemetry()
            .addData("angleCWDeg", String.valueOf(robot.turretSubsystem.turretAngleCWDeg));
        panelsTelemetry
            .getTelemetry()
            .addData("tracking", String.valueOf(robot.turretSubsystem.tracking));
        panelsTelemetry
            .getTelemetry()
            .addData("targetInDeadZone", String.valueOf(robot.turretSubsystem.targetInDeadZone));
        panelsTelemetry
            .getTelemetry()
            .addData("trackTargetX", String.valueOf(robot.turretSubsystem.trackTargetX));
        panelsTelemetry
            .getTelemetry()
            .addData("trackTargetY", String.valueOf(robot.turretSubsystem.trackTargetY));

        panelsTelemetry.getTelemetry().update(telemetry);
        if (robot.follower != null) {
            robot.follower.update();
        }
        if (robot.follower != null) {
            panelsTelemetry
                .getTelemetry()
                .addData("robotX", String.valueOf(robot.follower.getPose().getX()));
            panelsTelemetry
                .getTelemetry()
                .addData("robotY", String.valueOf(robot.follower.getPose().getY()));
            panelsTelemetry
                .getTelemetry()
                .addData(
                    "robotHeadingDeg",
                    String.valueOf(Math.toDegrees(robot.follower.getPose().getHeading()))
                );
        }
    }
}
