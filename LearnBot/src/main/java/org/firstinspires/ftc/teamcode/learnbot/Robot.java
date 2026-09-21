package org.firstinspires.ftc.teamcode.learnbot;

import com.bylazar.configurables.annotations.Configurable;
import com.technototes.library.logger.Loggable;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.teamcode.learnbot.components.Gimbal;
import org.firstinspires.ftc.teamcode.learnbot.components.PedroDrivebase;
import org.firstinspires.ftc.teamcode.learnbot.helpers.StartingPosition;
import org.firstinspires.ftc.teamcode.learnbot.subsystems.TargetSubsystem;

@Configurable
public class Robot implements Loggable {

    public Alliance alliance;
    public StartingPosition position;

    public double initialVoltage;
    // Subsystems:
    // (Currently, Mouse only has a single subsystem: The drivebase)
    public PedroDrivebase.Component drivebase;
    public Gimbal.Component gimbal;
    public TargetSubsystem vision;

    public Robot(Hardware hw, Alliance team, StartingPosition pos) {
        this.position = pos;
        this.alliance = team;
        this.initialVoltage = hw.voltage();
        if (Setup.Connected.LIMELIGHT) {
            this.vision = new TargetSubsystem(hw.limelight);
        } else {
            this.vision = null;
        }
        if (Setup.Connected.DRIVEBASE) {
            // Note that vision may be null, but the drivebase is okay with this.
            this.drivebase = new PedroDrivebase.Component(hw.follower, vision, team);
        }
        if (Setup.Connected.GIMBAL) {
            this.gimbal = new Gimbal.Component(hw.yawServo, hw.pitchServo, vision);
        }
    }

    public void atStart() {}
}
