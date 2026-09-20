package org.firstinspires.ftc.buzzball;

import com.pedropathing.follower.Follower;
import com.technototes.library.logger.Loggable;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.buzzball.helpers.StartingPosition;
import org.firstinspires.ftc.buzzball.subsystems.AimingSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.BrakeSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.ClaudeSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.DepositSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.SafetySubsystem;
import org.firstinspires.ftc.buzzball.subsystems.TestSubsystem;
import org.firstinspires.ftc.buzzball.subsystems.TurretSubsystem;

public class Robot implements Loggable {

    public StartingPosition position;
    public Alliance alliance;

    public double initialVoltage;

    public SafetySubsystem safetySubsystem;
    public LauncherSubsystem launcherSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public BrakeSubsystem brakeSubsystem;
    public AimingSubsystem aimingSubsystem;
    public LimelightSubsystem limelightSubsystem;
    public ClaudeSubsystem claudeSubsystem;
    public DepositSubsystem depositSubsystem;
    public TurretSubsystem turretSubsystem;
    public TestSubsystem testSubsystem;
    public Follower follower;
    private Hardware hardware;

    public Robot(Hardware hw, Alliance team, StartingPosition pos) {
        this.position = pos;
        this.alliance = team;
        this.hardware = hw;
        this.initialVoltage = hw.voltage();

        if (Setup.Connected.SAFETYSUBSYSTEM) {
            this.safetySubsystem = new SafetySubsystem(hw);
        }
        if (Setup.Connected.INTAKESUBSYSTEM) {
            this.intakeSubsystem = new IntakeSubsystem(hw);
        }
        if (Setup.Connected.LAUNCHERSUBSYSTEM) {
            this.launcherSubsystem = new LauncherSubsystem(hw, team);
        }
        if (Setup.Connected.BRAKESUBSYSTEM) {
            this.brakeSubsystem = new BrakeSubsystem(hw);
        }
        if (Setup.Connected.LIMELIGHTSUBSYSTEM) {
            this.limelightSubsystem = new LimelightSubsystem(hw, team);
        }
        if (Setup.Connected.AIMINGSUBSYSTEM) {
            this.aimingSubsystem = new AimingSubsystem(hw, limelightSubsystem);
        }
        if (Setup.Connected.TESTSUBSYSTEM) {
            this.testSubsystem = new TestSubsystem(hw);
        }
        if (Setup.Connected.DRIVEBASE) {
            follower = AutoConstants.createFollower(hw.map);
        }
        if (Setup.Connected.ODOSUBSYSTEM) {
            follower = AutoConstants.createFollower(hw.map);
        }
        if (Setup.Connected.CLAUDESUBSYSTEM) {
            this.claudeSubsystem = new ClaudeSubsystem(hw);
        }
        if (Setup.Connected.DEPOSITSUBSYSTEM) {
            this.depositSubsystem = new DepositSubsystem(hw);
        }
        if (Setup.Connected.TURRETSUBSYSTEM) {
            // built after follower above - the turret needs it for field-relative tracking
            this.turretSubsystem = new TurretSubsystem(hw, follower);
        }
    }

    public Hardware getHardware() {
        return hardware;
    }

    public Follower getFollower() {
        return follower;
    }

    public void prepForStart() {}
}
