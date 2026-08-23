package org.firstinspires.ftc.buzzball.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import org.firstinspires.ftc.buzzball.Hardware;
import org.firstinspires.ftc.buzzball.Setup;

/**
 * Turret subsystem: a 1:1 (direct-drive) servo turret mounted off-center on the chassis, able to
 * track any field point (goal, etc.) using the robot's live Pedro pose, or be pointed at a fixed
 * relative angle. Since there's no slip ring, this subsystem tracks a continuous "unwrapped"
 * angle and only ever commands positions inside the turret's real mechanical range - it will
 * never ask the servo to spin through the gap where the wiring runs out.
 *
 * ================= GEOMETRY - READ BEFORE TRUSTING THE NUMBERS =================
 * All field/robot-pose math is done in INCHES and RADIANS to match this codebase's Pedro
 * Follower (see AutoConstants - existing offsets there are already in inches, and
 * follower.getPose() returns inches/radians). The mm measurements you gave were converted to
 * inches below.
 *
 * TURRET OFFSET FROM ROBOT CENTER (133mm): your sketch shows the forward arrow running roughly
 * through the chassis's centerline with the turret sitting on that same centerline but noticeably
 * toward the back half of the 14" square - i.e. the annotated "133mm" reads to me as the turret
 * sitting almost directly BEHIND the robot's center (along the negative forward axis), with
 * close to zero left/right offset. That's what's encoded below (TURRET_OFFSET_FORWARD_IN
 * negative, TURRET_OFFSET_LATERAL_IN zero). Hand-sketches are easy to misread on this kind of
 * thing - if the turret is actually offset to the SIDE instead of behind, just swap which
 * constant gets the -133mm and which gets 0. Everything downstream only cares about these two
 * numbers, nothing else needs to change.
 *
 * ODO POD OFFSETS (64.5mm each direction): these belong to the DRIVEBASE localizer
 * (AutoConstants.TwoWheelConfig.forwardPodYOffset / strafePodXOffset), not this subsystem - the
 * turret only ever asks the Follower for its pose and doesn't care how that pose was computed.
 * 64.5mm = 2.5394in if you want to drop it straight into those two fields; I didn't touch that
 * file since sign (which side each pod sits on) needs your confirmation, same as above.
 *
 * ================= SERVO MAPPING =================
 * Servo position 1.0 = turret facing the same direction as the front of the robot (0deg).
 * Decreasing position rotates the turret CLOCKWISE (viewed from above - flip
 * TURRET_CW_POSITIVE below if that's backwards) up to 355deg of total travel at position 0.0.
 * That leaves a 5deg dead zone the turret can physically never enter (that's where the wiring
 * runs out without a slip ring).
 */
@Configurable
public class TurretSubsystem implements Loggable, Subsystem {

    // ================= GEOMETRY CONSTANTS (INCHES) =================

    /** mm -> inches, just for readability below */
    private static final double MM_TO_IN = 1.0 / 25.4;

    /** turret's offset from the robot's center along the robot's FORWARD axis (inches, +forward) */
    public static double TURRET_OFFSET_FORWARD_IN = -133.0 * MM_TO_IN; // ASSUMPTION - see class doc
    /** turret's offset from the robot's center along the robot's LATERAL axis (inches, +left) */
    public static double TURRET_OFFSET_LATERAL_IN = 0.0; // ASSUMPTION - see class doc

    // ================= SERVO / MECHANICAL CONSTANTS =================

    /** servo position when the turret faces dead ahead (matches the front of the robot) */
    public static double TURRET_FORWARD_SERVO_POS = 1.0;
    /** total mechanical travel of the turret, in degrees (355 real degrees + a 5deg dead zone) */
    public static double TURRET_TOTAL_TRAVEL_DEG = 350.0;
    /** true if decreasing servo position rotates the turret CLOCKWISE viewed from above */
    public static boolean TURRET_CW_POSITIVE = true;

    /** how close (servo units) is "close enough" to call the turret on-target */
    public static double TURRET_POSITION_TOLERANCE = 0.005;

    // ================= EASILY-CHANGEABLE FIELD POSITIONS =================
    // Tune these in one place - everything else (commands, telemetry, etc.) just refers to them.

    /** the field point the turret tracks by default - point this at your goal */
    public static Pose TRACK_TARGET_DEFAULT = new Pose(135, 142, 0);

    /** known field positions you can snap the robot's pose to (e.g. lined up against a wall) */
    public static Pose RELOCALIZE_POSITION_1 = new Pose(137, 7, 0);
    public static Pose RELOCALIZE_POSITION_2 = new Pose(0, 0, 0);

    // ================= TELEMETRY =================

    @Log.Number(name = "turretServoPos")
    public double turretServoPos;

    /** continuous ("unwrapped") commanded angle, CW-positive degrees from forward */
    @Log.Number(name = "turretAngleCWDeg")
    public double turretAngleCWDeg;

    @Log.Number(name = "trackTargetX")
    public double trackTargetX;

    @Log.Number(name = "trackTargetY")
    public double trackTargetY;

    @Log.Number(name = "tracking")
    public boolean tracking = false;

    @Log.Number(name = "targetInDeadZone")
    public boolean targetInDeadZone = false;

    // ================= HARDWARE / STATE =================

    boolean hasHardware;
    Servo turretServo;
    Follower follower;
    Pose trackTarget = TRACK_TARGET_DEFAULT;

    public TurretSubsystem(Hardware h, Follower follower) {
        hasHardware = Setup.Connected.TURRETSUBSYSTEM;
        this.follower = follower;
        if (hasHardware) {
            turretServo = h.turretServo;
            // start pointed forward - a safe, known position
            turretAngleCWDeg = 0;
            setServoPositionRaw(TURRET_FORWARD_SERVO_POS);
            CommandScheduler.register(this);
        } else {
            turretServo = null;
        }
    }

    // ================= ANGLE <-> SERVO CONVERSION =================

    public double angleCWDegToServoPos(double angleCWDeg) {
        double normalized = angleCWDeg / TURRET_TOTAL_TRAVEL_DEG; // 0 at forward, 1 at full travel
        return TURRET_CW_POSITIVE
            ? TURRET_FORWARD_SERVO_POS - normalized
            : TURRET_FORWARD_SERVO_POS + normalized;
    }

    public double servoPosToAngleCWDeg(double servoPos) {
        double normalized = TURRET_CW_POSITIVE
            ? TURRET_FORWARD_SERVO_POS - servoPos
            : servoPos - TURRET_FORWARD_SERVO_POS;
        return normalized * TURRET_TOTAL_TRAVEL_DEG;
    }

    // ================= ANGLE MATH HELPERS =================

    /** wraps radians into (-pi, pi] */
    private double normalizeRadians(double rad) {
        double twoPi = 2 * Math.PI;
        double wrapped = ((((rad + Math.PI) % twoPi) + twoPi) % twoPi) - Math.PI;
        return wrapped;
    }

    /** wraps degrees into [0, 360) */
    private double normalizeDegrees0to360(double deg) {
        double wrapped = deg % 360.0;
        return wrapped < 0 ? wrapped + 360.0 : wrapped;
    }

    /**
     * WRAPAROUND RESOLUTION - the core "no slip ring" logic.
     *
     * rawAngleCWDeg is some angle in [0, 360) (the turret's raw CW-from-forward bearing to a
     * target, with no notion of the turret's mechanical limits). Because the turret physically
     * only covers [0, TURRET_TOTAL_TRAVEL_DEG] with a small dead zone the rest of the way around,
     * a raw bearing near that dead zone might only be reachable by first checking whether adding
     * or subtracting a full 360 lands it in-range, and - among any candidates that ARE reachable
     * - picking whichever is CLOSEST to where the turret currently is, so it never takes an
     * unnecessarily long way around when a short move is available.
     *
     * If the raw target angle falls entirely inside the dead zone (no +-360 shift makes it
     * reachable), this clamps to whichever mechanical limit (0 or TURRET_TOTAL_TRAVEL_DEG) is
     * closest to the turret's current commanded angle, and flags targetInDeadZone so you know
     * the turret is doing its best but can't point exactly at that bearing.
     */
    public double resolveWraparound(double rawAngleCWDeg, double currentAngleCWDeg) {
        double best = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (int k = -1; k <= 1; k++) {
            double candidate = rawAngleCWDeg + 360.0 * k;
            if (candidate >= 0 && candidate <= TURRET_TOTAL_TRAVEL_DEG) {
                double dist = Math.abs(candidate - currentAngleCWDeg);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = candidate;
                }
            }
        }

        if (Double.isNaN(best)) {
            // Target genuinely lives in the dead zone - get as close as the hardware allows.
            targetInDeadZone = true;
            double distToZero = Math.abs(0 - currentAngleCWDeg);
            double distToMax = Math.abs(TURRET_TOTAL_TRAVEL_DEG - currentAngleCWDeg);
            return distToZero <= distToMax ? 0 : TURRET_TOTAL_TRAVEL_DEG;
        }

        targetInDeadZone = false;
        return best;
    }

    /**
     * Given a field point, returns the robot-relative bearing to it (CCW-positive radians, 0 =
     * dead ahead), computed from the TURRET's actual field position (not the robot's declared
     * center), using the live Follower pose.
     */
    public double robotRelativeBearingRadTo(Pose target) {
        if (follower == null) return 0;
        Pose robotPose = follower.getPose();
        double heading = robotPose.getHeading();

        double turretFieldX =
            robotPose.getX() +
            TURRET_OFFSET_FORWARD_IN * Math.cos(heading) -
            TURRET_OFFSET_LATERAL_IN * Math.sin(heading);
        double turretFieldY =
            robotPose.getY() +
            TURRET_OFFSET_FORWARD_IN * Math.sin(heading) +
            TURRET_OFFSET_LATERAL_IN * Math.cos(heading);

        double bearingFieldRad = Math.atan2(
            target.getY() - turretFieldY,
            target.getX() - turretFieldX
        );
        return normalizeRadians(bearingFieldRad - heading);
    }

    /**
     * Full pipeline: field point -> robot-relative bearing -> turret's CW convention -> wraparound
     * resolution against the turret's CURRENT commanded angle -> servo position. Does NOT command
     * the servo; use trackFieldPointNow() or the continuous tracking mode for that.
     */
    public double computeServoPositionForFieldPoint(Pose target) {
        double robotRelativeCCWDeg = Math.toDegrees(robotRelativeBearingRadTo(target));
        double rawAngleCWDeg = normalizeDegrees0to360(-robotRelativeCCWDeg);
        double resolvedAngleCWDeg = resolveWraparound(rawAngleCWDeg, turretAngleCWDeg);
        return angleCWDegToServoPos(resolvedAngleCWDeg);
    }

    // ================= LOW-LEVEL SERVO CONTROL =================

    private void setServoPositionRaw(double pos) {
        double clamped = Math.max(0.0, Math.min(1.0, pos));
        if (hasHardware) {
            turretServo.setPosition(clamped);
        }
        turretServoPos = clamped;
        turretAngleCWDeg = servoPosToAngleCWDeg(clamped);
    }

    /** Manual control - disables tracking until re-enabled. */
    public void setServoPosition(double pos) {
        tracking = false;
        setServoPositionRaw(pos);
    }

    /** Manual control by angle (CW-positive degrees from forward) - disables tracking. */
    public void setTurretAngleCW(double angleCWDeg) {
        tracking = false;
        double resolved = resolveWraparound(normalizeDegrees0to360(angleCWDeg), turretAngleCWDeg);
        setServoPositionRaw(angleCWDegToServoPos(resolved));
    }

    public void turretToForward() {
        setTurretAngleCW(0);
    }

    public boolean isAtTarget(double targetServoPos) {
        return Math.abs(turretServoPos - targetServoPos) <= TURRET_POSITION_TOLERANCE;
    }

    // ================= TRACKING =================

    /** Changes what field point tracking mode aims at, without changing whether it's on/off. */
    public void setTrackTarget(Pose target) {
        trackTarget = target;
    }

    public void enableTracking() {
        tracking = true;
    }

    public void disableTracking() {
        tracking = false;
    }

    /** Convenience: set the target AND start tracking it in one call. */
    public void trackTarget(Pose target) {
        setTrackTarget(target);
        enableTracking();
    }

    public void trackDefaultTarget() {
        trackTarget(TRACK_TARGET_DEFAULT);
    }

    /** One-shot aim at a field point right now, without entering continuous tracking mode. */
    public void trackFieldPointNow(Pose target) {
        tracking = false;
        setServoPositionRaw(computeServoPositionForFieldPoint(target));
    }

    // ================= RELOCALIZE =================

    /** Snaps the robot's Pedro pose to a known field position (e.g. lined up against a wall). */
    public void relocalizeTo(Pose pose) {
        if (follower != null) {
            follower.setStartingPose(pose);
        }
    }

    public void relocalizeToPosition1() {
        relocalizeTo(RELOCALIZE_POSITION_1);
    }

    public void relocalizeToPosition2() {
        relocalizeTo(RELOCALIZE_POSITION_2);
    }

    @Override
    public void periodic() {
        if (!hasHardware) return;

        trackTargetX = trackTarget.getX();
        trackTargetY = trackTarget.getY();

        if (tracking && follower != null) {
            setServoPositionRaw(computeServoPositionForFieldPoint(trackTarget));
        }
    }
}
