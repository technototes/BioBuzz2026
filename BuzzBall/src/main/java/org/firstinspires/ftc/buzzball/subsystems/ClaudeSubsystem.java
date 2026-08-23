package org.firstinspires.ftc.buzzball.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.util.PIDFController;
import org.firstinspires.ftc.buzzball.Hardware;
import org.firstinspires.ftc.buzzball.Setup;

/**
 * Deposit subsystem: vertical slides + a servo-pivoted arm + a claw + a coaxially-powered wrist.
 *
 * All linear units are METERS. All angular servo positions are the raw 0-1 servo range.
 *
 * ARM ANGLE COMPENSATION
 * -----------------------
 * The arm pivots on top of the vertical slides. Only ARM_VERT (pointing straight up) and
 * ARM_HORIZ_DEPO (pointing straight backward) are used for the height-compensation trig below,
 * since basket scoring only ever happens somewhere in that backward arc. ARM_DOWN lives outside
 * that arc (it's ~25deg off vertical in the *opposite*, forward direction - e.g. a transfer/stow
 * position) so it is intentionally NOT part of the angle<->height calibration.
 *
 * Treating the arm pivot as sitting at the current slide height, and treating "angle from
 * vertical" (theta) as 0deg at ARM_VERT and 90deg at ARM_HORIZ_DEPO:
 *
 *      clawHeight = slideHeight + DEPO_ARM_LENGTH * cos(theta)
 *
 * So to hit a desired claw/basket height from whatever height the slides currently happen to be
 * at, solve for theta:
 *
 *      theta = acos( (targetBasketHeight - currentSlideHeight) / DEPO_ARM_LENGTH )
 *
 * and convert theta back into a servo position. This is recomputed every loop (see periodic())
 * while auto-compensation is active, so the arm keeps correcting itself for wherever the slides
 * currently are, not just wherever they're eventually headed.
 *
 * WRIST
 * -----
 * WRIST_PARALLEL is defined as a 1:1 function of the arm's servo position (per spec: it should
 * equal WRIST_PARALLEL when the arm is at ARM_HORIZ_DEPO, and move 1:1 with the arm from there).
 * See wristParallelPosition(). WRIST_HORIZ_DEPO is the fixed "dump into the basket" position,
 * used only at the moment of actually depositing (see dump()).
 */
@Configurable
public class ClaudeSubsystem implements Loggable, Subsystem {

    // ================= CONSTANTS PROVIDED =================

    /** ratio of encoder ticks to total extension of the slides, in meters */
    public static double TICK_TO_METERS = 0.000870769;

    /** target total height of slides in meters when fully extended */
    public static double UP_POSITION = 0.973;
    /** target total height of slides in meters when set to 2/3 extended pos */
    public static double MID_POSITION = 0.62;
    /** target total height of slides in meters when set to 1/3 extended pos */
    public static double LOW_POSITION = 0.595;
    /** target total height of slides in meters when it is fully down */
    public static double ZERO_POSITION = 0.406;
    /** length of slides when fully retracted, in meters */
    public static double RETRACTED_LENGTH = 0.406;

    /** claw servo position when open */
    public static double CLAW_OPEN = 0.00;
    /** claw servo position when closed / grabbing something */
    public static double CLAW_CLOSED = 0.22;

    /** wrist position for dumping into the basket */
    public static double WRIST_HORIZ_DEPO = 1.0;
    /** wrist position when parallel to the arm, AT ARM_HORIZ_DEPO. See wristParallelPosition(). */
    public static double WRIST_PARALLEL = 1.0;

    /** arm servo position when down as far as it can go (~25deg from straight vertical) */
    public static double ARM_DOWN = 1.0;
    /** arm servo position when facing straight upward */
    public static double ARM_VERT = 0.46;
    /** arm servo position when facing horizontally straight backward (deposit direction, hard limit) */
    public static double ARM_HORIZ_DEPO = 0.21;

    /** final claw height target for the high basket, in meters */
    public static double HIGH_BASKET_HEIGHT = 1.12;
    /** final claw height target for the low basket, in meters */
    public static double LOW_BASKET_HEIGHT = 0.675;

    /** length of the depo arm, in meters, used for the height-compensation trig */
    public static double DEPO_ARM_LENGTH = 0.195;

    // ================= ADDED CONSTANTS (NOT PROVIDED - PLEASE TUNE) =================

    /** how many degrees the wrist moves per unit the arm servo moves (spec says 1:1) */
    public static double WRIST_ARM_RATIO = -1.0; // flip sign to -1.0 if wrist runs opposite the arm

    /** slide position PIDF - NEEDS TUNING, not provided */
    public static PIDFCoefficients SLIDE_PIDF = new PIDFCoefficients(0.01, -0.0000015, 0.00048, 0);
    /** feedforward (in motor power) to hold the slides up against gravity while extending - NEEDS TUNING */
    public static double SLIDE_FEEDFORWARD_HIGH = 0.143;
    /** feedforward (in motor power) while retracting - NEEDS TUNING, usually smaller than UP */
    public static double SLIDE_FEEDFORWARD_LOW = 0.1;
    private static double slideFeedforward = SLIDE_FEEDFORWARD_LOW;

    /** how close (in meters) the slides need to be to their target to be considered "there" */
    public static double SLIDE_POSITION_TOLERANCE_METERS = 0.004;
    public static double SLIDE_MIN_POWER = -0.5;

    // ================= TELEMETRY =================

    @Log.Number(name = "slideTicks")
    public double slideTicks;

    @Log.Number(name = "slideHeightM")
    public double slideHeightMeters;

    @Log.Number(name = "slideTargetHeightM")
    public double targetSlideHeightMeters;

    @Log.Number(name = "slidePower")
    public double slidePower;

    @Log.Number(name = "armTargetPos")
    public double armTargetPos;

    @Log.Number(name = "armAngleDeg")
    public double armAngleDeg;

    @Log.Number(name = "wristTargetPos")
    public double wristTargetPos;

    @Log.Number(name = "clawTargetPos")
    public double clawTargetPos;

    @Log.Number(name = "targetBasketHeightM")
    public double targetBasketHeightMeters;

    @Log.Number(name = "autoCompensateArm")
    public boolean autoCompensateArm = false;

    // ================= HARDWARE =================

    boolean hasHardware;
    EncodedMotor<DcMotorEx> slideMotor;
    Servo armServo;
    Servo clawServo;
    Servo wristServo;
    PIDFController slidePIDFController;

    public ClaudeSubsystem(Hardware h) {
        hasHardware = Setup.Connected.DEPOSITSUBSYSTEM;
        if (hasHardware) {
            slideMotor = h.depoSlideMotor;
            armServo = h.depoArmServo;
            clawServo = h.depoClawServo;
            wristServo = h.depoWristServo;

            slideMotor.setPIDFCoefficients(SLIDE_PIDF);
            slidePIDFController = new PIDFController(SLIDE_PIDF, target -> slideFeedforward);

            // start folded down / closed, matching a safe stowed state
            targetSlideHeightMeters = ZERO_POSITION;
            slidePIDFController.setTarget(0);
            setArmPositionRaw(ARM_DOWN);
            setWristPositionRaw(WRIST_PARALLEL);
            setClawPositionRaw(CLAW_CLOSED);

            CommandScheduler.register(this);
        } else {
            slideMotor = null;
            armServo = null;
            clawServo = null;
            wristServo = null;
        }
    }

    // ================= SLIDES =================

    /**
     * Converts a target TOTAL slide height (meters) into an extension distance (meters) by
     * subtracting the retracted length, then into encoder ticks.
     */
    public double heightToTicks(double targetTotalHeightMeters) {
        double extensionMeters = targetTotalHeightMeters - RETRACTED_LENGTH;
        return extensionMeters / TICK_TO_METERS;
    }

    public double ticksToHeight(double ticks) {
        return RETRACTED_LENGTH + (ticks * TICK_TO_METERS);
    }

    /** Commands the slides to a target TOTAL height in meters (e.g. UP_POSITION, MID_POSITION, ...). */
    public void setSlideTargetHeight(double targetTotalHeightMeters) {
        if (!hasHardware) return;
        targetSlideHeightMeters = targetTotalHeightMeters;
        double targetTicks = heightToTicks(targetTotalHeightMeters);
        slideFeedforward = (slideMotor.getSensorValue() > 325)
                ? SLIDE_FEEDFORWARD_HIGH
                : SLIDE_FEEDFORWARD_LOW;
        slidePIDFController.setTarget(targetTicks);
    }

    public double getSlideCurrentHeightMeters() {
        return hasHardware ? ticksToHeight(slideMotor.getSensorValue()) : ZERO_POSITION;
    }

    public boolean isSlideAtTarget() {
        return Math.abs(getSlideCurrentHeightMeters() - targetSlideHeightMeters) <=
                SLIDE_POSITION_TOLERANCE_METERS;
    }

    public void slidesToZero() {
        setSlideTargetHeight(ZERO_POSITION);
    }

    public void slidesToLow() {
        setSlideTargetHeight(LOW_POSITION);
    }

    public void slidesToMid() {
        setSlideTargetHeight(MID_POSITION);
    }

    public void slidesToUp() {
        setSlideTargetHeight(UP_POSITION);
    }

    public void resetSlideEncoder() {
        if (hasHardware) {
            slideMotor.tare();
        }
    }

    // ================= ARM ANGLE <-> HEIGHT TRIG =================

    /**
     * Calibrated ONLY between ARM_VERT (0deg) and ARM_HORIZ_DEPO (90deg) - the arc actually used
     * for basket scoring. Returns degrees from vertical.
     */
    public double servoPosToAngleDeg(double servoPos) {
        return 90.0 * (ARM_VERT - servoPos) / (ARM_VERT - ARM_HORIZ_DEPO);
    }

    public double angleDegToServoPos(double angleDeg) {
        return ARM_VERT - (angleDeg / 90.0) * (ARM_VERT - ARM_HORIZ_DEPO);
    }

    /**
     * Given the CURRENT slide height and a TARGET claw/basket height, solves for the arm servo
     * position that makes up the difference via clawHeight = slideHeight + L*cos(theta).
     * Clamped to the safe ARM_HORIZ_DEPO..ARM_VERT range - if the requested height is out of
     * reach (too high to need any tilt, or farther than the arm can add), it clamps to the
     * nearest reachable end rather than doing something undefined.
     */
    public double computeCompensatedArmPosition(
            double currentSlideHeightMeters,
            double targetBasketHeightMeters
    ) {
        double diff = targetBasketHeightMeters - currentSlideHeightMeters;
        double ratio = Math.max(-1.0, Math.min(1.0, diff / DEPO_ARM_LENGTH));
        double angleDeg = Math.toDegrees(Math.acos(ratio));
        angleDeg = Math.max(0.0, Math.min(90.0, angleDeg));

        double servoPos = angleDegToServoPos(angleDeg);
        double min = Math.min(ARM_HORIZ_DEPO, ARM_VERT);
        double max = Math.max(ARM_HORIZ_DEPO, ARM_VERT);
        return Math.max(min, Math.min(max, servoPos));
    }

    // ================= ARM =================

    private void setArmPositionRaw(double pos) {
        if (hasHardware) {
            armServo.setPosition(pos);
        }
        armTargetPos = pos;
        armAngleDeg = servoPosToAngleDeg(pos);
    }

    /** Manual arm control - disables auto height-compensation until re-enabled. */
    public void setArmPosition(double pos) {
        autoCompensateArm = false;
        setArmPositionRaw(pos);
    }

    public void armToVertical() {
        setArmPosition(ARM_VERT);
    }

    public void armToDown() {
        setArmPosition(ARM_DOWN);
    }

    public void armToHorizDepo() {
        setArmPosition(ARM_HORIZ_DEPO);
    }

    // ================= WRIST =================

    /**
     * Wrist position that keeps the wrist parallel to the arm, as a 1:1 function of the arm's
     * current servo position. Calibrated so it equals WRIST_PARALLEL exactly when the arm is at
     * ARM_HORIZ_DEPO, per spec.
     */
    public double wristParallelPosition(double currentArmPos) {
        return WRIST_PARALLEL + WRIST_ARM_RATIO * (currentArmPos - ARM_HORIZ_DEPO);
    }

    private void setWristPositionRaw(double pos) {
        if (hasHardware) {
            wristServo.setPosition(pos);
        }
        wristTargetPos = pos;
    }

    public void setWristParallelToArm() {
        setWristPositionRaw(wristParallelPosition(armTargetPos));
    }

    /** Tips the wrist over to dump into the basket. Call once the slides/arm are at target. */
    public void dump() {
        setWristPositionRaw(WRIST_HORIZ_DEPO);
    }

    // ================= CLAW =================

    private void setClawPositionRaw(double pos) {
        if (hasHardware) {
            clawServo.setPosition(pos);
        }
        clawTargetPos = pos;
    }

    public void openClaw() {
        setClawPositionRaw(CLAW_OPEN);
    }

    public void closeClaw() {
        setClawPositionRaw(CLAW_CLOSED);
    }

    // ================= HIGH-LEVEL SCORING =================

    /**
     * Sends the slides + arm toward the high basket. The arm continuously compensates (in
     * periodic()) for wherever the slides currently are relative to HIGH_BASKET_HEIGHT, and the
     * wrist tracks parallel to the arm the whole way. Call dump() once in position to score.
     */
    public void goToHighBasket() {
        setSlideTargetHeight(UP_POSITION);
        targetBasketHeightMeters = HIGH_BASKET_HEIGHT;
        autoCompensateArm = true;
        setWristPositionRaw(WRIST_HORIZ_DEPO);
    }

    /** Same as goToHighBasket() but for the low basket. */
    public void goToLowBasket() {
        setSlideTargetHeight(MID_POSITION);
        targetBasketHeightMeters = LOW_BASKET_HEIGHT;
        autoCompensateArm = true;
        setWristPositionRaw(WRIST_HORIZ_DEPO);
    }

    /** Stows the deposit mechanism: slides down, arm down, claw closed, auto-compensation off. */
    public void stow() {
        autoCompensateArm = false;
        slidesToZero();
        setArmPositionRaw(ARM_DOWN);
        closeClaw();
        setWristParallelToArm();
    }

    @Override
    public void periodic() {
        if (!hasHardware) return;

        slideTicks = slideMotor.getSensorValue();
        slideHeightMeters = ticksToHeight(slideTicks);
        slidePower = slidePIDFController.update(slideTicks);
        slidePower = Math.max(SLIDE_MIN_POWER, slidePower);
        slideMotor.setPower(slidePower);

        if (autoCompensateArm) {
            double compensatedArmPos = computeCompensatedArmPosition(
                    slideHeightMeters,
                    targetBasketHeightMeters
            );
            setArmPositionRaw(compensatedArmPos);
        } else setWristPositionRaw(wristParallelPosition(armTargetPos));
    }
}