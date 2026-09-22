package org.firstinspires.ftc.teamcode.buzzball.subsystems;

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
import org.firstinspires.ftc.teamcode.buzzball.Hardware;
import org.firstinspires.ftc.teamcode.buzzball.Setup;

@Configurable
public class DepositSubsystem implements Loggable, Subsystem {

    public static double TICK_TO_METERS = 0.000870769; // ratio of encoder ticks to total extension of the slides in meters this is not the final height to get target extension distance in meters take ur specific target position below and then subtract the retracted length from that
    public static double UP_POSITION = 0.973; // target total height of slides in meters when fully extended
    public static double MID_POSITION = 0.784; // target total height of slides in meters when 2/3 extended pos
    public static double LOW_POSITION = 0.595; // target total height of slides in meters when set to 1/3 extended pos
    public static double ZERO_POSITION = 0.406; // target total height of slides in meters when it is fully down
    public static double RETRACTED_LENGTH = 0.406; // length of slides when fully retracted in meters
    public static double CLAW_OPEN = 0.00; // position for the claw being open
    public static double CLAW_CLOSED = 0.22; // position the claw is at when it is grabbing something
    public static double WRIST_HORIZ_DEPO = 1.0; // the position the wrist is at when depositing
    public static double WRIST_PARALLEL = 1.0; //position the wrist servo is when the wrist is parallel to the arm this should be a function that changes with the arm position should be 1:1 this should be 1 when arm servo pos is at horiz depo pos
    public static double ARM_DOWN = 1.0; // position of arm servo when down as far as it can go ~25deg from being straight vertical
    public static double ARM_VERT = 0.5; // position of arm servo when facing straight upwards
    public static double ARM_HORIZ_DEPO = 0.23; // position of arm servo when facing horizontally straight backwards (we want to deposit backwards so this is good) we also dont want to go any further then this point
    public static double HIGH_BASKET_HEIGHT = 1.12; //final height i want the claw to be at for high basket
    public static double LOW_BASKET_HEIGHT = 0.635; // final height i want the claw to be at for low basket in meters
    public static double DEPO_ARM_LENGTH = 0.195; // length of depo arm in meters needed for doing all the trig and stuff to calculate the height the claw is at
    public static double WRIST_ARM_RATIO = -1.0; // flip sign to -1.0 if wrist runs opposite the arm
    public static PIDFCoefficients SLIDE_PIDF = new PIDFCoefficients(0.01, -0.0000015, 0.00048, 0);
    public static double SLIDE_FEEDFORWARD_HIGH = 0.143; // feedforward (in motor power) for the second stage of extension - WILL be higher then LOW
    public static double SLIDE_FEEDFORWARD_LOW = 0.1; // feedforward (in motor power) for the first stage of extension - WILL be lower then HIGH
    private static double slideFeedforward = SLIDE_FEEDFORWARD_LOW;

    public static double SLIDE_POSITION_TOLERANCE_METERS = 0.004; // how close the position needs to be for the command to finish
    public static double SLIDE_MIN_POWER = -0.5; // lower bound on slide power to help prevent the slides from slamming downwards super hard

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

    @Log.Number(name = "targetBasketHeightM")
    public double targetBasketHeightMeters;

    @Log.Number(name = "autoCompensateArm")
    public boolean autoCompensateArm = false;

    boolean hasHardware;
    EncodedMotor<DcMotorEx> slideMotor;
    Servo armServo;
    Servo clawServo;
    Servo wristServo;
    PIDFController slidePIDFController;

    public DepositSubsystem(Hardware h) {
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

            CommandScheduler.register(this);
        } else {
            slideMotor = null;
            armServo = null;
            clawServo = null;
            wristServo = null;
        }
    }

    public double heightToTicks(double targetTotalHeightMeters) {
        double extensionMeters = targetTotalHeightMeters - RETRACTED_LENGTH;
        return extensionMeters / TICK_TO_METERS;
    }

    public double ticksToHeight(double ticks) {
        return RETRACTED_LENGTH + ticks * TICK_TO_METERS;
    }

    public void setSlideTargetHeight(double targetTotalHeightMeters) {
        if (!hasHardware) return;
        targetSlideHeightMeters = targetTotalHeightMeters;
        double targetTicks = heightToTicks(targetTotalHeightMeters);
        slideFeedforward =
            slideMotor.getSensorValue() > 325 ? SLIDE_FEEDFORWARD_HIGH : SLIDE_FEEDFORWARD_LOW;
        slidePIDFController.setTarget(targetTicks);
    }

    public double getSlideCurrentHeightMeters() {
        return hasHardware ? ticksToHeight(slideMotor.getSensorValue()) : ZERO_POSITION;
    }

    public void slidesToZero() {
        setSlideTargetHeight(ZERO_POSITION);
    }

    public void slidesToLow() {
        setSlideTargetHeight(LOW_POSITION);
    }

    public void slidesToUp() {
        setSlideTargetHeight(UP_POSITION);
    }

    public void targetToHighBasket() {
        targetBasketHeightMeters = UP_POSITION;
    }

    public void targetToLowBasket() {
        targetBasketHeightMeters = LOW_POSITION;
    }

    // ik the theory math is just hard ;( so this is the one bit ripped straight from claude i can do it if i had to pretty simple trig plus it gives a nice explanation for how it works
    // ================= ARM ANGLE <-> HEIGHT TRIG =================

    /**
     * Calibrated ONLY between ARM_VERT (0deg) and ARM_HORIZ_DEPO (90deg) - the arc actually used
     * for basket scoring. Returns degrees from vertical.
     */
    public double servoPosToAngleDeg(double servoPos) {
        return (90.0 * (ARM_VERT - servoPos)) / (ARM_VERT - ARM_HORIZ_DEPO);
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

    public void armCompensationOn() {
        autoCompensateArm = true;
    }

    public void armCompensationOff() {
        autoCompensateArm = false;
    }

    public void armToVertical() {
        armServo.setPosition(ARM_VERT);
        armTargetPos = ARM_VERT;
    }

    public void armToDown() {
        armServo.setPosition(ARM_DOWN);
        armTargetPos = ARM_DOWN;
    }

    public void armToHorizDepo() {
        armServo.setPosition(ARM_HORIZ_DEPO);
        armTargetPos = ARM_HORIZ_DEPO;
    }

    public double wristParallelPosition(double armPos) {
        return WRIST_PARALLEL + WRIST_ARM_RATIO * (armPos - ARM_HORIZ_DEPO);
    }

    public void wristHoriz() {
        wristServo.setPosition(WRIST_HORIZ_DEPO);
    } // pro tip if you learn to read the english language it helps you understand code better!!!

    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN);
    } // hmmm i wonder what this does you have 1 guess!

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED);
    } // closes the claw mind blowing ik

    // sets slide target height to the high basket height and enables arm compensation
    // sets slide target height to the low basket height and enables arm compensation

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
            armTargetPos = compensatedArmPos;
            armServo.setPosition(compensatedArmPos);
        } else wristServo.setPosition(wristParallelPosition(armTargetPos));
    }
}
