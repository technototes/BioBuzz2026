package org.firstinspires.ftc.teamcode.twenty403.subsystems;

import static org.firstinspires.ftc.teamcode.twenty403.subsystems.LauncherSubsystem.TARGET_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.twenty403.subsystems.LauncherSubsystem.err;

import com.bylazar.configurables.annotations.Configurable;
import com.technototes.library.hardware.motor.CRServo;
import com.technototes.library.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.twenty403.Hardware;
import org.firstinspires.ftc.teamcode.twenty403.Setup;

@Configurable
public class FeedingSubsystem implements Subsystem {

    public static double CRSERVO_SPEED = 1; // 0.15 0.25
    public static double CRSERVO_SPEED_SLOW = .7;
    private boolean REQUEST_TO_MOVE = false;

    boolean hasHardware;
    CRServo bottomLeft, bottomRight;
    Hardware h;

    public FeedingSubsystem(Hardware h) {
        hasHardware = Setup.Connected.LAUNCHER;
        // Do stuff in here
        if (hasHardware) {
            bottomLeft = h.leftIntakeServo;
            bottomRight = h.rightIntakeServo;
            this.h = h;
        } else {
            bottomLeft = null;
            bottomRight = null;
        }
    }

    public void moveball() {
        if (hasHardware) {
            // Doesn't work with configurable launcher velocities
            // Also hardcodes behavior in subsystem instead of letting command do that
            //            if (LauncherSubsystem.top.getVelocity() < TARGET_MOTOR_VELOCITY) {
            //                // Do not feed the ball if the launcher has not reached the
            //                // target velocity. We don't want bland throws.
            //                return;
            //            }
            if (
                (h.launcher.getVelocity() >= TARGET_MOTOR_VELOCITY - 20 &&
                    h.launcher.getVelocity() < TARGET_MOTOR_VELOCITY + 50) ||
                err < 80
            ) {
                bottomRight.setPower(-CRSERVO_SPEED);
                bottomLeft.setPower(CRSERVO_SPEED);
            }
        }
    }

    public void moveballanyways() {
        if (hasHardware) {
            bottomRight.setPower(-CRSERVO_SPEED);
            bottomLeft.setPower(CRSERVO_SPEED);
        }
    }

    public void moveballslow() {
        if (hasHardware) {
            bottomRight.setPower(-CRSERVO_SPEED_SLOW);
            bottomLeft.setPower(CRSERVO_SPEED_SLOW);
        }
    }

    public void stop() {
        if (hasHardware) {
            bottomLeft.setPower(0);
            bottomRight.setPower(0);
        }
    }
}
