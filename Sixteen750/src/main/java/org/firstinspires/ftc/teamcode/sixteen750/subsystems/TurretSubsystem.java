package org.firstinspires.ftc.teamcode.sixteen750.subsystems;

import static java.lang.Math.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.util.MathUtils;
import com.technototes.library.util.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.sixteen750.Hardware;
import org.firstinspires.ftc.teamcode.sixteen750.Robot;
import org.firstinspires.ftc.teamcode.sixteen750.Setup;

@Configurable
public class TurretSubsystem implements Loggable, Subsystem {
    final Robot robot;
    Pose HIVE_TARGET1 = new Pose(58,53);
    Pose HIVE_TARGET2 = new Pose(58,88 );

    public static double TARGET_SWITCH_THRESHOLD = 71;
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.1;
    public static double TURRET_MAX = 0.9;

    public static double ROTATE_LEFT = 0.15;
    public static double ROTATE_RIGHT = 0.85;
    @Log.Number (name = "target servo Pos")
    public static double TARGET_SERVO_POS = 0;
    @Log.Number (name = "Robot Heading")
    public static double ROBOT_HEAD = 0;
    @Log.Number (name = "Absolute heading")
    public static double ABSOLUTE_HEAD = 0;
    @Log.Number (name = "Turret Heading")
    public static double TURRET_HEADING = 0;
    @Log.Number (name = "Target Velocity")
    public static double TARGET_VELO = 0;
    @Log.Number (name = "Current Velocity")
    public static double CURRENT_VELO = 0;
    @Log.Number (name = "Target Pose")
    public static Pose TARGET_POSE = new Pose(0,0);

    public static double UP = 0.1;
    public static double DOWN = 0.5;
    public static double INCREASE = 15;
    public static double ACTUAL_TARGET = 0;

    public static double LAUNCHER_VELOCITY = 2400;

    boolean hasHardware;
    Gamepad gamepad;

    EncodedMotor<DcMotorEx> launcher;
    Servo turret1, turret2, hood;
    // This the PIDF controller that's used manage the power.
    // The PIDF values are set in the Config class above.
    private final PIDFController pidfController;
    // The input value is the error of the target velocity that ranges from
    // -2800 to +2800 for a goBilda motor.
    // It's output is a power value in the -1 to +1 range.
    // So, P is probably in the range of .001-ish.
    // For a velocity-targeting PIDF, we probably want an I value, not a D value.
    public static PIDFCoefficients launchPID = new PIDFCoefficients(0.015, 0.0, 0.0, 0);
    private Hardware hardware;
    // Stuff used for the Feed Forward function.
    // This one is highly variable, based on the amount of friction in the system

    public static double kStaticFriction = 0.183;

    public static double kDynamicFriction = 0.168;

    // This one tends to be somewhere between 0.0035 to 0.005 or so.
    public static double kVelocityConstant = 0.0043;
    public static double MotorResistance = 12 / 9.2;

    public static double GetFrictionConstant(boolean inMotion) {
        return inMotion ? kDynamicFriction : kStaticFriction;
    }

    // GoBilda says stall current of 9.2A at 12V, so V = I * R, R = 12 / 9.2 (about 1.3 ohms)
    // As the motor heats up, resistance also increase, so we could increase this a little bit
    // or maybe increase it over time to counteract that, but this is probably good enough.

    public TurretSubsystem(Hardware h, Robot r) {
        hasHardware = Setup.Connected.TURRETSUBSYSTEM;
        robot = r;
        // Do stuff in here
        if (hasHardware) {
            hardware = h;
            launcher = h.launcher;
            turret1 = h.turret1;
            turret2 = h.turret2;
            hood = h.hood;
            CommandScheduler.register(this);
            gamepad = null;
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            launcher = null;
            turret1 = null;
            turret2 = null;
        }
        pidfController = new PIDFController(
            launchPID,
            target ->
                (Math.signum(target) * // signum(<0) = -1, signum(>0) = 1, signum(0) = *0*
                    (GetFrictionConstant(getActualVelocity() != 0) +
                        getMotor1Current() * MotorResistance) +
                    kVelocityConstant * target) /
                hardware.voltage()
        );
    }

    public void Launch() {
        // Spin the motors
        setVelocityTarget(LAUNCHER_VELOCITY);
    }



    // takes the already decided upon target and takes the current robot pose and does some math to figure out what angle the turret needs to point to face the target i just guessed which direction is positive should be easy to flip
    public double getTurretAngle() {
        Pose targetPose = getTargetPose();

        double x, y, robotHead, dx, dy, absoluteAngle, angle;

        x = robot.follower.pose().x();
        y = robot.follower.pose().y();
        robotHead = MathUtils.normalizeDeltaAngle(robot.follower.pose().heading(), AngleUnit.RADIANS); // head is in radians cause math is nice

        dx = targetPose.x() - x;
        dy = targetPose.y() - y;
        absoluteAngle = Math.atan2(dy, dx); // in radians the target angle of the turret relative to the field

        angle = (absoluteAngle - robotHead); // still in radians.......
        return angle;
    }
    //determines the distance from the robot to the current hive target in inches to be used for hood angle and flywheel speed
    public double getDistance() {
        Pose RobotPose = robot.follower.pose();
        Pose TargetPose = getTargetPose();

        double Distance = RobotPose.distance(TargetPose);

        return Distance;
    }
    public double increase() {
        ACTUAL_TARGET = LAUNCHER_VELOCITY +INCREASE;
        return ACTUAL_TARGET;
    }
    public double decrease() {
        ACTUAL_TARGET = LAUNCHER_VELOCITY - INCREASE;
        return ACTUAL_TARGET;
    }
    // determines which hive target to aim for based on what half of the field we are
    public Pose getTargetPose() {

        double Y;
        Pose  targetPose;

        Y = robot.follower.pose().y();

        if (Y > TARGET_SWITCH_THRESHOLD) {
            targetPose = HIVE_TARGET1;
        } else {
            targetPose = HIVE_TARGET2;
        }

        return targetPose;
    }
    // takes our turret target angle and turns it into a servo position also clamps it currently to not fry to wiring
    public double getTurretPos() {
        double angle = getTurretAngle();
        double servoPos = TURRET_CENTER - (angle / (2 * Math.PI));

        return (clamp(servoPos, TURRET_MIN, TURRET_MAX)-1  ) *-1;
    }
    public void setTurretTarget() {
        setTurretPosition(getTurretPos());
    }
    public void up() {
        LAUNCHER_VELOCITY = increase();
    }
    public void down() {
        LAUNCHER_VELOCITY = decrease();
    }


    @Override
    public void periodic() {
        getTurretPos();
        TARGET_SERVO_POS = getTurretPos();
        TARGET_POSE = getTargetPose();
        TURRET_HEADING = getTurretAngle();
        ROBOT_HEAD = robot.follower.pose().heading();
        TARGET_VELO = getVelocityTarget();
        CURRENT_VELO = getActualVelocity();


        // Add an item to the array and update the index for the next update to the 'circular' array
        double power = pidfController.update(getActualVelocity());
        setLauncherPower(power);
    }

    public double getMotor1Current() {
        return hasHardware ? launcher.getAmperage(CurrentUnit.AMPS) : 0;
    }

    public double getActualVelocity() {
        if (hasHardware) {
            return launcher.getVelocity();
        } else {
            return pidfController.getTarget(); // Not a Number
        }
    }

    // Explicitly set the target velocity for the motors
    public void setVelocityTarget(double speed) {
        pidfController.setTarget(speed);
    }
    public void setTurretPosition(double pos) {
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }

    // Returns the current target velocity (which may be set explicitly, or automatically)
    public double getVelocityTarget() {
        return pidfController.getTarget();
    }

    private void setLauncherPower(double power) {
        if (hasHardware) {
            launcher.setPower(power);
        }
    }
}
