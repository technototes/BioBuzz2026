package org.firstinspires.ftc.teamcode.sixteen750.subsystems;

import static java.lang.Math.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.logger.Log;
import com.technototes.library.logger.Loggable;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    @Log.Number (name = "Target Pose")
    public static Pose TARGET_POSE = new Pose(0,0);

    public static double UP = 0.1;
    public static double DOWN = 0.5;

    public static double LAUNCHER_VELOCITY = -0.5;

    boolean hasHardware;

    EncodedMotor<DcMotorEx> launcher;
    Servo turret1, turret2, hood;

    public TurretSubsystem(Hardware h, Robot r) {
        hasHardware = Setup.Connected.TURRETSUBSYSTEM;
        robot = r;
        // Do stuff in here
        if (hasHardware) {
            launcher = h.launcher;
            turret1 = h.turret1;
            turret2 = h.turret2;
            hood = h.hood;
            CommandScheduler.register(this);
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            launcher = null;
            turret1 = null;
            turret2 = null;
        }
        // Create the array to hold past current values
    }

    public void Launch() {
        // Spin the motors
        setLauncherVelocity(LAUNCHER_VELOCITY);
    }

    public void HoodUp() {
        setHoodPosition(UP);
    }

    public void HoodDown() {
        setHoodPosition(DOWN);
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


    @Override
    public void periodic() {
        getTurretPos();
        TARGET_SERVO_POS = getTurretPos();
        TARGET_POSE = getTargetPose();
        TURRET_HEADING = getTurretAngle();
        ROBOT_HEAD = robot.follower.pose().heading();


        // Add an item to the array and update the index for the next update to the 'circular' array
    }

    private void setLauncherVelocity(double velo) {
        if (hasHardware) {
            launcher.setPower(0.8);
            //            launcher.setVelocity(velo);
        }
    }

    private void setHoodPosition(double pos) {
        if (hasHardware) {
            hood.setPosition(pos);
        }
    }

    private void setTurretPosition(double pos) {
        if (hasHardware) {
            turret1.setPosition(pos);
            turret2.setPosition(pos);
        }
    }
}
