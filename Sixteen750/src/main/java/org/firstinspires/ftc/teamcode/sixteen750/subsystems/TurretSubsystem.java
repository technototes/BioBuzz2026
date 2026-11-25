package org.firstinspires.ftc.teamcode.sixteen750.subsystems;

import static java.lang.Math.clamp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.teamcode.sixteen750.commands.PedroDriver;
import org.firstinspires.ftc.teamcode.sixteen750.pc.TurretInputLog;

@Configurable
public class TurretSubsystem implements Loggable, Subsystem {

    final Robot robot;
    Pose HIVE_TARGET1 = new Pose(60, 54);
    Pose HIVE_TARGET2 = new Pose(60, 90);

    public static boolean turretSOTM = true;
    public static boolean sotmAccel = false; // acceleration term in SOTM aiming (shaky - see getSotmTargetPose)
    public static boolean velocitySOTM = false; // flywheel speed from distance to the SOTM-shifted target
    public static boolean hoodSOTM = true; // hood angle from distance to the SOTM-shifted target
    // Turret anti-shake (tunable in Panels):
    // smoothing time constant (seconds) for the robot velocity SOTM leads by; 0 = raw
    public static double SOTM_VELO_SMOOTHING = 0.15;
    // don't re-command the turret servos unless the aim moved more than this (servo units;
    // 1.0 = 360 degrees, so 0.003 is about 1 degree)
    public static double TURRET_DEADBAND = 0.003;
    // Turret turn-rate feedforward: lead the aim by (robot turn rate x this many seconds).
    // Roughly loop time + servo response; 0 = off.
    public static double TURRET_OMEGA_LEAD = 0.1;
    public static double TURRET_OMEGA_SMOOTHING = 0.05; // seconds; 0 = raw turn rate
    // Target-turn feedforward: seconds to lead by the not-yet-reached part of the follower's target
    // turn rate (0 = off). In teleop the target is the stick command scaled by
    // TURRET_STICK_MAX_OMEGA (robot turn rate in rad/s at a full rotation command); while following
    // a path it's the path's heading change per inch times path speed.
    public static double TURRET_TARGET_LEAD = 0.1;
    public static double TURRET_STICK_MAX_OMEGA = 5.0;
    private double smoothVx = 0,
        smoothVy = 0,
        smoothOmega = 0;
    // Turret interpolation: max servo travel per second while tracking (1.0 = 360 deg/s); 0 = jump
    // Safety clamp only - set at/above the servo's real top speed so it doesn't add lag (was 1.0).
    public static double TURRET_MAX_RATE = 2.0;
    // Bearing-rate feedforward: lead the aim by (aim bearing rate x this many seconds); 0 = off.
    public static double TURRET_BEARING_LEAD = 0.1;
    public static double TURRET_BEARING_SMOOTHING = 0.025; // seconds
    // Deadband only applies when turn rate and bearing rate are both below this (rad/s).
    public static double TURRET_STILL_RATE = 0.2;
    private double smoothBearingRate = 0,
        lastBearing = Double.NaN;
    private final ElapsedTime bearingTimer = new ElapsedTime();
    private double lastTurretCmd = Double.NaN;
    private final ElapsedTime turretTimer = new ElapsedTime();
    private final ElapsedTime smoothTimer = new ElapsedTime();

    public static Velocity previousVeloSOTM;
    private static ElapsedTime velocityTimer = new ElapsedTime();
    public static Velocity acceleration;

    public static double ejectiontime = .07;

    public static double TARGET_SWITCH_THRESHOLD = 72;
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.04;
    public static double TURRET_MAX = 0.9;

    @Log.Number(name = "target servo Pos")
    public static double targetServoPos = 0;

    public static double FLIGHT_TIME = 0.62;
    public static double SOTM_Y_SCALE = 1.0; // extra multiplier on the Y-velocity lead in SOTM

    @Log.Number(name = "xvelo")
    public static double xVelo = 0;

    @Log.Number(name = "yvelo")
    public static double yVelo = 0;

    @Log.Number(name = "Robot Heading")
    public static double robotHead = 0;

    //@Log.Number(name = "Turret Heading")
    public static double turretHead = 0;

    @Log.Number(name = "Target Velocity")
    public static double targetVelo = 0;

    @Log.Number(name = "Current Velocity")
    public static double currentVelo = 0;

    @Log(name = "Target Pose")
    public static Pose targetPose = new Pose(0, 0);

    //@Log(name = "sotm targetPose")
    public static Pose sotmTargetPose = new Pose(0, 0);

    @Log.Number(name = "final hood pose")
    public static double hoodAutoPos = 0.5; // final value we feed into the hood for position

    //@Log.Number(name = "Manual hood pos")
    public static double manualHoodPos = 0.5; // manual hood pos stuff for tuning

    public static double HOOD_INCREMENT = 0.01; // amount to manually adjust hood by for tuning
    public static double HOOD_POSITION_TO_ANGLE_CONSTANT = 43.42105; // did math to get this it is how the position relates to the angle in degrees
    public static boolean hoodVelocityComp = true; // adjust hood for flywheel speed error
    public static double hoodCompScalar = -0.0313; // 11/24/26: 2/3 of the old -0.047 // the ratio between our error in velocity ~in the couple of hundreds and the change in hood angle 21-38 degrees;
    public static double HOOD_MIN = 0.1;
    public static double HOOD_MAX = 0.5;
    public static double INCREASE = 15; // extremely jank increment and  decrement

    @Log.Number(name = "hood calculated Pos")
    public static double hoodTargetPos = 0.5;

    public static double actualTarget = 0; // target  for manual velocity tuning
    public static double launcherVelocity = 1500;

    @Log.Number(name = "Auto Velocity")
    public static double autoVelocity = 2000;

    public static double HOOD_REGRESSION_A = -0.000211401; // 0.00000232206; // slope of hood regression
    public static double HOOD_REGRESSION_B = 0.0260897; // -0.0003136;
    public static double HOOD_REGRESSION_C = 0.233456; // 0.0153873;
    //  public static double HOOD_REGRESSION_D = -0.315317;
    //  public static double HOOD_REGRESSION_E = 3.07; // offset to the slope (y intercept)
    public static double LAUNCHER_REGRESSION_A = -0.0662139; //0.00657618; // slope of launcher regression
    public static double LAUNCHER_REGRESSION_B = 9.58113; //-0.960648;
    public static double LAUNCHER_REGRESSION_C = 840; //51.54356;

    // public static double LAUNCHER_REGRESSION_D = -1197.91667; // offset to the launcher regression slope (y-intercept)

    //  public static double LAUNCHER_REGRESSION_E = 11400.0909;
    // public static double LAUNCHER_REGRESSION_E_SOTM = 11327.0909;

    @Log.Number(name = "rawdistance")
    public static double distanceToTarget = 0; // distance to the hive we are aiming at in inches

    @Log.Number(name = "SOTMdistance")
    public static double sotmDistanceToTarget = 0; // distance for sotm

    @Log.Number(name = "err")
    public static double error = 0; // error for launcher velo

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
    // 11/24/26 auto-tuned with scripts/launcher.ts (was 0.0024, 0.00013, 0.0, 0)
    public static PIDFCoefficients launchPID = new PIDFCoefficients(0.00384, 0.000208, 0.0, 0);
    private Hardware hardware;
    // Stuff used for the Feed Forward function.
    // This one is highly variable, based on the amount of friction in the system

    public static double kStaticFriction = 0.67; // measured 10/12

    public static double kDynamicFriction = 0.45; // measured 10/12

    // This one tends to be somewhere between 0.0035 to 0.005 or so.
    // 11/24/26: measured ~0.006 at 1050-1300 t/s while feeding balls, but that ran ~15-45 t/s
    // over target, so 0.0055 (was 0.0043)
    public static double kVelocityConstant = 0.0055;
    // Only integrate while within this many ticks/sec of target, so spin-up from rest doesn't
    // wind up I and overshoot (it overshot to ~1380 t/s on a 1200 target without this).
    public static double LAUNCHER_I_ZONE = 150;
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
            if (LOG_TURRET_INPUTS) TurretInputLog.start();
        } else {
            launcher = null;
            turret1 = null;
            turret2 = null;
        }
        pidfController = new PIDFController(
            launchPID,
            target ->
                (Math.signum(target) * // signum(<0) = -1, signum(>0) = 1, signum(0) = *0*
                    GetFrictionConstant(getActualVelocity() != 0) +
                    kVelocityConstant * target) /
                hardware.voltage()
        ).setIntegralZone(LAUNCHER_I_ZONE);
    }

    public void Launch() {
        // Spin the motors
        setVelocityTarget(autoVelocity);
    }

    public void ManualLaunch() {
        setVelocityTarget(launcherVelocity);
    }

    // returns the turret angle in radians relative to the robot
    public double getTurretAngle() {
        double robotHead = MathUtils.normalizeDeltaAngle(
            robot.follower.pose().heading(),
            AngleUnit.RADIANS
        ); // sets robotHead to robot heading and normalizes to delta radians
        double angle;
        // Bearing-rate feedforward (Thornbots AngleSetpointRate idea): lead the field-frame aim by
        // how fast it's changing, which covers driving sideways past the goal, not just turning.
        double absoluteAngle = getAimBearing() + smoothBearingRate * TURRET_BEARING_LEAD;

        // Turn-rate feedforward: aim for where the robot will be pointing TURRET_OMEGA_LEAD seconds
        // from now, so loop + servo lag doesn't leave the turret trailing while we rotate.
        // Target-turn feedforward: the turn the follower is being asked for that the robot hasn't
        // reached yet (target turn rate - measured), so the turret starts moving before the robot.
        double targetLead = (getTargetOmega() - smoothOmega) * TURRET_TARGET_LEAD;
        angle = absoluteAngle - (robotHead + smoothOmega * TURRET_OMEGA_LEAD + targetLead); // still in radians.......
        return MathUtils.normalizeDeltaRadians(angle);
    }

    // Field-frame direction (radians) from the robot to what the turret should aim at.
    private double getAimBearing() {
        Pose targetPose = getTargetPose();
        Pose sotmTargetPose = getSotmTargetPose();
        double x, y, dx, dy;
        x = robot.follower.pose().x(); // sets x to robots current y position
        y = robot.follower.pose().y(); // sets y to robots current y position
        if (turretSOTM) {
            dx = sotmTargetPose.x() - x;
            dy = sotmTargetPose.y() - y;
        } else {
            dx = targetPose.x() - x; // differ    ence between target x and robot x
            dy = targetPose.y() - y; // difference between target y and robot y
        }

        return Math.atan2(dy, dx); // in radians the target angle of the turret relative to the field
    }

    // Once per loop: low-pass the rate of change of the aim bearing (25 ms, like Thornbots), and
    // drop it on a jump (target switch / relocalization) instead of treating that as motion.
    private void updateBearingRate() {
        double bearing = getAimBearing();
        double dt = bearingTimer.seconds();
        bearingTimer.reset();
        if (Double.isNaN(lastBearing) || dt <= 0 || dt > 0.25) {
            lastBearing = bearing;
            smoothBearingRate = 0;
            return;
        }
        double change = MathUtils.normalizeDeltaRadians(bearing - lastBearing);
        lastBearing = bearing;
        if (Math.abs(change) > 0.25) {
            smoothBearingRate = 0;
            return;
        }
        double a = TURRET_BEARING_SMOOTHING <= 0 ? 1 : dt / (TURRET_BEARING_SMOOTHING + dt);
        smoothBearingRate += a * (change / dt - smoothBearingRate);
    }

    //returns the distance from the selected hive target in inches
    public double getDistance() {
        Pose robotPose = robot.follower.pose();
        Pose targetPose = getTargetPose();

        double Distance = robotPose.distance(targetPose);

        return clamp(Distance, 24, 48);
    }

    //returns the distance from the sotm target made the clamp slightly bigger cause why not ig
    public double getSotmDistance() {
        Pose robotPose = robot.follower.pose();
        Pose sotmTargetPose = getSotmTargetPose();

        double sotmDistance = robotPose.distance(sotmTargetPose);

        return clamp(sotmDistance, 24, 48);
    }

    // increment ignore how sus this implementation is btw
    public double increaseVelo() {
        actualTarget = launcherVelocity + INCREASE;
        return actualTarget;
    }

    // decrement
    public double decreaseVelo() {
        actualTarget = launcherVelocity - INCREASE;
        return actualTarget;
    }

    // determines which hive target to aim for based on what half of the field we are
    public Pose getTargetPose() {
        double Y;
        Pose targetPose;

        Y = robot.follower.pose().y();

        if (Y < TARGET_SWITCH_THRESHOLD) {
            targetPose = HIVE_TARGET1;
        } else {
            targetPose = HIVE_TARGET2;
        }

        return targetPose;
    }

    // gets our predicted pose by shifting the hive target by our velocity multiplied times our flight time (might have to make flight time a function we will see)
    public Pose getSotmTargetPose() {
        double Y, dx, dy;
        Pose sotmTargetPose;

        Y = robot.follower.pose().y();
        // The acceleration estimate is an unfiltered per-loop derivative of odometry velocity, so
        // it's mostly noise and made the turret shake. Off unless re-enabled (Panels: sotmAccel).
        double ax = sotmAccel ? acceleration.vx * ejectiontime : 0;
        double ay = sotmAccel ? acceleration.vy * ejectiontime : 0;
        dx = getXVelo() + ax;
        dy = getYVelo() + ay;

        Pose targetPose;
        if (Y < TARGET_SWITCH_THRESHOLD) {
            targetPose = HIVE_TARGET1;
        } else {
            targetPose = HIVE_TARGET2;
        }

        sotmTargetPose = new Pose(
            targetPose.x() - dx * FLIGHT_TIME,
            targetPose.y() - dy * FLIGHT_TIME * SOTM_Y_SCALE
        );
        /*
        if (Y < TARGET_SWITCH_THRESHOLD) {
            sotmTargetPose = new Pose(
                HIVE_TARGET1.x() - dx * FLIGHT_TIME,
                HIVE_TARGET1.y() - dy * FLIGHT_TIME
            );
        } else {
            sotmTargetPose = new Pose(
                HIVE_TARGET2.x() - dx * FLIGHT_TIME,
                HIVE_TARGET2.y() - dy * FLIGHT_TIME
            );
        }
*/
        return sotmTargetPose;
    }

    // takes our final turret angle does some math and returns a value in servo position aka 0-1
    public double getTurretPos() {
        double angle = getTurretAngle();
        double servoPos = TURRET_CENTER + angle / (2 * Math.PI);

        return clamp(servoPos, TURRET_MIN, TURRET_MAX);
    }

    public void setTurretTarget() {
        double target = getTurretPos();
        double dt = Math.min(turretTimer.seconds(), 0.1); // cap so a long gap can't cause a jump
        turretTimer.reset();
        if (Double.isNaN(lastTurretCmd)) {
            setTurretPosition(target);
            return;
        }
        double diff = target - lastTurretCmd;
        // Deadband: small pose/velocity noise shouldn't make the servos twitch every loop. Only while
        // nearly still - while turning/driving it would turn smooth tracking into a staircase.
        boolean still =
            Math.abs(smoothOmega) < TURRET_STILL_RATE &&
            Math.abs(smoothBearingRate) < TURRET_STILL_RATE;
        if (still && Math.abs(diff) <= TURRET_DEADBAND) return;
        // Interpolation: glide toward the target at no more than TURRET_MAX_RATE per second.
        double maxStep = TURRET_MAX_RATE <= 0 ? Math.abs(diff) : TURRET_MAX_RATE * dt;
        setTurretPosition(lastTurretCmd + clamp(diff, -maxStep, maxStep));
    }

    public void increaseVelocity() {
        launcherVelocity = increaseVelo();
    }

    public void decreaseVelocity() {
        launcherVelocity = decreaseVelo();
    }

    public void increaseHood() {
        manualHoodPos = manualHoodPos + HOOD_INCREMENT;
    }

    public void decreaseHood() {
        manualHoodPos = manualHoodPos - HOOD_INCREMENT;
    }

    // returns the target angle we want our hood to be at in degrees before we compensate for velocity
    public double getHoodTargetPos() {
        double x = hoodSOTM ? sotmDistanceToTarget : distanceToTarget;

        hoodTargetPos =
            HOOD_REGRESSION_A * Math.pow(x, 2) + HOOD_REGRESSION_B * x + HOOD_REGRESSION_C; //* Math.pow() +
        // HOOD_REGRESSION_D * x +
        //    HOOD_REGRESSION_E; // we run our distance into our regression formula

        return hoodTargetPos;
    }

    //returns the target servo position for our hood before we compensate for velocity
    /* public double getHoodTargetPos() {
        hoodTargetPos =
            HOOD_MAX - (hoodTargetAngle - HOOD_DOWN_ANGLE) / HOOD_POSITION_TO_ANGLE_CONSTANT; // basically shifting and scaling it to work

        return clamp(hoodTargetPos, HOOD_MIN, HOOD_MAX); // clamping it so the servo doesnt rebel from the rest of the robot
    } */

    // returns the final position in servo position (0-1) that we want our hood servo to be at after velocity compensation
    public double getHoodAutoPos() {
        double comp = hoodVelocityComp
            ? (error * hoodCompScalar) / HOOD_POSITION_TO_ANGLE_CONSTANT // takes our target and subtracts our error times a scalar
            : 0;
        hoodAutoPos = /*hoodTargetPos commented out for manual testing*/ hoodTargetPos + comp;
        return clamp(hoodAutoPos, HOOD_MIN, HOOD_MAX); // clamp it again so it doesnt try and unionize (i think you only need to clamp it once but by doing it twice both the compensated and uncompensated values are actually usable)
    }

    public void setHoodAutoPos() {
        setHoodPos(hoodAutoPos);
    }

    // Diagnostics: write every loop's turret inputs to /sdcard/FIRST/turretlogs (TurretInputLog).
    public static boolean LOG_TURRET_INPUTS = true;

    private void logTurretInputs() {
        Pose p = robot.follower.pose();
        Velocity v = robot.follower.velocity();
        Pose aim = turretSOTM ? getSotmTargetPose() : getTargetPose();
        TurretInputLog.log(
            p.x(),
            p.y(),
            p.heading(),
            v.vx,
            v.vy,
            v.omega,
            smoothVx,
            smoothVy,
            smoothOmega,
            aim.x(),
            aim.y(),
            getAimBearing(),
            smoothBearingRate,
            getTargetOmega(),
            PedroDriver.lastRotCmd,
            targetServoPos,
            lastTurretCmd
        );
    }

    @Override
    public void periodic() {
        updateSmoothedVelocity();
        updateBearingRate();
        updateAcceleration(true);
        targetServoPos = getTurretPos();
        if (LOG_TURRET_INPUTS) logTurretInputs();
        targetPose = getTargetPose();
        robotHead = robot.follower.pose().heading();
        targetVelo = getVelocityTarget();
        currentVelo = getActualVelocity();
        autoVelocity = getAutoVelocity();
        distanceToTarget = getDistance();
        hoodAutoPos = getHoodAutoPos();
        getHoodTargetPos();
        getHoodAutoPos();
        sotmDistanceToTarget = getSotmDistance();
        error = pidfController.getLastError();

        // Add an item to the array and update the index for the next update to the 'circular' array
        double power = pidfController.update(getActualVelocity());
        setLauncherPower(power);
    }

    private double getMotor1Current() {
        return hasHardware ? launcher.getAmperage(CurrentUnit.AMPS) : 0;
    }

    private double getActualVelocity() {
        if (hasHardware) {
            return launcher.getVelocity();
        } else {
            return pidfController.getTarget(); // Not a Number
        }
    }

    // takes distance and returns AUTO_VELOCITY by plugging it into the regression
    public double getAutoVelocity() {
        double x = velocitySOTM ? sotmDistanceToTarget : distanceToTarget;

        autoVelocity =
            LAUNCHER_REGRESSION_A * Math.pow(x, 2) +
            LAUNCHER_REGRESSION_B * x +
            LAUNCHER_REGRESSION_C; //* Math.pow(x, 2) +
        //   LAUNCHER_REGRESSION_D * x +
        // LAUNCHER_REGRESSION_E; // comment when use SOTM
        //  LAUNCHER_REGRESSION_E_SOTM;
        return autoVelocity;
    }

    // Explicitly set the target velocity for the motors
    public void setVelocityTarget(double speed) {
        pidfController.setTarget(speed);
    }

    public void setTurretPosition(double pos) {
        turret1.setPosition(pos);
        turret2.setPosition(pos);
        lastTurretCmd = pos;
    }

    // The turn rate (rad/s, + = counter-clockwise) the follower is trying to reach right now.
    private double getTargetOmega() {
        if (robot.follower.following()) {
            double t = robot.follower.parametricCompletion();
            if (Double.isNaN(t)) return 0;
            double t2 = Math.min(1, t + 0.01),
                t1 = Math.max(0, t2 - 0.01);
            Pose a = robot.follower.poseAt(t1),
                b = robot.follower.poseAt(t2);
            double ds = a.distance(b);
            if (ds < 1e-3) return 0; // turning in place: no path speed to scale by
            double dh = MathUtils.normalizeDeltaRadians(b.heading() - a.heading());
            return (dh / ds) * robot.follower.tangentialVelocity();
        }
        return PedroDriver.lastRotCmd * TURRET_STICK_MAX_OMEGA;
    }

    // Low-pass the follower velocity once per loop (called from periodic). SOTM multiplies this by
    // the flight time, so raw odometry velocity noise turned straight into turret jitter.
    private void updateSmoothedVelocity() {
        Velocity v = robot.follower.velocity();
        double dt = smoothTimer.seconds();
        smoothTimer.reset();
        double a = SOTM_VELO_SMOOTHING <= 0 ? 1 : Math.min(1, dt / (SOTM_VELO_SMOOTHING + dt));
        smoothVx += a * (v.vx - smoothVx);
        smoothVy += a * (v.vy - smoothVy);
        double ao =
            TURRET_OMEGA_SMOOTHING <= 0 ? 1 : Math.min(1, dt / (TURRET_OMEGA_SMOOTHING + dt));
        smoothOmega += ao * (v.omega - smoothOmega);
    }

    //gets our (smoothed) x velocity in in/s clamping it to not make the launcher have a seizure
    public double getXVelo() {
        xVelo = smoothVx;
        return clamp(xVelo, -50, 50);
    }

    //gets our (smoothed) y velocity in in/s clamping it to not make the launcher have a seizure
    public double getYVelo() {
        yVelo = smoothVy;
        return clamp(yVelo, -50, 50);
    }

    /*
    public Velocity getAcceleration(boolean update) {
        com.pedropathing.math.Velocity currentVelocity = robot.follower.velocity();
        if (previousVeloSOTM != null && update) {
            Velocity changeVelocity = currentVelocity.minus(previousVeloSOTM);
            double changeTime = velocityTimer.seconds();
            previousVeloSOTM = currentVelocity;
            velocityTimer.reset();
            acceleration = changeVelocity.scale(1 / changeTime);
        } else if (previousVeloSOTM == null) {
            previousVeloSOTM = currentVelocity;
            velocityTimer.reset();
            acceleration = currentVelocity;
        }
        return acceleration;
    }
     */

    public void updateAcceleration(boolean update) {
        Velocity currentVelocity = robot.follower.velocity();
        if (previousVeloSOTM != null && update) {
            Velocity changeVelocity = currentVelocity.minus(previousVeloSOTM);
            double changeTime = velocityTimer.seconds();
            previousVeloSOTM = currentVelocity;
            velocityTimer.reset();
            acceleration = changeVelocity.scale(1 / changeTime);
        } else if (previousVeloSOTM == null) {
            previousVeloSOTM = currentVelocity;
            velocityTimer.reset();
            acceleration = new Velocity(0, 0, 0);
        }
    }

    private void setHoodPos(double pos) {
        hood.setPosition(pos);
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
