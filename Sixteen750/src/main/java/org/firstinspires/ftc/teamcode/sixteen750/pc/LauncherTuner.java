package org.firstinspires.ftc.teamcode.sixteen750.pc;

import android.content.Context;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.WebHandlerManager;
import com.technototes.library.util.PIDFController;
import fi.iki.elonen.NanoHTTPD;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.sixteen750.Setup;
import org.firstinspires.ftc.teamcode.sixteen750.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.sixteen750.subsystems.TurretSubsystem;

/**
 * Launcher (flywheel) tuning rig, driven from the PC by scripts/launcher-tune.ts.
 *
 * <p>Runs the SAME controller TurretSubsystem uses (TechnoLib PIDFController + the
 * friction/kV/voltage feed-forward), with a target that sweeps sinusoidally between min and max.
 * Feeds balls continuously (intake + transfers + gate open, like IntakeSubsystem.Feed()), and logs
 * every loop so the PC can score the gains and try new ones on the fly.
 *
 * <p>Endpoints on the Robot Controller web server (port 8080):
 *   /launcher/start?...   start (auto-inits/starts this OpMode) with any config keys below
 *   /launcher/config?...  change config live: min max period p i d kv ks kd feed hood turret
 *   /launcher/data?since=N  CSV samples after seq N
 *   /launcher/status, /launcher/stop
 *
 * <p>Safety: everything stops if the PC goes quiet for 1.5 s, if the flywheel exceeds
 * MAX_SAFE_VELOCITY, or if it can't track (|error| > 500 for 0.75 s).
 */
@TeleOp(name = LauncherTuner.NAME, group = "zz dev")
public class LauncherTuner extends LinearOpMode {

    public static final String NAME = "Launcher Tuner";
    public static final long WATCHDOG_MS = 1500;
    public static final double MAX_SAFE_VELOCITY = 2600;

    // ---- live config (written by web thread, read by loop thread) ----
    static final class Config {

        double min = 1100,
            max = 1300,
            period = 5;
        double p = TurretSubsystem.launchPID.p;
        double i = TurretSubsystem.launchPID.i;
        double d = TurretSubsystem.launchPID.d;
        double kv = TurretSubsystem.kVelocityConstant;
        double ks = TurretSubsystem.kStaticFriction;
        double kd = TurretSubsystem.kDynamicFriction;
        boolean feed = true;
        double hood = 0.3;
        double turret = TurretSubsystem.TURRET_CENTER;

        Config copy() {
            Config c = new Config();
            c.min = min;
            c.max = max;
            c.period = period;
            c.p = p;
            c.i = i;
            c.d = d;
            c.kv = kv;
            c.ks = ks;
            c.kd = kd;
            c.feed = feed;
            c.hood = hood;
            c.turret = turret;
            return c;
        }

        String json() {
            return String.format(
                Locale.US,
                "{\"min\":%s,\"max\":%s,\"period\":%s,\"p\":%s,\"i\":%s,\"d\":%s,\"kv\":%s,\"ks\":%s,\"kd\":%s,\"feed\":%s,\"hood\":%s,\"turret\":%s}",
                min,
                max,
                period,
                p,
                i,
                d,
                kv,
                ks,
                kd,
                feed,
                hood,
                turret
            );
        }
    }

    private static volatile Config config = new Config();
    private static volatile int configVersion = 0;
    private static volatile long lastContactMs = 0;
    private static volatile boolean initted = false,
        started = false;
    private static volatile String fault = "";
    private static volatile double loopHz = 0;

    // ---- sample ring buffer ----
    private static final int N = 40000;
    private static final float[] sT = new float[N],
        sTarget = new float[N],
        sVel = new float[N];
    private static final float[] sPower = new float[N],
        sVolt = new float[N];
    private static final int[] sCfg = new int[N];
    private static long seq = 0; // guarded by LauncherTuner.class

    private static synchronized void record(
        float t,
        float target,
        float vel,
        float power,
        float volt,
        int cfg
    ) {
        int k = (int) (seq % N);
        sT[k] = t;
        sTarget[k] = target;
        sVel[k] = vel;
        sPower[k] = power;
        sVolt[k] = volt;
        sCfg[k] = cfg;
        seq++;
    }

    private static synchronized String csvSince(long since) {
        long from = Math.max(since + 1, Math.max(0, seq - N));
        long to = Math.min(seq, from + 8000);
        StringBuilder sb = new StringBuilder((int) (to - from) * 48 + 64);
        sb.append("seq,t,target,vel,power,volt,cfg\n");
        for (long s = from; s < to; s++) {
            int k = (int) (s % N);
            sb.append(s).append(',');
            sb.append(String.format(Locale.US, "%.4f", sT[k])).append(',');
            sb.append(String.format(Locale.US, "%.1f", sTarget[k])).append(',');
            sb.append(String.format(Locale.US, "%.1f", sVel[k])).append(',');
            sb.append(String.format(Locale.US, "%.4f", sPower[k])).append(',');
            sb.append(String.format(Locale.US, "%.3f", sVolt[k])).append(',');
            sb.append(sCfg[k]).append('\n');
        }
        return sb.toString();
    }

    // ---- the OpMode ----
    private double lastVolt = 12;
    private double lastVel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        fault = "";
        started = false;
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, Setup.HardwareNames.LAUNCHER_MOTOR);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, Setup.HardwareNames.INTAKE_MOTOR);
        DcMotorEx t1 = hardwareMap.get(DcMotorEx.class, Setup.HardwareNames.TRANSFER_MOTOR1);
        DcMotorEx t2 = hardwareMap.get(DcMotorEx.class, Setup.HardwareNames.TRANSFER_MOTOR2);
        Servo gate = hardwareMap.get(Servo.class, Setup.HardwareNames.GATE_SERVO);
        Servo hood = hardwareMap.tryGet(Servo.class, Setup.HardwareNames.HOOD_SERVO);
        Servo turret1 = hardwareMap.tryGet(Servo.class, Setup.HardwareNames.TURRET_SERVO1);
        Servo turret2 = hardwareMap.tryGet(Servo.class, Setup.HardwareNames.TURRET_SERVO2);

        // Match TurretSubsystem / IntakeSubsystem setup.
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for (DcMotorEx m : new DcMotorEx[] { intake, t1, t2 }) {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        PIDFCoefficients coeffs = new PIDFCoefficients(0, 0, 0, 0);
        PIDFController pidf = new PIDFController(coeffs, target -> {
            Config c = config;
            return (Math.signum(target) * (lastVel != 0 ? c.kd : c.ks) + c.kv * target) / lastVolt;
        }).setIntegralZone(TurretSubsystem.LAUNCHER_I_ZONE); // same as the real launcher

        initted = true;
        telemetry.addLine("Launcher Tuner - driven from the PC. STOP here kills it.");
        telemetry.update();
        waitForStart();
        started = true;
        lastContactMs = System.currentTimeMillis();

        long t0 = System.nanoTime();
        int appliedVersion = -1;
        boolean feeding = false;
        long overSince = -1;
        long lastTelem = 0;
        int loops = 0;
        long hzWindowStart = System.nanoTime();

        try {
            while (opModeIsActive()) {
                for (LynxModule h : hubs) h.clearBulkCache();
                long nowNs = System.nanoTime();
                double t = (nowNs - t0) / 1e9;
                long nowMs = System.currentTimeMillis();

                if (nowMs - lastContactMs > WATCHDOG_MS) {
                    fault = "PC link lost (watchdog)";
                    break;
                }

                Config c = config;
                int ver = configVersion;
                if (ver != appliedVersion) {
                    coeffs.p = c.p;
                    coeffs.i = c.i;
                    coeffs.d = c.d;
                    pidf.reset(); // don't let I built up under the old gains bump the new ones
                    if (hood != null && !Double.isNaN(c.hood)) hood.setPosition(c.hood);
                    if (!Double.isNaN(c.turret)) {
                        if (turret1 != null) turret1.setPosition(c.turret);
                        if (turret2 != null) turret2.setPosition(c.turret);
                    }
                    appliedVersion = ver;
                }

                // Feed once the flywheel has had a moment to spin up.
                boolean wantFeed = c.feed && t > 1.5 && fault.isEmpty();
                if (wantFeed != feeding) {
                    intake.setPower(wantFeed ? IntakeSubsystem.INTAKE_FEED_VELOCITY : 0);
                    t1.setPower(wantFeed ? IntakeSubsystem.TRANSFER1_FEED_VELOCITY : 0);
                    t2.setPower(wantFeed ? IntakeSubsystem.TRANSFER2_FEED_VELOCITY : 0);
                    gate.setPosition(
                        wantFeed ? IntakeSubsystem.FIRE_POSITION : IntakeSubsystem.BLOCK_POSITION
                    );
                    feeding = wantFeed;
                }

                double mid = (c.min + c.max) / 2,
                    amp = (c.max - c.min) / 2;
                double target = mid + amp * Math.sin((2 * Math.PI * t) / c.period);

                double vel = launcher.getVelocity();
                // Same as Hardware.voltage(): average of all hubs, read every loop.
                double v = 0;
                for (LynxModule h : hubs) v += h.getInputVoltage(VoltageUnit.VOLTS);
                lastVolt = hubs.isEmpty() ? 12 : v / hubs.size();
                lastVel = vel;

                pidf.setTarget(target);
                double power = pidf.update(vel);
                power = Math.max(-1, Math.min(1, power));

                if (fault.isEmpty()) {
                    if (Math.abs(vel) > MAX_SAFE_VELOCITY) {
                        fault = String.format(Locale.US, "overspeed %.0f t/s", vel);
                    } else if (t > 2 && Math.abs(target - vel) > 500) {
                        if (overSince < 0) overSince = nowMs;
                        if (nowMs - overSince > 750) fault = "can't track (|error|>500 for 0.75 s)";
                    } else {
                        overSince = -1;
                    }
                }
                if (!fault.isEmpty()) power = 0;
                launcher.setPower(power);

                record(
                    (float) t,
                    (float) target,
                    (float) vel,
                    (float) power,
                    (float) lastVolt,
                    appliedVersion
                );

                loops++;
                if (nowNs - hzWindowStart > 1e9) {
                    loopHz = loops / ((nowNs - hzWindowStart) / 1e9);
                    loops = 0;
                    hzWindowStart = nowNs;
                }
                if (nowMs - lastTelem > 500) {
                    lastTelem = nowMs;
                    telemetry.addData("target", "%.0f", target);
                    telemetry.addData("velocity", "%.0f", vel);
                    telemetry.addData("loop Hz", "%.0f", loopHz);
                    if (!fault.isEmpty()) telemetry.addData("FAULT", fault);
                    telemetry.update();
                }
            }
        } finally {
            launcher.setPower(0);
            intake.setPower(0);
            t1.setPower(0);
            t2.setPower(0);
            gate.setPosition(IntakeSubsystem.BLOCK_POSITION);
            started = false;
            initted = false;
        }
    }

    // ---- web side ----

    @WebHandlerRegistrar
    public static void attachWebHandlers(Context context, WebHandlerManager manager) {
        manager.register("/launcher/start", s -> handle(s, "start"));
        manager.register("/launcher/config", s -> handle(s, "config"));
        manager.register("/launcher/data", s -> handle(s, "data"));
        manager.register("/launcher/status", s -> handle(s, "status"));
        manager.register("/launcher/stop", s -> handle(s, "stop"));
    }

    private static NanoHTTPD.Response handle(NanoHTTPD.IHTTPSession s, String what) {
        try {
            Map<String, List<String>> q = s.getParameters();
            switch (what) {
                case "start": {
                    if (!started) {
                        // fresh run: clear any stale fault before checking
                        fault = "";
                    }
                    applyConfig(q);
                    lastContactMs = System.currentTimeMillis();
                    String err = RemoteOpMode.ensureRunning(
                        NAME,
                        () -> initted,
                        () -> started,
                        q.containsKey("force")
                    );
                    if (err != null) {
                        return RemoteOpMode.error(NanoHTTPD.Response.Status.CONFLICT, err);
                    }
                    return RemoteOpMode.json(status());
                }
                case "config":
                    applyConfig(q);
                    lastContactMs = System.currentTimeMillis();
                    return RemoteOpMode.json(status());
                case "data": {
                    lastContactMs = System.currentTimeMillis();
                    List<String> since = q.get("since");
                    long n = since == null || since.isEmpty() ? -1 : Long.parseLong(since.get(0));
                    return RemoteOpMode.respond("text/csv", csvSince(n));
                }
                case "status":
                    return RemoteOpMode.json(status());
                case "stop":
                    RemoteOpMode.stopIfActive(NAME);
                    return RemoteOpMode.json(status());
                default:
                    return RemoteOpMode.error(NanoHTTPD.Response.Status.NOT_FOUND, "unknown");
            }
        } catch (Exception e) {
            return RemoteOpMode.error(NanoHTTPD.Response.Status.INTERNAL_ERROR, e.toString());
        }
    }

    private static synchronized void applyConfig(Map<String, List<String>> q) {
        Config c = config.copy();
        boolean changed = false;
        for (Map.Entry<String, List<String>> e : q.entrySet()) {
            if (e.getValue().isEmpty()) continue;
            String v = e.getValue().get(0);
            switch (e.getKey()) {
                case "min":
                    c.min = Double.parseDouble(v);
                    break;
                case "max":
                    c.max = Double.parseDouble(v);
                    break;
                case "period":
                    c.period = Math.max(0.2, Double.parseDouble(v));
                    break;
                case "p":
                    c.p = Double.parseDouble(v);
                    break;
                case "i":
                    c.i = Double.parseDouble(v);
                    break;
                case "d":
                    c.d = Double.parseDouble(v);
                    break;
                case "kv":
                    c.kv = Double.parseDouble(v);
                    break;
                case "ks":
                    c.ks = Double.parseDouble(v);
                    break;
                case "kd":
                    c.kd = Double.parseDouble(v);
                    break;
                case "feed":
                    c.feed = v.equals("1") || v.equalsIgnoreCase("true");
                    break;
                case "hood":
                    c.hood = v.equals("off") ? Double.NaN : Double.parseDouble(v);
                    break;
                case "turret":
                    c.turret = v.equals("off") ? Double.NaN : Double.parseDouble(v);
                    break;
                default:
                    continue;
            }
            changed = true;
        }
        if (c.max > MAX_SAFE_VELOCITY - 200) c.max = MAX_SAFE_VELOCITY - 200;
        if (changed) {
            config = c;
            configVersion++;
        }
    }

    private static String status() {
        long n;
        synchronized (LauncherTuner.class) {
            n = seq;
        }
        return String.format(
            Locale.US,
            "{\"running\":%s,\"active\":\"%s\",\"fault\":\"%s\",\"loopHz\":%.1f,\"seq\":%d,\"cfg\":%d,\"config\":%s}",
            started,
            RemoteOpMode.activeName().replace("\"", "'"),
            fault.replace("\"", "'"),
            loopHz,
            n - 1,
            configVersion,
            config.json()
        );
    }
}
