package org.firstinspires.ftc.teamcode.sixteen750.pc;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Lets a PC set motor velocities/powers over HTTP, with no Driver Station interaction.
 *
 * <p>The Robot Controller's built-in web server (port 8080) gets a few extra endpoints under
 * /pcmotor/. The first command automatically inits + starts this OpMode (the same way FTC
 * Dashboard / Panels start OpModes), and the OpMode applies the requested targets each loop.
 *
 * <p>Safety: targets are a lease. If the PC hasn't sent anything for {@link #WATCHDOG_MS}, every
 * motor is set to zero. The PC tool (scripts/motors.ts) keeps the lease alive while it runs and
 * stops everything on Ctrl+C. It also refuses to take over if another OpMode is running.
 *
 * <p>Over USB: adb forward tcp:8080 tcp:8080, then http://127.0.0.1:8080/pcmotor/status
 */
@TeleOp(name = PcMotorControl.NAME, group = "zz dev")
public class PcMotorControl extends OpMode {

    public static final String NAME = "PC Motor Control";
    public static final long WATCHDOG_MS = 1000;

    /** A requested motor command: velocity in encoder ticks/sec, or raw power -1..1. */
    private static final class Target {

        final boolean velocity;
        final double value;

        Target(boolean velocity, double value) {
            this.velocity = velocity;
            this.value = value;
        }
    }

    private static final Map<String, Target> targets = new ConcurrentHashMap<>();
    private static volatile long lastContactMs = 0;
    public static final long IDLE_EXIT_MS = 30000;
    private static volatile PcMotorControl running = null;
    private static volatile boolean started = false;
    private static volatile String lastError = "";

    private final Map<String, Boolean> appliedVelocityMode = new ConcurrentHashMap<>();

    // ---------------------------------------------------------------- OpMode side

    @Override
    public void init() {
        appliedVelocityMode.clear();
        running = this;
        telemetry.addLine("Driven from the PC (scripts/motors.ts). Stop here to kill it.");
    }

    @Override
    public void init_loop() {
        applyTargets(false);
    }

    @Override
    public void start() {
        started = true;
    }

    @Override
    public void loop() {
        applyTargets(true);
        // Nobody's talked to us for a while: get out of the way so the robot is back to normal.
        if (System.currentTimeMillis() - lastContactMs > IDLE_EXIT_MS) {
            requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        started = false;
        targets.clear();
        for (DcMotorEx m : hardwareMap.getAll(DcMotorEx.class)) {
            m.setPower(0);
        }
        running = null;
    }

    private void applyTargets(boolean started) {
        boolean expired = System.currentTimeMillis() - lastContactMs > WATCHDOG_MS;
        if (expired && !targets.isEmpty()) {
            targets.clear();
        }
        for (Map.Entry<String, DcMotorEx> e : motors().entrySet()) {
            String name = e.getKey();
            DcMotorEx m = e.getValue();
            Target t = started ? targets.get(name) : null;
            if (t == null) {
                if (appliedVelocityMode.containsKey(name)) {
                    m.setPower(0);
                    appliedVelocityMode.remove(name);
                }
                continue;
            }
            Boolean was = appliedVelocityMode.get(name);
            if (was == null || was != t.velocity) {
                m.setMode(
                    t.velocity
                        ? DcMotor.RunMode.RUN_USING_ENCODER
                        : DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );
                appliedVelocityMode.put(name, t.velocity);
            }
            if (t.velocity) {
                m.setVelocity(t.value);
            } else {
                m.setPower(t.value);
            }
        }
        telemetry.addData("PC link", expired ? "idle (watchdog)" : "live");
        for (Map.Entry<String, Target> e : targets.entrySet()) {
            telemetry.addData(
                e.getKey(),
                (e.getValue().velocity ? "vel " : "pwr ") + e.getValue().value
            );
        }
    }

    private Map<String, DcMotorEx> motors() {
        Map<String, DcMotorEx> out = new java.util.TreeMap<>();
        for (DcMotorEx m : hardwareMap.getAll(DcMotorEx.class)) {
            for (String n : hardwareMap.getNamesOf(m)) {
                out.put(n, m);
            }
        }
        return out;
    }

    private String statusJson() {
        StringBuilder sb = new StringBuilder("{\"opmode\":\"running\",\"motors\":[");
        boolean first = true;
        for (Map.Entry<String, DcMotorEx> e : motors().entrySet()) {
            DcMotorEx m = e.getValue();
            Target t = targets.get(e.getKey());
            if (!first) sb.append(',');
            first = false;
            sb.append(
                String.format(
                    Locale.US,
                    "{\"name\":\"%s\",\"port\":%d,\"velocity\":%.1f,\"power\":%.3f,\"amps\":%.2f,\"position\":%d,\"target\":%s}",
                    e.getKey(),
                    m.getPortNumber(),
                    m.getVelocity(),
                    m.getPower(),
                    m.getCurrent(CurrentUnit.AMPS),
                    m.getCurrentPosition(),
                    t == null
                        ? "null"
                        : String.format(
                              Locale.US,
                              "{\"mode\":\"%s\",\"value\":%s}",
                              t.velocity ? "velocity" : "power",
                              t.value
                          )
                )
            );
        }
        return sb.append("]}").toString();
    }

    // ---------------------------------------------------------------- web side

    @WebHandlerRegistrar
    public static void attachWebHandlers(Context context, WebHandlerManager manager) {
        manager.register("/pcmotor/status", s -> handle(s, "status"));
        manager.register("/pcmotor/set", s -> handle(s, "set"));
        manager.register("/pcmotor/ping", s -> handle(s, "ping"));
        manager.register("/pcmotor/stop", s -> handle(s, "stop"));
    }

    private static NanoHTTPD.Response handle(NanoHTTPD.IHTTPSession s, String what) {
        try {
            switch (what) {
                case "status":
                    return json(statusOrIdle());
                case "ping":
                    lastContactMs = System.currentTimeMillis();
                    return json(statusOrIdle());
                case "stop":
                    targets.clear();
                    OpModeManagerImpl mgr = manager();
                    if (mgr != null && NAME.equals(mgr.getActiveOpModeName())) {
                        mgr.stopActiveOpMode();
                    }
                    return json("{\"ok\":true}");
                case "set":
                    return set(s.getParameters());
                default:
                    return error(NanoHTTPD.Response.Status.NOT_FOUND, "unknown");
            }
        } catch (Exception e) {
            lastError = e.toString();
            return error(NanoHTTPD.Response.Status.INTERNAL_ERROR, e.toString());
        }
    }

    /** /pcmotor/set?launcher=v1500&intake=p0.6  (v = ticks/sec, p = power). force=1 takes over. */
    private static NanoHTTPD.Response set(Map<String, List<String>> params)
        throws InterruptedException {
        boolean force = params.containsKey("force");
        String err = ensureRunning(force);
        if (err != null) {
            return error(NanoHTTPD.Response.Status.CONFLICT, err);
        }
        PcMotorControl op = running;
        Map<String, DcMotorEx> known = op.motors();
        for (Map.Entry<String, List<String>> e : params.entrySet()) {
            String name = e.getKey();
            if (name.equals("force") || e.getValue().isEmpty()) continue;
            if (!known.containsKey(name)) {
                return error(
                    NanoHTTPD.Response.Status.BAD_REQUEST,
                    "no motor named '" + name + "' (have " + known.keySet() + ")"
                );
            }
            String v = e.getValue().get(0).trim().toLowerCase(Locale.US);
            boolean velocity = !v.startsWith("p");
            double value = Double.parseDouble(v.replaceAll("^[vp]", ""));
            if (!velocity) value = Math.max(-1, Math.min(1, value));
            if (value == 0) targets.put(name, new Target(false, 0));
            else targets.put(name, new Target(velocity, value));
        }
        lastContactMs = System.currentTimeMillis();
        return json(op.statusJson());
    }

    /** Make sure our OpMode is running. Returns an error string, or null on success. */
    private static String ensureRunning(boolean force) throws InterruptedException {
        OpModeManagerImpl mgr = manager();
        if (mgr == null) return "Robot Controller activity not available";
        String active = mgr.getActiveOpModeName();
        if (NAME.equals(active) && running != null && started) return null;
        if (!NAME.equals(active) && !OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(active)) {
            if (!force) {
                return "'" + active + "' is running; stop it first (or use --force)";
            }
            mgr.stopActiveOpMode();
            if (
                !waitFor(() ->
                    OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(mgr.getActiveOpModeName())
                )
            ) {
                return "couldn't stop '" + active + "'";
            }
        }
        if (!NAME.equals(mgr.getActiveOpModeName())) {
            mgr.initOpMode(NAME);
            if (!waitFor(() -> running != null)) return "OpMode didn't init";
        }
        lastContactMs = System.currentTimeMillis();
        mgr.startActiveOpMode();
        if (!waitFor(() -> started)) return "OpMode didn't start";
        return null;
    }

    private interface Cond {
        boolean ok();
    }

    private static boolean waitFor(Cond c) throws InterruptedException {
        long end = System.currentTimeMillis() + 3000;
        while (System.currentTimeMillis() < end) {
            if (c.ok()) return true;
            Thread.sleep(20);
        }
        return c.ok();
    }

    private static String statusOrIdle() {
        PcMotorControl op = running;
        if (op != null) return op.statusJson();
        OpModeManagerImpl mgr = manager();
        String active = mgr == null ? "?" : mgr.getActiveOpModeName();
        return String.format(
            Locale.US,
            "{\"opmode\":\"idle\",\"active\":\"%s\",\"lastError\":\"%s\"}",
            active.replace("\"", "'"),
            lastError.replace("\"", "'")
        );
    }

    private static OpModeManagerImpl manager() {
        return OpModeManagerImpl.getOpModeManagerOfActivity(
            AppUtil.getInstance().getRootActivity()
        );
    }

    private static NanoHTTPD.Response json(String body) {
        return NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK,
            "application/json",
            body
        );
    }

    private static NanoHTTPD.Response error(NanoHTTPD.Response.Status st, String msg) {
        return NanoHTTPD.newFixedLengthResponse(
            st,
            "application/json",
            "{\"error\":\"" + msg.replace("\"", "'") + "\"}"
        );
    }
}
