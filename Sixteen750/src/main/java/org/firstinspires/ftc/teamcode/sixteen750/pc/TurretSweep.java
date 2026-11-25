package org.firstinspires.ftc.teamcode.sixteen750.pc;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.WebHandlerManager;
import fi.iki.elonen.NanoHTTPD;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.teamcode.sixteen750.Setup;
import org.firstinspires.ftc.teamcode.sixteen750.subsystems.TurretSubsystem;

/**
 * Turret sweep experiment: both turret servos follow a sine between TURRET_MIN and TURRET_MAX
 * (TurretSubsystem's limits), default 4 s period, starting from the middle.
 *
 * <p>Run it from the Driver Hub ("Turret Sweep", Stop ends it) or from the PC with
 * scripts/turret.ts (/turret/start?period=4&min=..&max=..). When started from the PC it stops by
 * itself if the PC goes quiet for 1.5 s.
 */
@TeleOp(name = TurretSweep.NAME, group = "zz dev")
public class TurretSweep extends LinearOpMode {

    public static final String NAME = "Turret Sweep";
    public static final long WATCHDOG_MS = 1500;

    private static volatile double period = 4,
        min = TurretSubsystem.TURRET_MIN,
        max = TurretSubsystem.TURRET_MAX;
    private static volatile boolean remote = false,
        initted = false,
        started = false;
    private static volatile long lastContactMs = 0;
    private static volatile double lastPos = Double.NaN,
        lastT = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        started = false;
        Servo t1 = hardwareMap.get(Servo.class, Setup.HardwareNames.TURRET_SERVO1);
        Servo t2 = hardwareMap.get(Servo.class, Setup.HardwareNames.TURRET_SERVO2);
        initted = true;
        telemetry.addLine("Turret Sweep: turret moves between its limits on a sine once started.");
        telemetry.update();
        waitForStart();
        started = true;
        lastContactMs = System.currentTimeMillis();
        long t0 = System.nanoTime();
        try {
            while (opModeIsActive()) {
                if (remote && System.currentTimeMillis() - lastContactMs > WATCHDOG_MS) break;
                double t = (System.nanoTime() - t0) / 1e9;
                double lo = Math.max(0, Math.min(min, max)),
                    hi = Math.min(1, Math.max(min, max));
                double pos = (lo + hi) / 2 + ((hi - lo) / 2) * Math.sin((2 * Math.PI * t) / period);
                t1.setPosition(pos);
                t2.setPosition(pos);
                lastPos = pos;
                lastT = t;
                telemetry.addData("t", "%.1f s", t);
                telemetry.addData(
                    "position",
                    "%.3f  (%.3f..%.3f, %.1f s period)",
                    pos,
                    lo,
                    hi,
                    period
                );
                telemetry.update();
            }
        } finally {
            started = false;
            initted = false;
            remote = false;
        }
    }

    @WebHandlerRegistrar
    public static void attachWebHandlers(Context context, WebHandlerManager manager) {
        manager.register("/turret/start", s -> handle(s, "start"));
        manager.register("/turret/status", s -> handle(s, "status"));
        manager.register("/turret/stop", s -> handle(s, "stop"));
    }

    private static NanoHTTPD.Response handle(NanoHTTPD.IHTTPSession s, String what) {
        try {
            Map<String, List<String>> q = s.getParameters();
            switch (what) {
                case "start": {
                    Double p = num(q, "period"),
                        lo = num(q, "min"),
                        hi = num(q, "max");
                    if (p != null) period = Math.max(0.5, p);
                    if (lo != null) min = lo;
                    if (hi != null) max = hi;
                    remote = true;
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
                case "status":
                    lastContactMs = System.currentTimeMillis();
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

    private static Double num(Map<String, List<String>> q, String k) {
        List<String> v = q.get(k);
        return v == null || v.isEmpty() ? null : Double.parseDouble(v.get(0));
    }

    private static String status() {
        return String.format(
            Locale.US,
            "{\"running\":%s,\"active\":\"%s\",\"t\":%.2f,\"pos\":%.4f,\"min\":%s,\"max\":%s,\"period\":%s}",
            started,
            RemoteOpMode.activeName().replace("\"", "'"),
            lastT,
            Double.isNaN(lastPos) ? 0 : lastPos,
            min,
            max,
            period
        );
    }
}
