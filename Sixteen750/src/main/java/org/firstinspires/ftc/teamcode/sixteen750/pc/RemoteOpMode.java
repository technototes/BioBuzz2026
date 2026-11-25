package org.firstinspires.ftc.teamcode.sixteen750.pc;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import fi.iki.elonen.NanoHTTPD;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** Helpers for starting/stopping OpModes from the Robot Controller web server (no DS needed). */
public final class RemoteOpMode {

    private RemoteOpMode() {}

    public static OpModeManagerImpl manager() {
        return OpModeManagerImpl.getOpModeManagerOfActivity(
            AppUtil.getInstance().getRootActivity()
        );
    }

    public static String activeName() {
        OpModeManagerImpl mgr = manager();
        return mgr == null ? "?" : mgr.getActiveOpModeName();
    }

    /**
     * Make sure OpMode {@code name} is running (init + start). {@code initted} must become true in
     * the OpMode's init, {@code started} once it's past start. Returns an error string, or null.
     */
    public static String ensureRunning(
        String name,
        BooleanSupplier initted,
        BooleanSupplier started,
        boolean force
    ) throws InterruptedException {
        OpModeManagerImpl mgr = manager();
        if (mgr == null) return "Robot Controller activity not available";
        String active = mgr.getActiveOpModeName();
        if (name.equals(active) && started.getAsBoolean()) return null;
        if (!name.equals(active) && !OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(active)) {
            if (!force) return "'" + active + "' is running; stop it first (or use --force)";
            mgr.stopActiveOpMode();
            if (
                !waitFor(() ->
                    OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(mgr.getActiveOpModeName())
                )
            ) {
                return "couldn't stop '" + active + "'";
            }
        }
        if (!name.equals(mgr.getActiveOpModeName())) {
            mgr.initOpMode(name);
            if (!waitFor(initted)) return name + " didn't init";
        }
        mgr.startActiveOpMode();
        if (!waitFor(started)) return name + " didn't start";
        return null;
    }

    public static void stopIfActive(String name) {
        OpModeManagerImpl mgr = manager();
        if (mgr != null && name.equals(mgr.getActiveOpModeName())) mgr.stopActiveOpMode();
    }

    public static boolean waitFor(BooleanSupplier c) throws InterruptedException {
        long end = System.currentTimeMillis() + 3000;
        while (System.currentTimeMillis() < end) {
            if (c.getAsBoolean()) return true;
            Thread.sleep(20);
        }
        return c.getAsBoolean();
    }

    public static NanoHTTPD.Response respond(String mime, String body) {
        return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, mime, body);
    }

    public static NanoHTTPD.Response json(String body) {
        return respond("application/json", body);
    }

    public static NanoHTTPD.Response error(NanoHTTPD.Response.Status st, String msg) {
        return NanoHTTPD.newFixedLengthResponse(
            st,
            "application/json",
            "{\"error\":\"" + msg.replace("\"", "'") + "\"}"
        );
    }
}
