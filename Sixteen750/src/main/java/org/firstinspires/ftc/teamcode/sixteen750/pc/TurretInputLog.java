package org.firstinspires.ftc.teamcode.sixteen750.pc;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Per-loop log of everything the turret aim is computed from, written to the robot's storage
 * (/sdcard/FIRST/turretlogs/turret-NNN.csv) so it can be pulled with adb after a drive and
 * analyzed (see doc/experiments.md). One file per OpMode run that constructs TurretSubsystem.
 */
public final class TurretInputLog {

    public static final String DIR = "/sdcard/FIRST/turretlogs";
    public static final String HEADER =
        "t,dt,x,y,heading,vx,vy,omega,svx,svy,somega,aimX,aimY,bearing,bearingRate," +
        "targetOmega,rotCmd,turretTarget,turretCmd";
    private static final int MAX_LINES = 60000; // ~10 minutes at 100 Hz

    private static BufferedWriter out;
    private static long t0Ns, lastNs, lines, lastFlushNs;

    private TurretInputLog() {}

    /** Start a new log file (closes any previous one). */
    public static synchronized void start() {
        close();
        try {
            File dir = new File(DIR);
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            int n = 0;
            File f;
            do {
                f = new File(dir, String.format(Locale.US, "turret-%03d.csv", n++));
            } while (f.exists());
            out = new BufferedWriter(new FileWriter(f), 1 << 16);
            out.write(HEADER);
            out.write('\n');
            t0Ns = lastNs = lastFlushNs = System.nanoTime();
            lines = 0;
        } catch (IOException e) {
            out = null;
        }
    }

    /** Log one loop. Values are written in HEADER order after t and dt. */
    public static synchronized void log(double... v) {
        if (out == null || lines >= MAX_LINES) return;
        long now = System.nanoTime();
        try {
            StringBuilder sb = new StringBuilder(200);
            sb.append(
                String.format(Locale.US, "%.4f,%.4f", (now - t0Ns) / 1e9, (now - lastNs) / 1e9)
            );
            for (double d : v) {
                sb.append(',');
                sb.append(Double.isNaN(d) ? "" : String.format(Locale.US, "%.5g", d));
            }
            sb.append('\n');
            out.write(sb.toString());
            lines++;
            lastNs = now;
            if (now - lastFlushNs > 1e9) {
                out.flush();
                lastFlushNs = now;
            }
        } catch (IOException e) {
            close();
        }
    }

    public static synchronized void close() {
        if (out == null) return;
        try {
            out.close();
        } catch (IOException ignored) {}
        out = null;
    }
}
