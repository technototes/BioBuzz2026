# Running robot experiments from the PC

These tools let a laptop drive motors and run tuning experiments on the 16750
robot without touching the Driver Station. Each experiment is an OpMode in the
`zz dev` group that the PC starts by itself through the Robot Controller's
built-in web server (port 8080), the same way FTC Dashboard and Panels start
OpModes. All of them can also be started from the Driver Hub.

| Experiment       | Robot side                                         | PC side            |
| ---------------- | -------------------------------------------------- | ------------------ |
| Drive any motor  | `pc/PcMotorControl.java` ("PC Motor Control")      | `bun run motors`   |
| Flywheel tuning  | `pc/LauncherTuner.java` ("Launcher Tuner")         | `bun run launcher` |
| Turret sweep     | `pc/TurretSweep.java` ("Turret Sweep")             | `bun run turret`   |
| Turret input log | `pc/TurretInputLog.java` (runs inside TeleOp/auto) | `adb pull`         |

All paths are under
`Sixteen750/src/main/java/org/firstinspires/ftc/teamcode/sixteen750/`.

## Safety

- The flywheel, intake, transfers and turret really move. Keep hands, cables and
  balls clear before starting anything.
- Every PC-started experiment has a watchdog: if the PC stops talking to it for
  1–1.5 s (cable pulled, script killed, laptop asleep), everything stops.
- Pressing **Stop** on the Driver Hub always kills the experiment.
- The PC tools refuse to take over if someone already started another OpMode,
  unless you pass `--force`.
- `PC Motor Control` uses raw motor directions, not the reversals the robot code
  applies. Commanding `fl`/`fr`/`rl`/`rr` will drive the robot: put it on a
  block first.

## One-time setup

1. Install a JDK 17+ (we use Temurin 21) and the Android SDK (platform 35 and
   platform-tools). Android Studio installs both.
2. Install [Bun](https://bun.sh) and run `bun install` at the repo root.
3. Make sure `adb` works: `adb devices` should list the Control Hub when it's
   plugged in over USB.
4. Build and install the robot code: `./gradlew :Sixteen750:installDebug`.

After an install the Robot Controller app needs about a minute on first launch
(Sloth rescans the code). If a PC tool says an OpMode "didn't init" right after
an install, wait a few seconds and run it again.

### USB or robot Wi-Fi

- **USB:** plug the laptop into the Control Hub. The scripts run
  `adb forward tcp:8080 tcp:8080` themselves.
- **Robot Wi-Fi:** connect the laptop to the robot's network and add
  `--host 192.168.43.1` to any command.

Note: school guest Wi-Fi (BSD Guest Access) sometimes blocks jitpack.io and the
npm registry. If a Gradle build or `bun install` hangs with no CPU use, that's
why; use another network for the first build.

## Drive any motor: `bun run motors`

```sh
bun run motors status                        # list motors with velocity, power, current, position
bun run motors set launcher=1500             # velocity in encoder ticks/sec
bun run motors set intake=p0.6 fl=p-0.2      # p = raw power, -1..1
bun run motors set launcher=1500 --for 5     # run 5 seconds, then stop
bun run motors stop
```

`set` keeps running and prints live readings until Ctrl+C. The OpMode exits on
its own after 30 s without commands.

## Flywheel tuning: `bun run launcher`

`Launcher Tuner` runs **the same controller as the real launcher**
(`TurretSubsystem`'s TechnoLib `PIDFController` with the friction/kV/voltage
feed-forward and `LAUNCHER_I_ZONE`), so whatever it finds applies directly to
TeleOp and auto. It sweeps the target speed on a sine wave, holds the feed on
(intake, transfers and gate open, same powers as `IntakeSubsystem.Feed()`), and
logs every loop. The turret is held at 0.5 and the hood at 0.3 unless you pass
`--turret`/`--hood` (`off` leaves a servo unpowered).

Run a sweep with given gains and watch it:

```sh
bun run launcher run --for 60                         # current gains, 1100-1300 t/s, 5 s period
bun run launcher run --min 1000 --max 1250 --period 4 --p 0.004 --kv 0.0055
```

Every 10 s it prints a score line:
`score | rms | track | shot | shots | dip | rec | chatter | Hz`.

- **rms**: overall error.
- **track**: error between shots.
- **shot**: error in the 0.6 s after each ball.
- **dip**: how far speed dropped per ball; **rec**: time to recover.
- **chatter**: power jitter.
- Lower is better for all of them.

Automatic tuning:

```sh
bun run launcher tune            # ~3-4 minutes; feed balls the WHOLE time
```

1. Runs a baseline with the current gains.
2. Fits the feed-forward from the data.
3. Searches P, then I, then D, then P again, in about 11 s trials.
4. Runs the best gains and the starting gains back to back.

It prints the lines to paste into `TurretSubsystem.java`. It does not edit the
code.

Logs go to `launcher-logs/` (git-ignored): one CSV per run with every loop
(`t, target, vel, power, volt, cfg`), plus a `-trials.json` for `tune`.

The tuner stops the launcher by itself on overspeed (> 2600 t/s), if it can't
track (error > 500 t/s for 0.75 s), or if the PC link is lost.

### What we found (11/24/26)

- Old gains (P 0.0024, I 0.00013, kV 0.0043): flywheel ran 35–70 t/s below
  target and lagged the sweep by ~0.9 s. Under ball load it needed 7–8 V where
  the feed-forward supplied 5–6 V, so P and I were doing most of the work.
- New gains (P 0.00384, I 0.000208, kV 0.0055): RMS error 190 → 87 RPM.
- Spin-up from rest overshot (~1380 t/s on a 1200 target) because I wound up
  during spin-up. `LAUNCHER_I_ZONE = 150` (I only integrates within 150 t/s of
  target) fixes that.
- **Battery matters a lot.** We saw 9.4–11.9 V during runs, and near 1300 t/s
  the motor hit 100% power. Tune on a charged battery, or the numbers move with
  the battery.
- RPM in our charts assumes the launcher motor reports 28 ticks per rev (goBILDA
  1:1): RPM = ticks/sec × 60 / 28.

## Turret sweep: `bun run turret`

```sh
bun run turret                    # TURRET_MIN..TURRET_MAX (0.04..0.9), 4 s period
bun run turret --period 2 --for 20
bun run turret --min 0.3 --max 0.7
```

Both turret servos follow a sine between the limits. The servos have no position
feedback, so this only commands the motion. Watch it for stutter, lag at the
ends, or buzzing (which would mean the two servos fight each other).

Result (11/24/26): a full-range 4 s sweep is perfectly smooth. The servos and
mechanism are fine.

## Turret input log

While `TurretSubsystem.LOG_TURRET_INPUTS` is true (default for now), every
OpMode that builds `TurretSubsystem` writes one row per loop to
`/sdcard/FIRST/turretlogs/turret-NNN.csv` on the robot. Each row has odometry
pose, raw and smoothed velocity and turn rate, the SOTM aim point, aim bearing
and its rate, target turn rate, stick command, computed turret target and the
servo command.

To collect a useful log, run TeleOp with tracking on and do about 10–15 s of
each:

1. sit completely still
2. drive slowly forward and back
3. spin in place
4. strafe sideways past the goal

Then stop and pull the logs:

```sh
adb pull /sdcard/FIRST/turretlogs
```

## Known issue: turret jitter is NOT solved

The turret still jitters while tracking. **Likely cause found, fix not yet
verified on the robot:** the tracking command was a one-shot bound with
`whileInverseToggled`, so the turret only got a new position a few times a
second (about 3 Hz observed). The `fix-toggled-commands` branch makes
`TrackHive` and `AutoHood` run every loop. Test that first; if it fixes the
jitter, the smoothing and lead settings below should be re-checked, because they
were all tuned by eye against the 3 Hz updates.

Other things tried before that was found. Things tried (all tunable in Panels,
`TurretSubsystem`):

- SOTM acceleration term off (`sotmAccel`): it was a raw per-loop derivative.
- Velocity smoothing (`SOTM_VELO_SMOOTHING` 0.15 s).
- Deadband, only while nearly still (`TURRET_DEADBAND`, `TURRET_STILL_RATE`).
- Setpoint slew limit / interpolation (`TURRET_MAX_RATE`).
- Lead feedforwards: measured turn rate (`TURRET_OMEGA_LEAD`), target turn rate
  (`TURRET_TARGET_LEAD`), aim bearing rate (`TURRET_BEARING_LEAD`).

The smooth sweep shows the hardware is fine, so the jitter comes from the aim
inputs: garbage in, garbage out. It is **also jerky with `turretSOTM` off**, so
the SOTM velocity terms are not the (only) cause. The next step is to analyze a
turret input log. Suspects, most likely first:

- uneven or slow TeleOp loop timing: the turret only gets a new position once
  per loop, so long or irregular loops look like steps while turning (check the
  `dt` column)
- odometry heading or pose noise
- the lead terms, which still run with SOTM off (turn rate, target turn rate,
  bearing rate)
- the goal switching at Y = 72 when pose noise crosses it
- with SOTM on: velocity noise multiplied by the 0.62 s flight time, or velocity
  in the robot frame and not the field frame

Quick checks in Panels: set the three `*_LEAD` values to 0 one at a time. If it
still shakes standing still with all of them at 0, it's odometry noise.

## Adding a new experiment

Copy the pattern in `TurretSweep.java`:

1. Write a `LinearOpMode` in `pc/` with `@TeleOp(group = "zz dev")`.
2. Add a `@WebHandlerRegistrar` method that registers `/yourthing/start`,
   `/status` and `/stop`.
3. `start` calls `RemoteOpMode.ensureRunning(...)`.
4. Add a watchdog that stops when the PC goes quiet.

Then copy `scripts/turret.ts` for the PC side and add a line to `package.json`.
