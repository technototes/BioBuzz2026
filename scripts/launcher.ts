// Launcher (flywheel) PID tuning from the PC. Talks to LauncherTuner.java on the robot.
//
//   bun run launcher run [--min 1100 --max 1300 --period 5] [--p .. --i .. --d .. --kv .. --kd ..]
//        sweep + feed balls with the given gains (default: TurretSubsystem's current ones),
//        live readout, Ctrl+C to stop. Logs every loop to launcher-logs/<time>.csv
//   bun run launcher tune [same options]
//        autonomous tuning: baseline -> fit feed-forward from data -> search P, I, D, P.
//        Keep feeding balls the whole time. Prints the best gains at the end.
//   bun run launcher stop
//
// Options: --host 192.168.43.1 (robot Wi-Fi instead of USB), --force (take over a running
// OpMode), --no-feed (don't run intake/transfer/gate), --trial 10 (seconds per tuning trial),
// --hood 0.3 / --turret 0.5 (servo positions, or 'off' to leave them unpowered).

import { appendFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { homedir } from 'node:os';
import { join } from 'node:path';

// ---------------------------------------------------------------- args
const argv = process.argv.slice(2);
const cmd = argv.shift() ?? '';
const opts: Record<string, string> = {};
const flags = new Set<string>();
for (let k = 0; k < argv.length; k++) {
  const a = argv[k];
  if (!a.startsWith('--')) continue;
  const name = a.slice(2);
  const next = argv[k + 1];
  if (next !== undefined && !next.startsWith('--')) {
    opts[name] = next;
    k++;
  } else {
    flags.add(name);
  }
}
const host = opts.host;
const base = `http://${host ?? '127.0.0.1'}:8080/launcher`;
const trialSecs = Number(opts.trial ?? 10);
const SETTLE = 1.0; // seconds ignored after each gain change

// ---------------------------------------------------------------- adb + http
function adbPath(): string {
  const dirs = [
    process.env.ANDROID_HOME &&
      join(process.env.ANDROID_HOME, 'platform-tools'),
    process.env.LOCALAPPDATA &&
      join(process.env.LOCALAPPDATA, 'Android', 'sdk', 'platform-tools'),
    join(homedir(), 'Library', 'Android', 'sdk', 'platform-tools'),
    join(homedir(), 'Android', 'Sdk', 'platform-tools'),
  ].filter((p): p is string => !!p);
  for (const d of dirs)
    for (const exe of ['adb.exe', 'adb'])
      if (existsSync(join(d, exe))) return join(d, exe);
  return 'adb';
}
function forwardUsb(): void {
  if (host) return;
  const r = Bun.spawnSync([adbPath(), 'forward', 'tcp:8080', 'tcp:8080']);
  if (r.exitCode !== 0) {
    console.error(
      'adb forward failed - is the robot plugged in over USB?\n' +
        r.stderr.toString(),
    );
    process.exit(1);
  }
}

type Config = {
  min: number;
  max: number;
  period: number;
  p: number;
  i: number;
  d: number;
  kv: number;
  ks: number;
  kd: number;
  feed: boolean;
  hood: number;
  turret: number;
};
type Status = {
  running: boolean;
  active: string;
  fault: string;
  loopHz: number;
  seq: number;
  cfg: number;
  config: Config;
  error?: string;
};

async function get(path: string): Promise<Response> {
  try {
    return await fetch(base + path, { signal: AbortSignal.timeout(5000) });
  } catch (e) {
    throw new Error(`can't reach the robot at ${base} (${e})`);
  }
}
async function call(
  path: string,
  q: Record<string, string | number | boolean> = {},
): Promise<Status> {
  const qs = new URLSearchParams(
    Object.entries(q).map(([k, v]) => [k, String(v)]),
  ).toString();
  const res = await get(path + (qs ? '?' + qs : ''));
  if (res.status === 404)
    throw new Error(
      'robot has no /launcher endpoints - install the current build',
    );
  const body = (await res.json()) as Status;
  if (body.error) throw new Error(body.error);
  return body;
}

// ---------------------------------------------------------------- data collection
type Sample = {
  seq: number;
  t: number;
  target: number;
  vel: number;
  power: number;
  volt: number;
  cfg: number;
};
const samples: Sample[] = [];
let lastSeq = -1;
let logFile = '';
let collecting = false;
let lastStatus: Status | null = null;

function startLog(): void {
  const dir = join(import.meta.dir, '..', 'launcher-logs');
  mkdirSync(dir, { recursive: true });
  if (!existsSync(join(dir, '.gitignore')))
    writeFileSync(join(dir, '.gitignore'), '*\n');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  logFile = join(dir, `${cmd}-${stamp}.csv`);
  writeFileSync(logFile, 'seq,t,target,vel,power,volt,cfg\n');
}

async function pullOnce(): Promise<void> {
  const res = await get(`/data?since=${lastSeq}`);
  const text = await res.text();
  const lines = text.trim().split('\n').slice(1);
  if (lines.length === 0 || lines[0] === '') return;
  appendFileSync(logFile, lines.join('\n') + '\n');
  for (const l of lines) {
    const [seq, t, target, vel, power, volt, cfg] = l.split(',').map(Number);
    samples.push({ seq, t, target, vel, power, volt, cfg });
    lastSeq = seq;
  }
}

async function collector(): Promise<void> {
  collecting = true;
  let n = 0;
  while (collecting) {
    try {
      await pullOnce();
      if (++n % 4 === 0) {
        lastStatus = await call('/status');
        if (lastStatus.fault)
          throw new Error('robot fault: ' + lastStatus.fault);
        if (!lastStatus.running)
          throw new Error('tuner OpMode stopped on the robot');
      }
    } catch (e) {
      console.error('\n' + (e instanceof Error ? e.message : e));
      await shutdown(1);
    }
    await Bun.sleep(150);
  }
}

let shuttingDown = false;
async function shutdown(code: number): Promise<never> {
  if (!shuttingDown) {
    shuttingDown = true;
    collecting = false;
    await call('/stop').catch(() => {});
    if (logFile) console.log(`\nlauncher stopped. log: ${logFile}`);
  }
  process.exit(code);
}
process.on('SIGINT', () => void shutdown(0));

// ---------------------------------------------------------------- analysis
type Metrics = {
  n: number;
  secs: number;
  rms: number;
  trackRms: number;
  shotRms: number;
  shots: number;
  dip: number;
  recover: number;
  chatter: number;
  score: number;
  hz: number;
};

function analyze(s: Sample[]): Metrics {
  const n = s.length;
  const secs = n > 1 ? s[n - 1].t - s[0].t : 0;
  const err = s.map((x) => x.target - x.vel);
  // Shot detection: velocity falls >= 50 t/s below its value ~60 ms earlier (balls hit hard + fast).
  const shotIdx: number[] = [];
  let j = 0;
  for (let k = 0; k < n; k++) {
    while (s[k].t - s[j].t > 0.06) j++;
    const drop = s[j].vel - s[k].vel - (s[j].target - s[k].target);
    if (
      drop > 50 &&
      (shotIdx.length === 0 || s[k].t - s[shotIdx[shotIdx.length - 1]].t > 0.3)
    ) {
      // back up to where the drop started
      shotIdx.push(j);
    }
  }
  const inShot = new Array<boolean>(n).fill(false);
  const dips: number[] = [];
  const recovers: number[] = [];
  for (const si of shotIdx) {
    let worst = 0;
    let rec = NaN;
    for (let k = si; k < n && s[k].t - s[si].t < 0.6; k++) {
      inShot[k] = true;
      worst = Math.max(worst, err[k]);
    }
    let pastPeak = false;
    for (let k = si; k < n && s[k].t - s[si].t < 1.5; k++) {
      if (err[k] >= worst - 1) pastPeak = true;
      if (pastPeak && Math.abs(err[k]) < 20) {
        rec = s[k].t - s[si].t;
        break;
      }
    }
    dips.push(worst);
    recovers.push(isNaN(rec) ? 1.5 : rec);
  }
  const rmsOf = (sel: (k: number) => boolean) => {
    let sum = 0,
      c = 0;
    for (let k = 0; k < n; k++)
      if (sel(k)) {
        sum += err[k] * err[k];
        c++;
      }
    return c ? Math.sqrt(sum / c) : NaN;
  };
  const rms = rmsOf(() => true);
  const trackRms = rmsOf((k) => !inShot[k]);
  const shotRms = rmsOf((k) => inShot[k]);
  // Chatter: high-frequency power noise (power minus its ~50 ms moving average).
  let ch = 0,
    cc = 0;
  for (let k = 0; k < n; k++) {
    let sum = 0,
      c = 0;
    for (let m = k; m >= 0 && s[k].t - s[m].t < 0.05; m--) {
      sum += s[m].power;
      c++;
    }
    const d = s[k].power - sum / c;
    ch += d * d;
    cc++;
  }
  const chatter = cc ? Math.sqrt(ch / cc) : 0;
  const mean = (a: number[]) =>
    a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN;
  const score =
    (isNaN(shotRms) ? trackRms : 0.5 * trackRms + 0.5 * shotRms) +
    300 * chatter;
  return {
    n,
    secs,
    rms,
    trackRms,
    shotRms,
    shots: shotIdx.length,
    dip: mean(dips),
    recover: mean(recovers),
    chatter,
    score,
    hz: secs ? n / secs : 0,
  };
}

// Least-squares fit of the plant: volts = kd*sign(v) + kv*v + ka*accel, using samples away from shots.
function fitFeedForward(
  s: Sample[],
): { kd: number; kv: number; ka: number; r2: number } | null {
  const rows: number[][] = [];
  const y: number[] = [];
  for (let k = 0; k < s.length; k++) {
    // smoothed acceleration over +-50 ms
    let a = k,
      b = k;
    while (a > 0 && s[k].t - s[a - 1].t <= 0.05) a--;
    while (b < s.length - 1 && s[b + 1].t - s[k].t <= 0.05) b++;
    if (s[b].t - s[a].t < 0.06) continue;
    const acc = (s[b].vel - s[a].vel) / (s[b].t - s[a].t);
    if (Math.abs(s[k].target - s[k].vel) > 40) continue; // skip shot dips / spin-up
    if (Math.abs(s[k].power) >= 0.999) continue; // saturated: not informative
    rows.push([Math.sign(s[k].vel), s[k].vel, acc]);
    y.push(s[k].power * s[k].volt);
  }
  if (rows.length < 200) return null;
  // normal equations 3x3
  const A = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  const B = [0, 0, 0];
  for (let r = 0; r < rows.length; r++)
    for (let i = 0; i < 3; i++) {
      B[i] += rows[r][i] * y[r];
      for (let j = 0; j < 3; j++) A[i][j] += rows[r][i] * rows[r][j];
    }
  const x = solve3(A, B);
  if (!x) return null;
  const meanY = y.reduce((p, q) => p + q, 0) / y.length;
  let ssRes = 0,
    ssTot = 0;
  for (let r = 0; r < rows.length; r++) {
    const pred = rows[r][0] * x[0] + rows[r][1] * x[1] + rows[r][2] * x[2];
    ssRes += (y[r] - pred) ** 2;
    ssTot += (y[r] - meanY) ** 2;
  }
  return { kd: x[0], kv: x[1], ka: x[2], r2: 1 - ssRes / ssTot };
}
function solve3(A: number[][], B: number[]): number[] | null {
  const M = A.map((r, i) => [...r, B[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c;
    for (let r = c + 1; r < 3; r++)
      if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < 3; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k];
    }
  }
  return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
}

// ---------------------------------------------------------------- live display
let label = '';
function liveLine(): string {
  const s = samples[samples.length - 1];
  if (!s) return 'waiting for data...';
  const hz = lastStatus ? lastStatus.loopHz.toFixed(0) : '?';
  return `${label.padEnd(28)} t=${s.t.toFixed(1).padStart(6)}s  target ${s.target.toFixed(0).padStart(5)}  vel ${s.vel
    .toFixed(0)
    .padStart(
      5,
    )}  err ${(s.target - s.vel).toFixed(0).padStart(5)}  pwr ${s.power.toFixed(2)}  ${s.volt.toFixed(1)}V  ${hz}Hz`;
}
let ticker: ReturnType<typeof setInterval> | null = null;
function startTicker(): void {
  ticker = setInterval(
    () => process.stdout.write('\r' + liveLine() + '   '),
    250,
  );
}

const fmtG = (c: Partial<Config>) =>
  `P=${c.p?.toPrecision(3)} I=${c.i?.toPrecision(3)} D=${c.d?.toPrecision(3)} kV=${c.kv?.toPrecision(3)} kS(dyn)=${c.kd?.toPrecision(3)}`;
const fmtM = (m: Metrics) =>
  `score ${m.score.toFixed(1).padStart(6)} | rms ${m.rms.toFixed(1)} track ${m.trackRms.toFixed(1)} shot ${isNaN(m.shotRms) ? '  -  ' : m.shotRms.toFixed(1)} | shots ${m.shots} dip ${isNaN(m.dip) ? '-' : m.dip.toFixed(0)} rec ${isNaN(m.recover) ? '-' : m.recover.toFixed(2) + 's'} | chatter ${m.chatter.toFixed(3)} | ${m.hz.toFixed(0)}Hz`;

// ---------------------------------------------------------------- trials
async function trial(
  name: string,
  gains: Partial<Config>,
  secs = trialSecs,
): Promise<Metrics> {
  label = name;
  const st = await call('/config', gains as Record<string, number>);
  const ver = st.cfg;
  // wait for samples under the new config, skip SETTLE seconds, then gather `secs` seconds
  let t0 = NaN;
  for (;;) {
    await Bun.sleep(200);
    const first = samples.find((x) => x.cfg >= ver);
    if (!first) continue;
    if (isNaN(t0)) t0 = first.t;
    const last = samples[samples.length - 1];
    if (last.t - t0 >= SETTLE + secs) break;
  }
  const win = samples.filter((x) => x.cfg === ver && x.t >= t0 + SETTLE);
  const m = analyze(win);
  process.stdout.write('\r' + ' '.repeat(150) + '\r');
  console.log(
    `${name.padEnd(24)} ${fmtG({ ...st.config })}\n${''.padEnd(24)} ${fmtM(m)}`,
  );
  return m;
}

function configFromOpts(): Record<string, string | number | boolean> {
  const q: Record<string, string | number | boolean> = {};
  for (const k of [
    'min',
    'max',
    'period',
    'p',
    'i',
    'd',
    'kv',
    'ks',
    'kd',
    'hood',
    'turret',
  ])
    if (opts[k] !== undefined) q[k] = opts[k];
  q.feed = !flags.has('no-feed');
  if (flags.has('force')) q.force = 1;
  return q;
}

async function begin(): Promise<Status> {
  forwardUsb();
  startLog();
  const st = await call('/start', configFromOpts());
  console.log(
    `sweeping ${st.config.min}..${st.config.max} t/s, ${st.config.period}s period; feed ${st.config.feed ? 'ON (gate open) - feed balls continuously' : 'off'}`,
  );
  console.log(`start gains: ${fmtG(st.config)}`);
  console.log(`log: ${logFile}\nCtrl+C stops the launcher.\n`);
  void collector();
  return st;
}

async function runCmd(): Promise<void> {
  const st = await begin();
  label = 'run';
  startTicker();
  const end = opts.for ? Date.now() + Number(opts.for) * 1000 : Infinity;
  let lastReport = Date.now();
  let from = 0;
  while (Date.now() < end) {
    await Bun.sleep(250);
    if (Date.now() - lastReport >= st.config.period * 2000) {
      lastReport = Date.now();
      const win = samples.slice(from).filter((x) => x.t > SETTLE);
      from = samples.length;
      process.stdout.write('\r' + ' '.repeat(150) + '\r');
      console.log(`last ${st.config.period * 2}s: ${fmtM(analyze(win))}`);
    }
  }
  await shutdown(0);
}

async function tuneCmd(): Promise<void> {
  const st = await begin();
  startTicker();
  const team = st.config;
  type Cand = { g: Config; m: Metrics; name: string };
  const results: Cand[] = [];
  const run = async (name: string, g: Config): Promise<Cand> => {
    const m = await trial(name, g);
    const c = { g: { ...g }, m, name };
    results.push(c);
    return c;
  };

  // 1. baseline with the team's current gains
  let best = await run('baseline (team gains)', { ...team });

  // 2. feed-forward from data (the plant doesn't care what controller was running)
  const ff = fitFeedForward(samples.filter((x) => x.t > SETTLE + 1));
  if (ff) {
    console.log(
      `\nplant fit: volts = ${ff.kd.toFixed(3)}*sign + ${ff.kv.toPrecision(4)}*vel + ${ff.ka.toPrecision(3)}*accel  (R^2 ${ff.r2.toFixed(3)})`,
    );
    const sane =
      ff.kv > team.kv * 0.5 &&
      ff.kv < team.kv * 2 &&
      ff.kd > -0.5 &&
      ff.kd < 3 &&
      ff.r2 > 0.5;
    if (sane) {
      const c = await run('fitted feed-forward', {
        ...best.g,
        kv: ff.kv,
        kd: Math.max(0, ff.kd),
      });
      if (c.m.score < best.m.score) best = c;
    } else {
      console.log('fit looks off - keeping the team feed-forward');
    }
  } else {
    console.log(
      '\nnot enough clean data to fit feed-forward - keeping the team values',
    );
  }

  // 3. coordinate search, multiplicative steps; must beat best by 3% to count (shots add noise)
  const better = (a: Metrics, b: Metrics) => a.score < b.score * 0.97;
  const search = async (key: 'p' | 'i' | 'd', seed: number) => {
    const step = 1.6;
    let cur = best.g[key] > 0 ? best.g[key] : seed;
    if (best.g[key] === 0) {
      const c = await run(`${key} seed`, { ...best.g, [key]: seed });
      if (better(c.m, best.m)) best = c;
      else return;
      cur = seed;
    }
    for (const dir of [step, 1 / step]) {
      let moved = false;
      for (let k = 0; k < 4; k++) {
        const v = cur * dir;
        const c = await run(
          `${key} ${dir > 1 ? 'up' : 'down'} -> ${v.toPrecision(3)}`,
          { ...best.g, [key]: v },
        );
        if (better(c.m, best.m)) {
          best = c;
          cur = v;
          moved = true;
        } else break;
      }
      if (moved) return;
    }
  };
  console.log('');
  await search('p', 0.002);
  await search('i', best.g.p * 0.05);
  await search('d', best.g.p * 0.01);
  await search('p', 0.002);

  // 4. confirm: best vs team, back to back
  console.log('\nconfirming...');
  const confirmBest = await run('confirm: best', { ...best.g });
  const confirmTeam = await run('confirm: team gains', { ...team });
  collecting = false;
  if (ticker) clearInterval(ticker);
  await call('/stop').catch(() => {});

  const g = best.g;
  console.log('\n=============== result ===============');
  console.log(
    `team gains : ${fmtG(team)}\n             ${fmtM(confirmTeam.m)}`,
  );
  console.log(`best gains : ${fmtG(g)}\n             ${fmtM(confirmBest.m)}`);
  if (ff && Math.abs(ff.ka) > 0)
    console.log(
      `\nnote: plant has kA ~ ${ff.ka.toPrecision(3)} V/(t/s^2); TurretSubsystem's feed-forward has no accel term.`,
    );
  console.log('\nTurretSubsystem.java:');
  console.log(
    `    public static PIDFCoefficients launchPID = new PIDFCoefficients(${g.p.toPrecision(3)}, ${g.i.toPrecision(3)}, ${g.d.toPrecision(3)}, 0);`,
  );
  console.log(
    `    public static double kDynamicFriction = ${g.kd.toPrecision(3)};`,
  );
  console.log(
    `    public static double kVelocityConstant = ${g.kv.toPrecision(4)};`,
  );
  console.log(`\nlog: ${logFile}`);
  const summary = logFile.replace(/\.csv$/, '-trials.json');
  writeFileSync(summary, JSON.stringify(results, null, 2));
  console.log(`trials: ${summary}`);
  process.exit(0);
}

// ---------------------------------------------------------------- main
try {
  switch (cmd) {
    case 'run':
      await runCmd();
      break;
    case 'tune':
      await tuneCmd();
      break;
    case 'stop':
      forwardUsb();
      await call('/stop');
      console.log('stopped');
      break;
    case 'status':
      forwardUsb();
      console.log(await call('/status'));
      break;
    default:
      console.log(
        'usage: bun run launcher run|tune|stop|status [--min --max --period --p --i --d --kv --kd --trial --no-feed --force --host]',
      );
  }
} catch (e) {
  console.error(e instanceof Error ? e.message : e);
  await shutdown(1);
}
