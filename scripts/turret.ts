// Turret sweep experiment from the PC. Talks to TurretSweep.java on the robot.
//
//   bun run turret [--period 4] [--min 0.04 --max 0.9] [--for 30] [--force] [--host 192.168.43.1]
//
// Both turret servos follow a sine between the limits (default: TurretSubsystem's TURRET_MIN/MAX).
// Ctrl+C or --for ends it; the robot also stops by itself if this script goes quiet for 1.5 s.

import { existsSync } from 'node:fs';
import { homedir } from 'node:os';
import { join } from 'node:path';

const argv = process.argv.slice(2);
const opts: Record<string, string> = {};
const flags = new Set<string>();
for (let k = 0; k < argv.length; k++) {
  if (!argv[k].startsWith('--')) continue;
  const name = argv[k].slice(2);
  const next = argv[k + 1];
  if (next !== undefined && !next.startsWith('--')) {
    opts[name] = next;
    k++;
  } else flags.add(name);
}
const base = `http://${opts.host ?? '127.0.0.1'}:8080/turret`;

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

type Status = {
  running: boolean;
  active: string;
  t: number;
  pos: number;
  min: number;
  max: number;
  period: number;
  error?: string;
};
async function call(path: string): Promise<Status> {
  let res: Response;
  try {
    res = await fetch(base + path, { signal: AbortSignal.timeout(5000) });
  } catch (e) {
    throw new Error(`can't reach the robot at ${base} (${e})`);
  }
  if (res.status === 404)
    throw new Error(
      'robot has no /turret endpoints - install the current build',
    );
  const body = (await res.json()) as Status;
  if (body.error) throw new Error(body.error);
  return body;
}

let stopping = false;
async function stop(code: number): Promise<never> {
  if (!stopping) {
    stopping = true;
    await call('/stop').catch(() => {});
    console.log('\nturret sweep stopped');
  }
  process.exit(code);
}
process.on('SIGINT', () => void stop(0));

try {
  if (!opts.host) {
    const r = Bun.spawnSync([adbPath(), 'forward', 'tcp:8080', 'tcp:8080']);
    if (r.exitCode !== 0)
      throw new Error('adb forward failed - is the robot plugged in over USB?');
  }
  const q = new URLSearchParams();
  for (const k of ['period', 'min', 'max']) if (opts[k]) q.set(k, opts[k]);
  if (flags.has('force')) q.set('force', '1');
  const st = await call('/start?' + q.toString());
  console.log(
    `turret sweeping ${st.min}..${st.max} (servo units), ${st.period}s period. Ctrl+C stops.`,
  );
  const end = opts.for ? Date.now() + Number(opts.for) * 1000 : Infinity;
  while (Date.now() < end) {
    await Bun.sleep(250);
    const s = await call('/status');
    if (!s.running) throw new Error('turret sweep stopped on the robot');
    process.stdout.write(
      `\rt=${s.t.toFixed(1).padStart(6)}s  position ${s.pos.toFixed(3)}   `,
    );
  }
  await stop(0);
} catch (e) {
  console.error(e instanceof Error ? e.message : e);
  await stop(1);
}
