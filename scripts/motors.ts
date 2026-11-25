// Drive the robot's motors from the PC over USB (or robot Wi-Fi), no Driver Station needed.
//
// Talks to PcMotorControl (Sixteen750/.../pc/PcMotorControl.java) through the Robot
// Controller's built-in web server. The first `set` auto-starts that OpMode on the robot.
//
//   bun run motors status                      list motors + live velocity/current
//   bun run motors set launcher=1500           velocity in encoder ticks/sec
//   bun run motors set intake=p0.6 fl=p-0.2    p = raw power (-1..1)
//   bun run motors set launcher=1500 --for 5   run for 5 seconds then stop
//   bun run motors set ... --force             take over even if another OpMode is running
//   bun run motors stop                        zero everything, exit the OpMode
//
// `set` keeps running (printing live readings) until Ctrl+C or --for runs out. The robot
// zeroes every motor if it hears nothing from this script for 1 second.
//
// Over USB this sets up `adb forward tcp:8080 tcp:8080` itself. Over the robot's Wi-Fi use
// --host 192.168.43.1.

import { existsSync } from 'node:fs';
import { homedir } from 'node:os';
import { join } from 'node:path';

type Motor = {
  name: string;
  port: number;
  velocity: number;
  power: number;
  amps: number;
  position: number;
  target: { mode: 'velocity' | 'power'; value: number } | null;
};
type Status =
  | { opmode: 'running'; motors: Motor[] }
  | { opmode: 'idle'; active: string; lastError: string };

const args = process.argv.slice(2);
function flag(name: string): boolean {
  const i = args.indexOf(name);
  if (i < 0) return false;
  args.splice(i, 1);
  return true;
}
function option(name: string): string | undefined {
  const i = args.indexOf(name);
  if (i < 0) return undefined;
  const v = args[i + 1];
  args.splice(i, 2);
  return v;
}

const force = flag('--force');
const forSecs = option('--for');
const host = option('--host');
const base = `http://${host ?? '127.0.0.1'}:8080/pcmotor`;

function adbPath(): string {
  const candidates = [
    process.env.ANDROID_HOME &&
      join(process.env.ANDROID_HOME, 'platform-tools'),
    process.env.LOCALAPPDATA &&
      join(process.env.LOCALAPPDATA, 'Android', 'sdk', 'platform-tools'),
    join(homedir(), 'Library', 'Android', 'sdk', 'platform-tools'),
    join(homedir(), 'Android', 'Sdk', 'platform-tools'),
  ].filter((p): p is string => !!p);
  for (const dir of candidates) {
    for (const exe of ['adb.exe', 'adb']) {
      if (existsSync(join(dir, exe))) return join(dir, exe);
    }
  }
  return 'adb';
}

function forwardUsb(): void {
  if (host) return;
  const r = Bun.spawnSync([adbPath(), 'forward', 'tcp:8080', 'tcp:8080']);
  if (r.exitCode !== 0) {
    console.error(
      'adb forward failed - is the robot plugged in over USB?\n' +
        r.stderr.toString().trim(),
    );
    process.exit(1);
  }
}

async function call(path: string): Promise<Status> {
  let res: Response;
  try {
    res = await fetch(base + path, { signal: AbortSignal.timeout(5000) });
  } catch (e) {
    throw new Error(
      `can't reach the robot at ${base} (${e}). Is the Robot Controller app running?`,
    );
  }
  const body = (await res.json()) as Status & { error?: string };
  if (res.status === 404) {
    throw new Error(
      'the robot code has no /pcmotor endpoints - install the current build first',
    );
  }
  if (body.error) throw new Error(body.error);
  return body;
}

function fmt(s: Status): string {
  if (s.opmode === 'idle') {
    return `PC control idle (active OpMode: ${s.active})`;
  }
  return s.motors
    .map((m) => {
      const tgt = m.target
        ? m.target.mode === 'velocity'
          ? `-> ${m.target.value} t/s`
          : `-> pwr ${m.target.value}`
        : '';
      return `${m.name.padEnd(10)} port ${m.port}  vel ${m.velocity
        .toFixed(0)
        .padStart(6)} t/s  pwr ${m.power.toFixed(2).padStart(5)}  ${m.amps
        .toFixed(2)
        .padStart(5)} A  pos ${String(m.position).padStart(7)}  ${tgt}`;
    })
    .join('\n');
}

function parseTargets(list: string[]): URLSearchParams {
  const q = new URLSearchParams();
  for (const a of list) {
    const m = /^([^=]+)=(p?-?[\d.]+)$/i.exec(a);
    if (!m) {
      console.error(`bad target '${a}' - use name=ticksPerSec or name=pPower`);
      process.exit(2);
    }
    q.set(m[1], m[2].toLowerCase().startsWith('p') ? m[2] : 'v' + m[2]);
  }
  if (force) q.set('force', '1');
  return q;
}

async function main(): Promise<void> {
  const [cmd, ...rest] = args;
  forwardUsb();
  switch (cmd) {
    case 'status':
      console.log(fmt(await call('/status')));
      return;
    case 'stop':
      await call('/stop');
      console.log('stopped');
      return;
    case 'set': {
      if (rest.length === 0) {
        console.error('usage: bun run motors set name=1500 [name=p0.5 ...]');
        process.exit(2);
      }
      const q = parseTargets(rest);
      let s = await call('/set?' + q.toString());
      let stopping = false;
      const stop = async () => {
        if (stopping) return;
        stopping = true;
        await call('/stop').catch(() => {});
        console.log('\nstopped');
        process.exit(0);
      };
      process.on('SIGINT', stop);
      const end = forSecs ? Date.now() + Number(forSecs) * 1000 : Infinity;
      console.log('running - Ctrl+C to stop');
      while (Date.now() < end) {
        const lines = fmt(s);
        process.stdout.write(`\x1b[2J\x1b[H${lines}\n\n(Ctrl+C to stop)\n`);
        await Bun.sleep(200);
        // Resend the targets: keeps the watchdog lease alive and recovers after a hiccup.
        // Without --force this refuses (and we bail) if someone started another OpMode.
        q.delete('force');
        s = await call('/set?' + q.toString());
      }
      await stop();
      return;
    }
    default:
      console.log(
        'usage: bun run motors status | set name=ticksPerSec|name=pPower ... [--for secs] [--force] | stop',
      );
  }
}

main().catch((e) => {
  console.error(String(e instanceof Error ? e.message : e));
  process.exit(1);
});
