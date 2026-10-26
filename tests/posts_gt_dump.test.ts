// Developer tool for the post GT: writes the studio output for every fixture case next to RoboDK's output so the two
// can be compared with a diff tool:  POSTS_GT_DUMP=<out dir> [POSTS_GT_POST=KUKA_KRC2] npx vitest run tests/posts_gt_dump.test.ts
import { it } from 'vitest';
import { existsSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { generateForCase } from './posts_gt_util';
import { PostJob } from '../src/posts/job';

const OUT = process.env.POSTS_GT_DUMP ?? '';
const CASES = path.resolve(__dirname, 'fixtures/posts_gt/cases');

it.skipIf(!OUT)('dump studio output for the post GT cases', () => {
  const only = process.env.POSTS_GT_POST ?? '';
  for (const c of readdirSync(CASES)) {
    const jp = path.join(CASES, c, 'job.json');
    if (!existsSync(jp)) continue;
    const job = JSON.parse(readFileSync(jp, 'utf8')) as PostJob;
    for (const post of job.posts ?? []) {
      if (only && !only.split(',').includes(post)) continue;
      const dir = path.join(OUT, c, post);
      mkdirSync(dir, { recursive: true });
      try {
        for (const f of generateForCase({ job, post })) {
          const p = path.join(dir, f.name);
          mkdirSync(path.dirname(p), { recursive: true });
          writeFileSync(p, f.encoding === 'base64' ? Buffer.from(f.content, 'base64') : f.content);
        }
      } catch (e: any) { writeFileSync(path.join(dir, '_ERROR.txt'), String(e?.stack ?? e)); }
    }
  }
});
