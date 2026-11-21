// Sokoban solver for sokoban-solver-01-run-20261121.
//
// Strategy: the generator builds each level as a 1-wide hallway plus 1-4
// rooms (6x6 slots, one door each), 3-4 boxes per room, and a box pushed
// through a door can never come back, so every box stays in its room and
// the level's push-optimal count is the sum of the rooms' optima. This
// solver decomposes the level into rooms, solves each room independently
// with a push-optimal A* (admissible min-assignment heuristic over
// box-ignoring BFS distances, dead-square + 2x2-block pruning, transposition
// table), then stitches the rooms together with plain walks through the
// hallway. If decomposition ever fails on an unseen shape it falls back to
// a global push-A* with the same heuristic and a fixed expansion budget.
// All budgets are fixed counts of work, never the clock, so two runs are
// byte-identical. No Date.now, no Math.random, deterministic tie-breaking.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed work budgets (counts of expansions, never time).
const ROOM_EXPANSION_CAP = 800_000;
const GLOBAL_EXPANSION_CAP = 250_000;

const DIRS: [number, number, string][] = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

type Level = {
  w: number;
  h: number;
  wall: boolean[];
  goals: number[];
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#';
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS distances on floor cells ignoring boxes, from one source cell.
// Returns an array of length w*h with -1 for unreachable / wall cells.
function bfsDist(w: number, h: number, floor: boolean[], src: number): number[] {
  const dist: number[] = new Array(w * h).fill(-1);
  if (!floor[src]) return dist;
  dist[src] = 0;
  const q: number[] = [src];
  let qi = 0;
  while (qi < q.length) {
    const c = q[qi++]!;
    const x = c % w;
    const y = Math.floor(c / w);
    for (const [dx, dy] of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!floor[n] || dist[n] !== -1) continue;
      dist[n] = dist[c]! + 1;
      q.push(n);
    }
  }
  return dist;
}

// BFS walk path (player only, no pushes) from src to dst over cells in
// `open` (floor minus boxes). Returns the move string or null.
function walkPath(
  w: number,
  h: number,
  open: boolean[],
  src: number,
  dst: number,
): string | null {
  if (src === dst) return '';
  const prev: number[] = new Array(w * h).fill(-1);
  const pmove: string[] = new Array(w * h).fill('');
  prev[src] = src;
  const q: number[] = [src];
  let qi = 0;
  while (qi < q.length) {
    const c = q[qi++]!;
    const x = c % w;
    const y = Math.floor(c / w);
    for (const [dx, dy, letter] of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!open[n] || prev[n] !== -1) continue;
      prev[n] = c;
      pmove[n] = letter;
      if (n === dst) {
        let s = '';
        for (let t = n; t !== src; t = prev[t]!) s = pmove[t]! + s;
        return s;
      }
      q.push(n);
    }
  }
  return null;
}

// One BFS from src over `open` cells, keeping parent pointers so any
// walk path can be read back without a second search. Returns the reached
// set plus the parent / move arrays.
function bfsWalk(
  w: number,
  h: number,
  open: boolean[],
  src: number,
): { reach: boolean[]; prev: number[]; pmove: string[] } {
  const reach: boolean[] = new Array(w * h).fill(false);
  const prev: number[] = new Array(w * h).fill(-1);
  const pmove: string[] = new Array(w * h).fill('');
  if (!open[src]) return { reach, prev, pmove };
  reach[src] = true;
  prev[src] = src;
  const q: number[] = [src];
  let qi = 0;
  while (qi < q.length) {
    const c = q[qi++]!;
    const x = c % w;
    const y = Math.floor(c / w);
    for (const [dx, dy, letter] of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!open[n] || reach[n]) continue;
      reach[n] = true;
      prev[n] = c;
      pmove[n] = letter;
      q.push(n);
    }
  }
  return { reach, prev, pmove };
}

// Walk path from src to dst from one bfsWalk result, or null.
function pathFromBfs(prev: number[], pmove: string[], src: number, dst: number): string | null {
  if (dst === src) return '';
  if (prev[dst] === -1) return null;
  let s = '';
  for (let t = dst; t !== src; t = prev[t]!) s = pmove[t]! + s;
  return s;
}

// Cells from which a box can still be pushed onto a goal, ignoring other
// boxes (reverse-push fixpoint from the goals). A box on any other floor
// cell is dead.
function liveCells(w: number, h: number, floor: boolean[], goals: number[]): boolean[] {
  const live: boolean[] = new Array(w * h).fill(false);
  for (const g of goals) live[g] = true;
  let changed = true;
  while (changed) {
    changed = false;
    for (let c = 0; c < w * h; c++) {
      if (!floor[c] || live[c]) continue;
      const x = c % w;
      const y = Math.floor(c / w);
      for (const [dx, dy] of DIRS) {
        const ax = x + dx;
        const ay = y + dy;
        const bx = x - dx;
        const by = y - dy;
        if (ax < 0 || ay < 0 || ax >= w || ay >= h) continue;
        if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
        const a = ay * w + ax;
        const b = by * w + bx;
        if (live[a] && floor[b]) {
          live[c] = true;
          changed = true;
          break;
        }
      }
    }
  }
  return live;
}

// Push-optimal A* over pushes. `floor` is the region the player may walk
// (room floor, or whole level for the global fallback). Returns the full
// player-move string, or null if unsolved within the expansion cap.
function pushAStar(
  w: number,
  h: number,
  floor: boolean[],
  boxes0: number[],
  goals: number[],
  player0: number,
  cap: number,
): string | null {
  const n = boxes0.length;
  const goalSet = new Set(goals);
  const live = liveCells(w, h, floor, goals);
  for (const b of boxes0) if (!goalSet.has(b) && !live[b]) return null;

  // All-pairs BFS distances ignoring boxes, between floor cells.
  const floorCells: number[] = [];
  for (let c = 0; c < w * h; c++) if (floor[c]) floorCells.push(c);
  const distFrom = new Map<number, number[]>();
  for (const c of floorCells) distFrom.set(c, bfsDist(w, h, floor, c));

  // Min-assignment heuristic: exhaustive matching for <= 4 boxes (rooms);
  // for the global fallback (up to 16 boxes) use the sum of each box's
  // nearest-goal distance, which is still admissible.
  const heuristic = (bs: number[]): number => {
    if (n <= 4) {
      let best = Infinity;
      const used: boolean[] = new Array(n).fill(false);
      const assign = (bi: number, acc: number): void => {
        if (acc >= best) return;
        if (bi === n) {
          best = acc;
          return;
        }
        const d = distFrom.get(bs[bi]!)!;
        for (let gi = 0; gi < n; gi++) {
          if (used[gi]) continue;
          const dd = d[goals[gi]!]!;
          if (dd < 0) continue;
          used[gi] = true;
          assign(bi + 1, acc + dd);
          used[gi] = false;
        }
      };
      assign(0, 0);
      return best;
    }
    let sum = 0;
    for (const b of bs) {
      const d = distFrom.get(b)!;
      let m = Infinity;
      for (const g of goals) {
        const dd = d[g]!;
        if (dd >= 0 && dd < m) m = dd;
      }
      if (m === Infinity) return Infinity;
      sum += m;
    }
    return sum;
  };

  const isDead = (bs: number[]): boolean => {
    for (const b of bs) if (!goalSet.has(b) && !live[b]) return true;
    // 2x2 frozen block: all four cells wall-or-box, >= 1 box, no goal.
    const boxSet = new Set(bs);
    for (let y = 0; y + 1 < h; y++) {
      for (let x = 0; x + 1 < w; x++) {
        const cells = [y * w + x, y * w + x + 1, (y + 1) * w + x, (y + 1) * w + x + 1];
        let boxes = 0;
        let blocked = true;
        for (const c of cells) {
          if (goalSet.has(c)) {
            blocked = false;
            break;
          }
          if (boxSet.has(c)) boxes++;
          else if (floor[c]) {
            blocked = false;
            break;
          }
        }
        if (blocked && boxes > 0) return true;
      }
    }
    return false;
  };

  // Transposition table, keyed by (canonical player region, boxes): the
  // player can stand anywhere in its reachable region at zero push cost,
  // so states that differ only in the player's cell within one region are
  // the same state. The region signature is computed when a state is
  // popped, which is also when its single BFS runs, so canonicalisation
  // costs nothing extra.
  const keyOf = (rep: string, bs: number[]): string => rep + ':' + bs.join(',');
  const h0 = heuristic(boxes0);
  if (h0 === Infinity) return null;

  // Binary heap on [f, h, seq]; seq breaks ties deterministically (FIFO).
  const heap: [number, number, number, number][] = [];
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const sp: number[] = [player0];
  const sb: number[][] = [boxes0.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const seg: string[] = [''];
  const best = new Map<string, number>();
  let seq = 0;
  heapPush([h0, h0, seq++, 0]);

  let expanded = 0;
  while (heap.length > 0 && expanded < cap) {
    const [, , , s] = heapPop();
    const bs = sb[s]!;
    const p = sp[s]!;
    const g = sg[s]!;
    if (bs.every((b) => goalSet.has(b))) {
      let moves = '';
      for (let t = s; t > 0; t = parent[t]!) moves = seg[t]! + moves;
      return moves;
    }
    // Open cells for the player: floor minus boxes.
    const open: boolean[] = floor.slice();
    for (const b of bs) open[b] = false;
    open[p] = true;
    const { reach, prev, pmove } = bfsWalk(w, h, open, p);
    // Canonical region signature: the reachable set itself, as a bit
    // string over the floor cells. (The lowest reachable cell alone does
    // not identify a region: two different regions can share it.)
    let rep = '';
    for (const c of floorCells) rep += reach[c] ? '1' : '0';
    const k = keyOf(rep, bs);
    const seenG = best.get(k);
    if (seenG !== undefined && seenG <= g) continue;
    best.set(k, g);
    expanded++;
    const boxSet = new Set(bs);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi]!;
      const bx = b % w;
      const by = (b - bx) / w;
      for (const [dx, dy, letter] of DIRS) {
        const behindX = bx - dx;
        const behindY = by - dy;
        const destX = bx + dx;
        const destY = by + dy;
        if (behindX < 0 || behindY < 0 || behindX >= w || behindY >= h) continue;
        if (destX < 0 || destY < 0 || destX >= w || destY >= h) continue;
        const behind = behindY * w + behindX;
        const dest = destY * w + destX;
        if (!reach[behind]) continue;
        if (!floor[dest] || boxSet.has(dest)) continue;
        const walk = pathFromBfs(prev, pmove, p, behind);
        if (walk === null) continue;
        const nb = bs.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (isDead(nb)) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        // Player ends on the box's old cell after the push.
        const np = b;
        const t = sp.length;
        sp.push(np);
        sb.push(nb);
        sg.push(g + 1);
        parent.push(s);
        seg.push(walk + letter);
        heapPush([g + 1 + hh, hh, seq++, t]);
      }
    }
  }
  return null;
}

// Decompose a level into rooms. Returns null when the shape does not fit
// the hallway-plus-rooms generator (caller falls back to global A*).
function decompose(lv: Level): { floor: boolean[]; boxes: number[]; goals: number[]; door: number; entry: number }[] | null {
  const { w, h, wall } = lv;
  const hallRow = Math.floor(lv.player / w);
  const floor: boolean[] = wall.map((v) => !v);
  const seen: boolean[] = new Array(w * h).fill(false);
  const rooms: { floor: boolean[]; boxes: number[]; goals: number[]; door: number; entry: number }[] = [];
  const boxRoom = new Map<number, number>();
  const goalRoom = new Map<number, number>();
  for (let c = 0; c < w * h; c++) {
    if (!floor[c] || Math.floor(c / w) === hallRow || seen[c]) continue;
    // Flood-fill one component (never crossing the hallway row).
    const comp: number[] = [];
    const q: number[] = [c];
    seen[c] = true;
    let qi = 0;
    while (qi < q.length) {
      const cur = q[qi++]!;
      comp.push(cur);
      const x = cur % w;
      const y = (cur - x) / w;
      for (const [dx, dy] of DIRS) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (ny === hallRow) continue;
        const n = ny * w + nx;
        if (!floor[n] || seen[n]) continue;
        seen[n] = true;
        q.push(n);
      }
    }
    // Door: a component cell adjacent to a hallway-row floor cell. The
    // player enters from that hallway cell (`entry`): the door cell itself
    // may hold a box, which the player pushes inward from the hallway, so
    // the room solve starts with the player on `entry`, not on the door.
    let door = -1;
    let entry = -1;
    for (const cell of comp) {
      const x = cell % w;
      const y = (cell - x) / w;
      for (const [dx, dy] of DIRS) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (ny !== hallRow) continue;
        const n = ny * w + nx;
        if (floor[n]) {
          door = cell;
          entry = n;
          break;
        }
      }
      if (door >= 0) break;
    }
    if (door < 0) return null;
    const rfloor: boolean[] = new Array(w * h).fill(false);
    for (const cell of comp) rfloor[cell] = true;
    const idx = rooms.length;
    rooms.push({ floor: rfloor, boxes: [], goals: [], door, entry });
    for (const cell of comp) {
      if (lv.boxes.includes(cell)) {
        rooms[idx]!.boxes.push(cell);
        boxRoom.set(cell, idx);
      }
      if (lv.goals.includes(cell)) {
        rooms[idx]!.goals.push(cell);
        goalRoom.set(cell, idx);
      }
    }
  }
  // Every box and goal must live in exactly one room; none in the hallway.
  if (boxRoom.size !== lv.boxes.length || goalRoom.size !== lv.goals.length) return null;
  if (rooms.length < 1 || rooms.length > 4) return null;
  for (const r of rooms) {
    if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) return null;
    r.boxes.sort((a, b) => a - b);
    r.goals.sort((a, b) => a - b);
  }
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

function solve(text: string): string {
  const orig = parseLevel(text);
  const lv = parseLevel(text);
  const { w, h, wall } = lv;
  const floorAll: boolean[] = wall.map((v) => !v);
  try {
    const rooms = decompose(lv);
    if (rooms) {
      let moves = '';
      let player = lv.player;
      // Rooms in door order; walk the hallway between them.
      for (const room of rooms) {
        // Walk to the room's entry cell in the hallway; the room solve
        // starts there and walks/pushes in from the door itself.
        const open2: boolean[] = floorAll.slice();
        for (const b of lv.boxes) open2[b] = false;
        open2[player] = true;
        const walk = walkPath(w, h, open2, player, room.entry);
        if (walk === null) return globalFallback(orig);
        moves += walk;
        player = room.entry;
        // The room floor for the solve is the room plus its entry cell.
        const solveFloor = room.floor.slice();
        solveFloor[room.entry] = true;
        const roomMoves = pushAStar(w, h, solveFloor, room.boxes, room.goals, room.entry, ROOM_EXPANSION_CAP);
        if (roomMoves === null) return globalFallback(orig);
        moves += roomMoves;
        player = endPlayer(w, room, roomMoves, room.entry);
        // This room's boxes are now on its goals.
        applyRoomSolved(lv, room);
      }
      if (verifyFresh(orig, moves)) return moves;
      return globalFallback(orig);
    }
  } catch {
    return globalFallback(orig);
  }
  return globalFallback(orig);
}

function applyRoomSolved(lv: Level, room: { boxes: number[]; goals: number[] }): void {
  for (let i = 0; i < lv.boxes.length; i++) {
    const idx = room.boxes.indexOf(lv.boxes[i]!);
    if (idx >= 0) lv.boxes[i] = room.goals[idx]!;
  }
  lv.boxes.sort((a, b) => a - b);
}

// Replay the room's move string starting from the entry cell to find
// where the player ends up (boxes are confined to the room floor).
function endPlayer(
  w: number,
  room: { floor: boolean[]; boxes: number[] },
  moves: string,
  door: number,
): number {
  const boxes = new Set(room.boxes);
  let p = door;
  for (const ch of moves) {
    const d = DIRS.find(([, , l]) => l === ch)!;
    const q = p + d[1] * w + d[0];
    if (boxes.has(q)) {
      const dest = q + d[1] * w + d[0];
      boxes.delete(q);
      boxes.add(dest);
    }
    p = q;
  }
  return p;
}

function globalFallback(lv: Level): string {
  const { w, h, wall } = lv;
  const floorAll: boolean[] = wall.map((v) => !v);
  const sol = pushAStar(w, h, floorAll, lv.boxes, lv.goals, lv.player, GLOBAL_EXPANSION_CAP);
  if (sol !== null && verifyFresh(lv, sol)) return sol;
  return '';
}

// Replay a move string on the original level; true iff every box ends on
// a goal (and no illegal step occurred).
function verifyFresh(lv: Level, moves: string): boolean {
  const boxes = new Set(lv.boxes);
  let p = lv.player;
  for (const ch of moves) {
    const d = DIRS.find(([, , l]) => l === ch);
    if (!d) return false;
    const q = p + d[1] * lv.w + d[0];
    if (lv.wall[q]) return false;
    if (boxes.has(q)) {
      const dest = q + d[1] * lv.w + d[0];
      if (lv.wall[dest] || boxes.has(dest)) return false;
      boxes.delete(q);
      boxes.add(dest);
    }
    p = q;
  }
  const goalSet = new Set(lv.goals);
  for (const b of boxes) if (!goalSet.has(b)) return false;
  return true;
}

for (const id of readdirSync(I).sort()) {
  const text = readFileSync(join(I, id, 'level.txt'), 'utf8');
  let moves = '';
  try {
    moves = solve(text);
    // Safety net: only emit a solution that replays cleanly on the level.
    const lv = parseLevel(text);
    if (moves !== '' && !verifyFresh(lv, moves)) moves = '';
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
