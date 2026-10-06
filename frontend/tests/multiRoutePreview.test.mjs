import assert from "node:assert/strict";
import test from "node:test";
import { MultiRoutePreview } from "../src/lib/multiRoutePreview.ts";

const point = (x) => ({ x, y: 0 });
const goals = (...xs) => xs.map(point);
const route = (start, goal) => [
  { ...start, z: 0 }, { ...goal, z: 0 },
];

function fixture() {
  const preview = new MultiRoutePreview();
  preview.setOrigin(point(0));
  const calls = [];
  const load = async (start, goal) => {
    calls.push([start.x, goal.x]);
    return route(start, goal);
  };
  const plan = async (points) => {
    preview.retain(points);
    await preview.plan(points, load, () => true, () => {});
  };
  return { preview, calls, load, plan };
}

test("appending a goal keeps the existing route and only requests the new segment", async () => {
  const { preview, calls, plan } = fixture();
  await plan(goals(1, 2));
  const previous = preview.cachedRoute(goals(1, 2));
  preview.setOrigin(point(0.01)); // odom drift must not invalidate the preview origin
  preview.retain(goals(1, 2, 3));
  assert.deepEqual(preview.cachedRoute(goals(1, 2, 3)), previous);
  await plan(goals(1, 2, 3));
  assert.deepEqual(calls, [[0, 1], [1, 2], [2, 3]]);
  const next = preview.cachedRoute(goals(1, 2, 3));
  previous.forEach((p, i) => assert.strictEqual(next[i], p));
});

test("deleting a middle goal only replans its connecting segment", async () => {
  const { preview, calls, plan } = fixture();
  await plan(goals(1, 2, 3, 4));
  await plan(goals(1, 3, 4));
  assert.deepEqual(calls, [[0, 1], [1, 2], [2, 3], [3, 4], [1, 3]]);
  assert.deepEqual(preview.cachedRoute(goals(1, 3, 4)).map((p) => p.x), [0, 1, 3, 4]);
});

test("deleting the last goal does not issue planning requests", async () => {
  const { preview, calls, plan } = fixture();
  await plan(goals(1, 2, 3));
  await plan(goals(1, 2));
  assert.equal(calls.length, 3);
  assert.deepEqual(preview.cachedRoute(goals(1, 2)).map((p) => p.x), [0, 1, 2]);
});

test("rapid clicks share in-flight prefix planning and ignore superseded results", async () => {
  const { preview, calls, load } = fixture();
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const blockedLoad = async (start, goal) => {
    if (start.x === 0) await gate;
    return load(start, goal);
  };
  let version = 1;
  const updates = [];
  const first = preview.plan(goals(1), blockedLoad, () => version === 1, () => assert.fail("stale update"));
  version = 2;
  const second = preview.plan(goals(1, 2), blockedLoad, () => version === 2, (points) => updates.push(points));
  release();
  await Promise.all([first, second]);
  assert.deepEqual(calls, [[0, 1], [1, 2]]);
  assert.deepEqual(updates.at(-1).map((p) => p.x), [0, 1, 2]);
});

test("a failed new segment preserves the prefix and can be retried", async () => {
  const { preview, calls, load, plan } = fixture();
  await plan(goals(1));
  await assert.rejects(preview.plan(goals(1, 2), async (start, goal) => {
    if (goal.x === 2) throw new Error("no path");
    return load(start, goal);
  }, () => true, () => {}), /no path/);
  assert.deepEqual(preview.cachedRoute(goals(1, 2)).map((p) => p.x), [0, 1]);
  await plan(goals(1, 2));
  assert.deepEqual(calls, [[0, 1], [1, 2]]);
});

test("clearing while planning prevents a late result from restoring the preview", async () => {
  const { preview, load } = fixture();
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  let current = true;
  const pending = preview.plan(goals(1), async (start, goal) => {
    await gate;
    return load(start, goal);
  }, () => current, () => assert.fail("cleared preview restored"));
  current = false;
  preview.clear();
  release();
  await pending;
  preview.setOrigin(point(10));
  assert.deepEqual(preview.cachedRoute(goals(1)), []);
});

test("starting navigation with an empty dispatch route keeps the full preview", async () => {
  const { preview, plan } = fixture();
  await plan(goals(1, 2, 3));
  const previous = preview.cachedRoute(goals(1, 2, 3));
  assert.equal(preview.updateDispatched(goals(1, 2, 3), 0, []), null);
  assert.deepEqual(preview.cachedRoute(goals(1, 2, 3)), previous);
});

test("dispatch replaces only the active segment and preserves completed and future segments", async () => {
  const { preview, plan } = fixture();
  await plan(goals(1, 2, 3));
  const previous = preview.cachedRoute(goals(1, 2, 3));
  const first = [{ x: 0.1, y: 0, z: 0 }, { x: 0.5, y: 0.2, z: 0 }, { x: 1, y: 0, z: 0 }];
  const afterFirst = preview.updateDispatched(goals(1, 2, 3), 0, first);
  assert.deepEqual(afterFirst.slice(0, 3), first);
  assert.strictEqual(afterFirst[3], previous[2]);
  assert.strictEqual(afterFirst[4], previous[3]);
  assert.equal(preview.updateDispatched(goals(1, 2, 3), 1, []), null);
  assert.deepEqual(preview.cachedRoute(goals(1, 2, 3)), afterFirst);
  const second = [{ x: 1, y: 0, z: 0 }, { x: 1.5, y: 0.3, z: 0 }, { x: 2, y: 0, z: 0 }];
  const afterSecond = preview.updateDispatched(goals(1, 2, 3), 1, second);
  first.forEach((p, i) => assert.strictEqual(afterSecond[i], p));
  assert.deepEqual(afterSecond.slice(3, 5), second.slice(1));
  assert.strictEqual(afterSecond.at(-1), previous.at(-1));
});

test("repeated status messages with identical routes do not update the rendered route", async () => {
  const { preview, plan } = fixture();
  await plan(goals(1, 2));
  const active = [{ x: 0.1, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }];
  assert.ok(preview.updateDispatched(goals(1, 2), 0, active));
  assert.equal(preview.updateDispatched(goals(1, 2), 0, active.map((p) => ({ ...p }))), null);
});

test("reconnecting without preview cache still displays the current dispatched segment", () => {
  const preview = new MultiRoutePreview();
  const active = route(point(1), point(2));
  assert.deepEqual(preview.updateDispatched(goals(1, 2, 3), 1, active), active);
});
