import assert from "node:assert/strict";
import test from "node:test";
import { getLocalizationState } from "../src/lib/localizationStatus.ts";

const pose = (cov = 0.1, stamp = 100) => ({ x: 0, y: 0, z: 0, yaw: 0, cov, stamp });

test("missing pose or disconnected websocket cannot authorize navigation", () => {
  assert.equal(getLocalizationState(null, 100, true), "unavailable");
  assert.equal(getLocalizationState(undefined, 100, true), "unavailable");
  assert.equal(getLocalizationState(pose(), 100, false), "unavailable");
});

test("0.99 is the inclusive localization failure boundary", () => {
  assert.equal(getLocalizationState(pose(0), 100, true), "normal");
  assert.equal(getLocalizationState(pose(0.989), 100, true), "normal");
  assert.equal(getLocalizationState(pose(0.99), 100, true), "failed");
  assert.equal(getLocalizationState(pose(0.99000001), 100, true), "failed");
});

test("fresh messages with invalid quality cannot authorize navigation", () => {
  for (const cov of [NaN, Infinity, -1]) {
    assert.equal(getLocalizationState(pose(cov), 100, true), "failed");
  }
});

test("expired pose remains unavailable even if its quality was good", () => {
  assert.equal(getLocalizationState(pose(), 104.999, true), "normal");
  assert.equal(getLocalizationState(pose(), 105, true), "unavailable");
  assert.equal(getLocalizationState(pose(), 100, true, 100), "unavailable");
  assert.equal(getLocalizationState(pose(0.99), 105, true), "unavailable");
});

test("new pose recovers from expiry only when localization quality is healthy", () => {
  assert.equal(getLocalizationState(pose(0.1, 101), 101, true, 100), "normal");
  assert.equal(getLocalizationState(pose(0.99, 101), 101, true, 100), "failed");
  assert.equal(getLocalizationState(pose(0.1, NaN), 101, true), "unavailable");
});
