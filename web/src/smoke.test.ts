// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from "vitest";
import { HONESTY } from "./main";

describe("dashboard constants", () => {
  it("exports the locked honesty lines", () => {
    expect([...HONESTY]).toEqual([
      "Simulation only",
      "Synthetic fleet scenarios; not a live allocator",
      "Recorded CI output; this page does not run the simulator",
      "Independent travel is not multi-agent path finding; robots may occupy the same edge.",
    ]);
  });
});
