// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "tests",
  timeout: 60_000,
  use: {
    baseURL: "http://127.0.0.1:4173/multi-robot-task-allocation/",
    headless: true,
  },
  webServer: {
    command:
      "npx vite build --base /multi-robot-task-allocation/ && npx vite preview --host 127.0.0.1 --port 4173",
    url: "http://127.0.0.1:4173/multi-robot-task-allocation/",
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
});
