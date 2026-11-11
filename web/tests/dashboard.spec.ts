// SPDX-License-Identifier: Apache-2.0
import { expect, test } from "@playwright/test";

test("dashboard honesty labels", async ({ page }) => {
  await page.goto("./", { waitUntil: "networkidle" });
  await expect(page).toHaveTitle(/simulation only/i);
  await expect(page.getByText("Simulation only")).toBeVisible();
  await expect(page.getByText("Synthetic fleet scenarios; not a live allocator")).toBeVisible();
  await expect(
    page.getByText("Recorded CI output; this page does not run the simulator"),
  ).toBeVisible();
  await expect(
    page.getByText(
      "Independent travel is not multi-agent path finding; robots may occupy the same edge.",
    ),
  ).toBeVisible();
});

test("missing report error state", async ({ page }) => {
  await page.route("**/report.json", (route) => route.abort());
  await page.goto("./", { waitUntil: "networkidle" });
  await expect(page.locator("#error-state")).toBeVisible();
  await expect(page.locator("#sample-banner")).toHaveCount(0);
});

test("no sample banner when sample is false", async ({ page }) => {
  await page.route("**/report.json", async (route) => {
    await route.fulfill({
      contentType: "application/json",
      body: JSON.stringify({
        sample: false,
        git_sha: "0123456789abcdef0123456789abcdef01234567",
        rows: [],
      }),
    });
  });
  await page.goto("./", { waitUntil: "networkidle" });
  await expect(page.locator("#sample-banner")).toHaveCount(0);
  await expect(page.getByText("git_sha: 0123456789abcdef0123456789abcdef01234567")).toBeVisible();
});
