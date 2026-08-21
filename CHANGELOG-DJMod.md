# B747-400 DJMod — changes against the upstream Sparky mod

This repository is the 747-400 "Sparky" mod for the Laminar 747 in X-Plane 12,
with a set of targeted fixes to ILS tracking, autopilot stability and script
safety. The unmodified upstream source is preserved at the git tag
`baseline-sparky`, so any change here can be inspected with:

```bash
git diff baseline-sparky
```

and any individual fix reverted with `git revert <commit>`.

---

## Fixes in this release

### 1. ILS hunting on the localiser and glideslope

**Flight-director command smoothing rebuilt.**
`ap_director_pitch_integral()` and `ap_director_roll_integral()` smoothed the
flight-director command with a 10-sample moving average. Because the sample
interval was itself variable (0.2–2.0 s), the averaging window was **2–10
seconds wide**, putting 1–5 seconds of pure phase lag between a beam deviation
and the control response. Roll was worst: the sample rate slowed to a full
1.0 s per sample once wings-level, so the window stretched to 10 seconds
exactly when established on the localiser.

Both now use a per-frame first-order lag filter driven off `SIM_PERIOD`.
Group delay drops by roughly 5× and no longer varies with framerate.

The pitch sampling gate was deliberately kept — `ap_director_pitch()` is an
incremental controller whose step size is tuned for being called once per
sample interval, so calling it every frame would multiply its integration rate
by 12–120×. Only the smoothing changed.

**Five-second open-loop roll freeze after capture removed.**
`ap_director_roll()` returned a frozen roll command for 5 seconds after
localiser capture, so the entire capture turn was flown open-loop — and those
frozen samples then sat in the smoothing filter for several seconds more. This
was the direct cause of the aircraft starting to hunt the moment it captured.
The genuine signal-dropout hold is retained.

**Localiser bearing average fixed across the 360/0 boundary.**
The two ILS receiver bearings were combined with a plain arithmetic mean, so
nav1 = 359° with nav2 = 001° averaged to **180°** instead of 000°. That 180°
error fed straight into the commanded heading, causing sudden hard excursions
on any approach with a course near north (roughly RWY 34 through 02).

*Files:* `B747.19.xt.hydraulics_override.lua`, `B747.70.xt.autopilot.monitor.lua`,
`B747.70.xt.autopilot.lua`

**Tuning:** if the approach needs further adjustment, `FD_PITCH_TAU` (0.5 s)
and `FD_ROLL_TAU` (0.6 s) near the top of the *FLIGHT DIRECTOR COMMAND
SMOOTHING* block in `B747.19.xt.hydraulics_override.lua` are the first two
knobs to try. Raise them if the response feels twitchy; lower them if the
aircraft still answers the beam slowly.

### 2. Framerate

**Per-frame full garbage collections throttled.**
Six separate scripts called `collectgarbage("collect")` at the top of
`after_physics()` — six full stop-the-world collections *every rendered frame*.
Beyond the direct cost, the resulting frame-time spikes also destabilised the
flight-director loops, which sample on wall-clock intervals. Now throttled to
roughly once every 5 seconds per script.

*Files:* `B747.25.xt.fuel.lua`, `B747.42.xt.EEC.lua`, `B747.90.xt.lighting.lua`,
`B747.61.xt.nd.lua`, `B747.01.xt.fdr.lua`, `B747.70.xt.autopilot.lua`

### 3. Script stability

**`getHeadingDifference()` no longer shadows Lua's built-in `error()`.**
All six copies assigned their working value to a global named `error`. After
the first call, `error()` was a number for the rest of that script's life, so
any later `error(...)` call — including xtlua's own "Unable to find command"
diagnostic — died with *"attempt to call a number value"*, turning a clear
diagnostic into a confusing crash.

**NaN guard in `getWCAforHeading()`.**
`(wind/tas)*sin(angle)` was passed to `math.asin()` with no clamp. At low TAS
the ratio exceeds 1, `asin` returns NaN, and the NaN went straight into
`sim/cockpit/autopilot/heading_mag`, leaving the autopilot with no valid
heading to fly.

### 4. Security

- **Hoppie logon code no longer written to `Log.txt`.** It was printed on every
  CPDLC transmission. `Log.txt` is the file users routinely upload for support,
  so this leaked a working third-party credential.
- **Path traversal fixed when writing an uplinked flight plan.** The CDU
  `fltno` field was interpolated directly into the output path, so a flight
  number containing `../..` wrote outside the FMS plans directory — with
  contents taken from an inbound network message. Now restricted to
  `[A-Z0-9_-]`, with the file handle nil-checked.
- **Three dead global `sleep()` definitions removed.** Two shelled out via
  `os.execute`, leaving an arbitrary shell-command primitive reachable from
  every script in the Lua state; the third busy-waited and would have frozen
  the sim. None had live callers.

---

## Known issues *not* addressed in this release

These were identified during review but deliberately left alone, either because
they need in-sim validation first or because they are larger pieces of work.

**ILS / autopilot**

- **No beam-gain scheduling.** Both loops apply a fixed gain to an *angular*
  deviation (`75 × vdef_dots` for glideslope, `4 × ldef_dots` for localiser).
  As range to the transmitter shrinks, the same dot count represents less
  linear displacement, so effective loop gain rises continuously to the
  threshold. Real 747 autopilots program beam gain down with radio altitude.
- **PID tuning.** `pidPitchI` is set equal to `pidPitchP` (line 981 overrides
  the branch above it) while `pidPitchD` is effectively zero; roll damping is
  scheduled to its *minimum* on approach. Gains are scheduled on altitude
  rather than dynamic pressure.
- **Derivative kick.** `pid.lua` uses derivative-on-error rather than
  derivative-on-measurement, so every step in the target injects a spike.
  `dtime` also comes from `total_running_time_sec`, which advances while
  paused.
- **LOC self-demotion.** Exceeding 4 combined dots sets `nav_status` back to
  armed, treating an overshoot as signal loss. Should key off
  `nav1_display_horizontal` instead. The fixes above should stop the
  oscillation that triggers it, but the logic itself is unchanged.

**FMS / LNAV**

- **DIRECT TO is overwritten within one frame.** `B747_getCurrentWayPoint_function()`
  writes `B747DR_ap_lnavHeading_mode` back to its own nearest-leg result
  (`B747.70.xt.autopilot.lua` lines ~2298 and ~2314), erasing the direct-to
  target. The clamp intended to prevent this is a dead store — it assigns to a
  local that is never read again. The steering code also uses
  `B747DR_fmscurrentIndex`, never `lnavHeading_mode`.
- **Active leg is found by nearest-leg search, not sequenced.** The whole plan
  is rescanned every frame for the closest leg. A real FMC sequences legs in
  order at a turn-anticipation point. After a route edit the active leg is a
  bare integer index into an array that was just rebuilt, with no re-anchoring
  by waypoint identity.
- **Manually selected courses do not stick.** `findILS()` parses the crew's
  course and then discards it (the assignment is commented out), and
  `B747_fltmgmt_setILS()` re-asserts frequency and course every 2 seconds
  within 45 NM of destination.
- **Flat-earth geometry.** `getTriSpaceSolver()` uses plane trigonometry for
  along-track/cross-track; `getDistance()` uses the spherical law of cosines,
  which loses precision at short range.
- **Cross-track is unsigned** and the PROGRESS page hardcodes an `L` prefix, so
  deviation always displays as Left.

**Performance**

- The autopilot still runs `json.decode()` on the entire flight plan every
  frame, which is the source of the garbage the collections above were
  compensating for.
- The ILS auto-tune search is O(waypoints × navaids) and re-runs every 2 s
  within 45 NM of the destination.
- Roughly 120 unguarded `print()` calls remain across the autopilot, monitor
  and autoland modules, several in per-frame paths.

**Security**

- The Hoppie request URL is still assembled by string concatenation without
  URL-encoding, so CDU text containing `&` or `'` can inject query parameters
  or break the request envelope.
- Override datarefs (`override_control_surfaces`, `override_wheel_steer`,
  `override_throttles`, `override_fms_advance`) are never released, because
  `aircraft_unload()` is commented out.

---

## Credits

Original 747-400 mod by Mark Parker (mSparks) and contributors, building on
Jim Gregory / Laminar Research's default 747-400. Licensed CC-BY-NC 4.0 — see
`LICENSE`. This fork keeps that licence.
