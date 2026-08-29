# B747-400 DJMod — changes against the upstream Sparky mod

This repository is the 747-400 "Sparky" mod for the Laminar 747 in X-Plane 12,
with a set of targeted fixes to ILS tracking, autopilot stability and script
safety. The unmodified upstream source is preserved at the git tag
`baseline-sparky`, so any change here can be inspected with:

```bash
git diff baseline-sparky
```

and any individual fix reverted with `git revert <commit>`.

To update an existing install without re-downloading 2.6 GB, see
[UPDATING.md](UPDATING.md).

---

## v1.3.0 — glideslope pitch law rewritten

Still hunting with large deviations after three rounds of tuning, so this stops
patching the existing law and changes its structure.

### Why the old law could not be tuned straight

`getGlideSlopeFPM()` commanded a **vertical speed** proportional to beam
deviation. Deviation is in dots, which is an **angle**, and the vertical speed
needed merely to *hold* a given angular deviation shrinks in proportion to
range. So the correct fpm-per-dot gain changes continuously all the way down
the approach. The beam-gain scheduling added in v1.1.0 only approximates that,
and the residual is what was still showing up as hunting.

Commanding a **flight path angle** is scale invariant. With vertical offset *h*
at range *R* the angular deviation is θ = h/R; since dR/dt = −V, holding θ
constant already requires a path-angle change of θ, and closing it requires
proportionally more. The required correction is therefore proportional to
**dots at every range**, with no range term at all — which is why this form
needs no beam gain programming and behaves the same at 10 NM as at 300 ft.

### The new law

```
θ = γ + α
θ_cmd = θ_actual + K · (γ_cmd − γ_actual)
```

Holding angle of attack constant, the pitch that achieves the commanded path is
the current pitch plus the flight-path error. It is self-trimming: α is
measured implicitly rather than integrated, so there is no integrator to wind
up or to corrupt across a mode change, and it settles cleanly — at steady state
γ_actual = γ_cmd, so θ_cmd = θ_actual and the loop stops moving.

This also collapses a three-stage cascade (dots → target fpm → incremental
pitch nudge → elevator) into one stage, removing two of the three integrators
from the glideslope path. G/S no longer shares the generic vertical-speed
branch, and gets a shorter smoothing time constant (0.25 s vs 0.5 s) and a 10 Hz
sample rate — both safe now the command is a direct computation rather than an
incremental nudge.

Speed is held by the autothrottle in SPD mode whenever pitch mode is G/S, so
pitching for path is the correct division of labour.

### Gains

Chosen from an offline sweep of the closed loop (point-mass approach with a
first-order pitch response). **P = 2.0 deg/dot, D = 3.0 deg per dot/s**:
converges to the centreline by 2 NM from a 1 dot intercept with zero overshoot,
and stays well behaved with the airframe response anywhere from 1 to 4 seconds,
from ±2 dots, and with 0.08 dot of beam noise. D is insurance rather than
load-bearing — the loop is stable at D = 0 and at D = 6.

### A/B testing this in the sim

Set `GS_USE_LEGACY_FPM_LAW = true` near the top of the glideslope block in
`B747.19.xt.hydraulics_override.lua` to switch back to the old vertical-speed
law without a rebuild, so the two can be compared on the same approach.

For a trace of what the law is doing, set the dataref
`laminar/B747/debug/flight_directors` to 1 (DataRefEditor or DataRefTool). The
G/S line reports dots, deviation rate, commanded and actual flight path angle,
current pitch and the resulting command, once per sample.

### Also fixed: two inverted conditions in the shared vertical-speed branch

```
currentFPM < targetFPM and speed_delta < -max_speedDelta  ->  pitch DOWN
currentFPM > targetFPM and speed_delta >  max_speedDelta  ->  pitch UP
```

Both read *"already diverging from the target, so move further from it"* —
positive feedback that fired precisely during an excursion. This branch still
serves V/S and VNAV PATH, so the fix matters even though G/S no longer uses it.
The pitch-up test also compared raw `simDR_vvi_fpm_pilot` while its partner
used `currentFPM`, so the two halves of one comparison used different values.

---

## v1.2.0 — VNAV speed tracking and speedbrake axis option

### VNAV SPD / FLCH now pitches for the FMS speed target

Reported after flight test: climb is smooth, but the flight director bars do
not reflect the speed target on the ASI — VNAV vertical guidance looks
disconnected from the FMS-computed climb speed.

Two things it turned out **not** to be, both checked first:

- *FD routing is fine.* The PFD reads `laminar/B747/autopilot/flight_director_pitch_deg`
  (confirmed in the `.acf` panel definition), which is the dataref this mod
  writes. The bars do show what is actually being flown.
- *The speed chain is intact.* FMS `clbrestspd`/`clbspd`/`transpd`/`crzspd` →
  `B747DR_ap_ias_dial_value` → `simDR_autopilot_airspeed_kts`, and
  `ap_director_pitch()` targets exactly that.

The defect was the pitch law. The VNAV SPD / FLCH branch is a pure rate-based
nudge integrator: it moves commanded pitch by ±`rog` based only on whether
speed is *changing* fast enough, gated by deadbands, with **no term
proportional to how far off the target speed actually is**. It converges
eventually — hence the smooth climb — but the command never visibly encodes
the speed error, so the bars read as disconnected from the speed bug. The real
aircraft pitches directly for the FMS speed in VNAV SPD.

Proportional and rate terms on speed error are now applied to the command:

```
vError  = IAS − target                 (clamped ±25 kt)
vRate   = dIAS/dt                      (clamped ±3 kt/s)
command = trimPitch + 0.15·vError + 0.80·vRate     (term capped ±5°)
```

Sign convention is speed-on-elevator: too fast pitches up to trade speed for
climb. The existing integrator is deliberately left untouched, so the smooth
convergence gained in v1.1.0 is preserved and only the command becomes
speed-aware. At steady state both new terms are ~0, so they do not fight the
integrator.

*Tuning:* `SPD_PITCH_P`, `SPD_PITCH_D`, `SPD_PITCH_LIMIT`, `SPD_ERR_CLAMP` at
the top of `ap_director_pitch` in `B747.19.xt.hydraulics_override.lua`. Setting
`SPD_PITCH_P` and `SPD_PITCH_D` to 0 restores exactly the v1.1.0 behaviour.

### SPEEDBRAKE AXIS option

New **SPEEDBRAKE AXIS NORM/REV** toggle at L4 on **MAINT > SIM CONFIG page 2**,
saved per livery with SAVE like the other options there. Reverses the
speedbrake axis for this aircraft only, so a joystick profile shared with other
types does not need changing just for the 747.

Only the 0..1 working travel is mirrored — the negative ARMED value is a
sentinel rather than a position, so it passes through untouched and the
arm/disarm logic is unaffected. Defaults to NORM; configs saved before this
option existed have no key and fall back to NORM without erroring, so no
migration is needed.

---

## v1.1.0 — vertical axis and LNAV

Flight test of v1.0.0 reported lateral tracking now stable, but pitch hunting
both after takeoff with the AP engaged and on approach, getting worse closer to
the runway.

### Vertical axis

**Pitch integral gain was ~10× too high.** `ap_pitch_assist()` computed
`pidPitchI = pidPitchP * 0.1` in one branch and `= pidPitchP` in the other, then
unconditionally overrode *both* with `pidPitchI = pidPitchP` on the next line.
With error in degrees of pitch, `ki = kp` drives the integrator to full elevator
authority in ~15 s of a 1° error. This was the dominant cause of the slow
oscillation in every pitch mode — which is why it appeared after takeoff as well
as on approach. The author's own first branch already had the right value; it
was dead code.

**Pitch D term made real, and corrected to derivative-on-measurement.**
`pidPitchD` was `0.0002` — effectively no damping at all. It had to stay that
small because the D term acted on *error*, so any meaningful gain amplified the
kick from every flight-director step instead of damping. `pid.lua` now supports
`derivativeOnMeasurement` per instance (enabled for pitch only, so the roll and
yaw tuning that is currently working is untouched), and `pidPitchD` is `0.02`.

**Glideslope beam gain programming.** `getGlideSlopeFPM()` applied a fixed
75 fpm per dot. Dots are an *angular* measure, so the linear displacement one
dot represents shrinks with range — effective loop gain climbed as ~1/range all
the way to the threshold. This is exactly the "worse as you get close to the
runway" symptom. The correction is now scheduled on radio altitude (full gain
at/above 1500 ft, 0.15× at/below 150 ft) with an added deviation-rate damping
term. The hard 3× gain step at exactly 2.0 dots — a discontinuity the loop can
limit-cycle across — is now a smooth ramp from 1× at 1 dot to 3× at 2.5 dots.

**Integrator corruption on pitch mode change.** `ap_director_pitch_retVal()`
holds the command for 0.7 s across a mode change, but it also assigned the held
value back into `ap_director_pitch()`'s integrator, and the VS/GS branch did the
same again on its way out. The integrator was dragged backwards for 0.7 s on
every mode change, and the command jumped when the hold released.

**Elevator rate limit was inverted.** `B747_interpolate_value`'s `speed`
argument is *seconds to traverse the range*, so larger is slower. The schedule
was `rescale(1, 3, 10, 10, error)` — the elevator got **slower** the larger the
pitch error, worst exactly during a capture or mode change.

**`pid.lua` hardening.** `dtime` is clamped to 0.2 s (`simDRTime` keeps
advancing while paused or loading, producing a huge integral jump on resume),
and proper anti-windup was added via an optional `iLimit` plus conditional
integration when the output is saturated.

*Tuning knobs, in likely order of usefulness:* `B747DR_pidPitchD` (0.02) in
`B747.19.xt.hydraulicsmodel.lua`; `GS_RATE_GAIN` (250) and `GS_GAIN_FLOOR`
(0.15) in `B747.19.xt.hydraulics_override.lua`.

### LNAV

**DIRECT TO now works.** The FMS publishes the target leg in
`B747DR_ap_lnavHeading_mode` with the `-100` cross-track sentinel.
`B747_getCurrentWayPoint_function()` then overwrote it from its own nearest-leg
result in *two* places, and the clamp meant to prevent that assigned to a local
(`best`) never read again — a dead store that did nothing. A direct-to is now a
lock: the active leg is forced to the requested waypoint and the nearest-leg
search is skipped entirely until the existing 10 NM release hands back to normal
sequencing.

**Active leg re-anchors by waypoint identity.** The name of the tracked waypoint
is remembered; if the waypoint under the index changes identity, the route was
edited and the leg follows the waypoint to its new index. This is the "it loses
its location" case.

**Sequencing is forward-only.** The old condition also fired for
`best < fmscurrentIndex-1`, so the search could throw the active leg *backwards*
to any geometrically closer leg — on procedure turns, holds, or parallel STAR
segments. One non-forward move is still permitted when the tracked waypoint
disappears entirely (a stock-CDU DIRECT TO deletes intervening waypoints and
renumbers the rest), otherwise forward-only would deadlock LNAV after a stock
direct-to.

**Cross-track error is signed**, and the PROGRESS page shows L/R correctly
instead of a hardcoded `L`. The no-match sentinel (100) is no longer published
as a real cross-track.

**Manually entered ILS course is honoured.** `findILS()` parsed the crew's
FREQ/CRS entry, used the course to choose which ILS to tune, then discarded it —
the assignment was commented out.

---

## v1.0.0 — first ILS and stability pass

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

- **LOC self-demotion.** Exceeding 4 combined dots sets `nav_status` back to
  armed, treating an overshoot as signal loss. Should key off
  `nav1_display_horizontal` instead. The fixes above should stop the
  oscillation that triggers it, but the logic itself is unchanged.

**FMS / LNAV**

- **No leg-type model.** Legs are still selected by nearest-track search rather
  than a real ARINC leg model (TF/DF/CF/HM). v1.1.0 made that search behave
  (forward-only, identity-anchored, DIRECT TO honoured) but it is still a
  search, not a sequencer. Holds in particular are not modelled.
- **Turn anticipation is a distance heuristic**, not radius computed from bank
  limit and TAS, so turn onset varies with groundspeed in ways the real FMC's
  would not.
- **Steering aims at a pseudo-waypoint** placed down the leg rather than
  commanding bank from cross-track and track-angle error. This works but is not
  how the real airplane flies a leg.
- **Flat-earth geometry.** `getTriSpaceSolver()` uses plane trigonometry for
  along-track/cross-track; `getDistance()` uses the spherical law of cosines,
  which loses precision at short range.

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
