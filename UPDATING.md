# Updating an existing install

The full package is ~2.6 GB, but a typical update touches only a handful of
Lua scripts — v1.0.0 → v1.1.0 is **6 files, 72 KB**. There is no reason to
re-download the aircraft to pick up a fix, and no reason to lose your liveries,
preferences or joystick setup doing it.

Two ways to do this. Pick one.

---

## Option A — `git pull` (recommended)

Make your aircraft folder a git working copy once. After that every update is a
single command that downloads only the changed objects.

**One-time setup**, run inside your existing aircraft folder
(`X-Plane 12/Aircraft/.../747-400/`):

```bash
git init -b main
```
```bash
git remote add origin https://github.com/Carolinadoug/B747-400_DJMod.git
```
```bash
git fetch origin --tags
```
```bash
git reset --hard v1.1.0
```

`git reset --hard` overwrites *tracked* files with the repository versions. Any
file that is not in the repository — liveries you added yourself, prefs, your
`.acf` if you have re-tuned it — is untracked and is left completely alone.

If you have deliberately edited a tracked file and want to keep your version,
commit it first (`git add -A && git commit -m "my changes"`) and use
`git merge v1.1.0` instead of `git reset --hard`.

**Every update after that:**

```bash
git fetch origin --tags && git reset --hard v1.2.0
```

Or to track the latest without waiting for a tag:

```bash
git pull origin main
```

To see exactly what an update will change before you take it:

```bash
git diff --stat HEAD v1.2.0
```

To roll back to any earlier release:

```bash
git reset --hard v1.1.0
```

This is the option worth the five minutes of setup. Updates become a few KB,
rollback is instant, and you can always see what changed.

---

## Option B — update zip

For a one-off, or if you would rather not use git.

1. Go to the [Releases page](https://github.com/Carolinadoug/B747-400_DJMod/releases).
2. Download the `..._update.zip` for your current version → the new version.
3. Extract it **over** your existing aircraft folder, keeping the directory
   structure, and let it overwrite.
4. If the zip contains `DELETED-FILES.txt`, delete the files it lists. A zip
   cannot express a deletion, so this step is manual. Most updates will not
   have one.

**Back up first.** Copying the `plugins/` folder somewhere safe is enough for a
script-only update, and is what you need to undo it:

```bash
cp -r plugins plugins.backup
```

### Building an update zip yourself

The zips on the Releases page are produced by a script in this repository, so
you can build one between any two versions:

```bash
bash tools/make-update-zip.sh v1.0.0 v1.1.0
```

It prints the file list, writes
`B747-400_DJMod_<from>_to_<to>_update.zip`, and notes any deletions.

---

## Comparing releases in the browser

GitHub will show you the diff between any two tags without downloading
anything:

```
https://github.com/Carolinadoug/B747-400_DJMod/compare/v1.0.0...v1.1.0
```

Append `.patch` to that URL for a plain-text patch.

---

## After any update

X-Plane caches Lua scripts at aircraft load. **Reload the aircraft** (or
restart X-Plane) — reloading the *flight* alone is not always enough.

If something misbehaves, `Log.txt` in your X-Plane root is the first place to
look; Lua errors from the mod appear there with the script name.

---

## Releases

| Version | Contents |
|---|---|
| `baseline-sparky` | Unmodified upstream Sparky mod, for reference and diffing |
| `v1.0.0` | First pass: FD command filters, LOC capture freeze, localiser bearing wraparound, per-frame GC, security fixes |
| `v1.1.0` | Vertical axis: pitch PID integral/derivative, glideslope beam gain programming, integrator corruption, inverted elevator rate limit. LNAV: DIRECT TO, route-edit re-anchoring, forward-only sequencing, signed cross-track, manual ILS course |
| `v1.2.0` | VNAV SPD/FLCH pitch now tracks the FMS speed target; SPEEDBRAKE AXIS NORM/REV option on MAINT > SIM CONFIG p2 |

See `CHANGELOG-DJMod.md` for the detail on each.
