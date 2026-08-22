# Update packages

Small zips containing **only the files that changed** between two releases, so
an existing install can be updated without re-downloading the full 2.6 GB
package and redoing liveries, preferences and joystick setup.

## Download

Click the zip below, then **Download raw file** (the download button at the top
right of the file view). GitHub will not preview a zip, so the raw download is
the only way to get it from the web UI.

| Update | Size | Files |
|---|---|---|
| [v1.0.0 → v1.1.0](B747-400_DJMod_v1.0.0_to_v1.1.0_update.zip) | 79 KB | 10 |
| [v1.1.0 → v1.2.0](B747-400_DJMod_v1.1.0_to_v1.2.0_update.zip) | 60 KB | 10 |

## Install

1. Back up your `plugins/` folder — that is all a script-only update touches:
   ```bash
   cp -r plugins plugins.backup
   ```
2. Extract the zip **over** your 747-400 aircraft folder, keeping the directory
   structure, and let it overwrite.
3. If the zip contains `DELETED-FILES.txt`, delete the files it lists by hand.
   A zip cannot express a deletion. Most updates will not have one.
4. **Reload the aircraft** in X-Plane. Lua scripts are cached at aircraft load,
   so reloading the flight alone is not always enough.

## Prefer not to use zips?

See [UPDATING.md](../UPDATING.md) for the `git pull` route, which downloads only
changed objects, lets you diff before taking an update, and makes rollback a
single command.

## Building one yourself

```bash
bash tools/make-update-zip.sh v1.0.0 v1.1.0
```
