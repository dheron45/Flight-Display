# Flight display

An overhead flight display for the Adafruit Matrix Portal M4 with a 64x32 LED matrix. The board updates itself over WiFi from this repo's GitHub releases.

## How updates work

1. Publish a GitHub release (e.g. tag `v1.2.0`).
2. A GitHub Action compiles `src/*.py` to `.mpy`, writes `manifest.json` (the version, plus each file's size and CRC32) and force-pushes both to the `dist` branch.
3. The board picks up the release at its next reset, or within an hour while it's running. It downloads the files, checks them against the manifest and swaps them in, keeping the old files as `*.prev`.

If a release crashes the display 3 times in a row, the board puts the previous version back and skips the bad release. Publishing a newer release installs normally.

| On the board | Where it comes from |
| --- | --- |
| `boot.py` | `board/boot.py`, copied once over USB. Makes the drive writable by the board. |
| `code.py` | `board/code.py`, copied once over USB. The updater. |
| `app.py` | `board/app.py`, copied once over USB. Starts the display and resets the board if it crashes. |
| `flight_app.mpy` | Built from `src/flight_app.py` and installed by the updater. |
| `version.txt`, `skip_version.txt`, `*.prev` | Written by the updater. |
| `settings.toml`, `lib/` | Your own. They are never in the repo or touched by updates. |

The files in `board/` aren't updated over the air, so a bad release can't break the updater.

## First-time setup

1. Back up the board's current `code.py` and `settings.toml`.
2. Add this line to `settings.toml` on the board:
   ```toml
   update_manifest_url = "https://raw.githubusercontent.com/dheron45/Flight-Display/dist/manifest.json"
   ```
3. Push this repo to GitHub as a **public** repo, then publish a release. Check that the Action ran and the `dist` branch exists.
4. Copy `board/boot.py`, `board/code.py` and `board/app.py` onto CIRCUITPY. This replaces the old `code.py`.
5. Press RESET. In the serial console, the updater installs the release and then starts the display.

## Editing files over USB

`boot.py` makes the drive writable by the board, which makes it **read-only from your computer**. To edit files on the board, **hold UP while pressing RESET**. The drive is then writable from the computer, and updates are skipped until the next normal reset.

## Notes

- The `CIRCUITPY_VERSION` setting in `.github/workflows/release.yml` must have the same major version as the board's CircuitPython (see `boot_out.txt`).
- To test a build locally:
  ```sh
  mpy-cross src/flight_app.py -o build/flight_app.mpy
  python3 tools/make_manifest.py build test
  ```
- `raw.githubusercontent.com` can serve a stale copy for about 5 minutes after a release. If the board gets a mismatched file, it discards it and tries again on the next check.
