# Junyong 30 m urban disaster profile

Source: https://github.com/qkrckstj/py_bt_ros/tree/dev_junyong/urban_disaster_30m
Imported commit: fd8ecd1 (2026-11-28). The original files are preserved under `upstream/`.

Run from repository root: `bash scripts/run_disaster_junyong.sh`.
The existing city remains available through `bash scripts/run_disaster_city.sh`.

- Six simulated LIMO, seven incidents (one of each type), 22 survey sites.
- Existing public-state minimum-crew baseline, team/robot BTs, Nav2, one-slot charger.
- Incident centers remain visual locations; navigation uses upstream approach poses.
- Tasks are revealed at their accessible survey positions. Public observations omit undiscovered tasks.
- Start formation shifted within the southwest staging area because upstream robot1's first-row position overlaps an occupied cell in the supplied static map. All starts, survey sites, charger and work positions pass inflated-map connectivity checks.
- Preview robot meshes are replaced by simulated LIMO. World coordinate system explicitly ENU; textures/models retained.
- Progress, arrived/minimum crew and actual work multiplier appear above discovered tasks.
- Original hybrid/timed-event presentation triggers are retained only in the upstream reference; this profile uses survey discovery and six simulated robots.
