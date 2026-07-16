# Safe boot-time updates

The repository contains, but does not install automatically, a oneshot updater
ordered before the existing `waverover.service`. The live service inspected on
WaveRover 131 runs as `waverover:waverover`, has supplementary `dialout`, works
in `/home/waverover/ros2_ws`, sources ROS Jazzy and the workspace overlay,
launches `waverover robot.launch.py`, and restarts on failure. The supplied
drop-in preserves all of that and adds only ordering plus the persistent
`WAVEROVER_IDENTITY_FILE` environment variable.

The updater accepts only the expected GitHub HTTPS/SSH remote, an ignored and
untracked strict identity, a completely clean `main`, and a fetched commit that
is a fast-forward. It never cleans, merges, rebases, installs dependencies, or
controls the rover service. Offline fetches return success so the installed
rover can start. No-change boots skip a build when `install/setup.bash` exists;
a missing overlay triggers a build. After an update, a failed build restores
the exact old commit and rebuilds it. A successful rollback returns success;
failure of both builds returns an error.

`/etc/default/waverover-update` may override `REMOTE`, `BRANCH`, or other
environment values without being overwritten by repeat installation. Because
the deployment copies are root-owned, rerun `deployment/install.sh` after a
later commit changes them.

## Review and installation

```bash
bash -n deployment/update_and_build.sh deployment/install.sh
sudo ./deployment/install.sh
```

The installer validates a clean repository and safe identity, copies files,
and runs only `systemctl daemon-reload`. It does not enable, start, stop, or
restart anything and does not run the updater. Follow the exact manual test,
log, and removal commands it prints. Test only while the rover service is
stopped.

## First-time migration of another rover

Perform this manually on each rover, with motors made safe:

1. Stop `waverover.service` and record `robot_name` from the old defaults.
2. Move the old `src` outside `/home/waverover/ros2_ws`; do not delete it.
3. Also preserve or move old `build`, `install`, and `log`, which can contain
   symlinks into the copied source.
4. Clone `https://github.com/Yacine-Derder/Waverover-SIJ.git` directly as
   `/home/waverover/ros2_ws/src`—never as another directory inside `src`.
5. Create `waverover/config/robot_identity.yaml` containing only the recorded
   ID, for example `robot_name: "132"`.
6. Verify `git check-ignore -v waverover/config/robot_identity.yaml` succeeds
   and `git ls-files --error-unmatch ...` fails.
7. Perform an initial clean Jazzy `colcon build --symlink-install`, source the
   new overlay, and run the verified non-hardware tests.
8. Inspect the live service and repository integration, then install the
   updater manually.
9. With the rover service still stopped, test the updater and inspect its log.
10. Start the rover, inspect its log, then reboot and inspect both service logs.
11. Keep the old backup until the installation has been fully validated.

Backup source must remain outside `ros2_ws`; otherwise colcon recursively finds
duplicate packages, as occurred with `src.before-git`.

## CI note

The old `ros2waverover/.github/workflows/app.yaml` is intentionally retained:
it is a package-local Humble multi-architecture Docker Hub publishing workflow
whose Docker build context assumptions do not match the repository root. The
root Jazzy workflow builds and tests the workspace without deployment or a real
identity. It builds the vendored LDROBOT driver but skips that package's
lint-only test target because the imported source has a large pre-existing
copyright and formatting baseline; all WaveRover-owned test targets run.
