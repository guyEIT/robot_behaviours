# Hamilton STAR USB setup

The STAR appears on USB as vendor `08af` (Hamilton Bonaduz AG). To let this
project talk to it without running as root, and to give the device a stable
name, install the included udev rule:

```bash
./install-udev.sh
```

What it does:

1. Copies [99-hamilton.rules](99-hamilton.rules) to
   `/etc/udev/rules.d/99-hamilton.rules`.
2. Reloads udev and re-triggers device events.
3. Adds your user to the `plugdev` group if it isn't already.

After running it:

- **Log out and back in** so the new group membership takes effect.
- **Replug the STAR's USB cable** (or power-cycle the robot) so the rule
  applies to the live device node.

## Verification

- `ls -l /dev/hamilton_star` — should be a symlink owned by group `plugdev`.
- `pixi run lsusb` — should list an `08af:` device.
- `pixi run test-star` — connects, prints machine configuration, exits.
