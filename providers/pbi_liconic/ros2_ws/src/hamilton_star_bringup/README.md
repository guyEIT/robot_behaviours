# hamilton_star_bringup

Launch files, deck config, and systemd unit for the Hamilton STAR ROS 2 action server.

## Don't kill the server casually

Every cold start of the action server calls `pylabrobot`'s `LiquidHandler.setup()`, which triggers a **full end-stops calibration** on the Hamilton Microlab STAR. That takes a minute or two and puts wear on the mechanics. So:

- Keep the process running across sessions — it holds the USB link open.
- Use the `AbortMotion` service (not `Ctrl-C`) to stop an in-progress goal.
- Use `ResetError` (with an acknowledgment string) to clear the `Error` state.
- Only restart the systemd unit when you've genuinely lost the hardware (USB drop, firmware hang).

## systemd install

```bash
sudo cp ~/code/plr/ros2_ws/src/hamilton_star_bringup/systemd/hamilton-star-action-server.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable hamilton-star-action-server.service
sudo systemctl start hamilton-star-action-server.service
journalctl -u hamilton-star-action-server.service -f
```

The unit depends on `/dev/hamilton_star` being present — install the
project's udev rule first (`99-hamilton.rules`) so the symlink appears
when the STAR is plugged in.

## Manual launch (dev)

```bash
cd ~/code/plr
pixi run --environment ros2 colcon build --symlink-install   # one-time
pixi run --environment ros2 ros2 launch hamilton_star_bringup action_server.launch.py \
    deck_file:=$PWD/ros2_ws/src/hamilton_star_bringup/config/star_deck.json
```

## Seed the deck

```bash
pixi run --environment ros2 python - <<'PY' > ros2_ws/src/hamilton_star_bringup/config/star_deck.json
import json
from pylabrobot.resources.hamilton import STARDeck
print(json.dumps(STARDeck(core_grippers="1000uL-5mL-on-waste").serialize()))
PY
```

Then edit the JSON to add plate carriers, tip racks, plates, etc., or
serialize from a scripted deck-assembly pass via `~/serialize_deck`.
