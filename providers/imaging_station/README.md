# imaging_station provider

Generic plate imaging-station provider for the Robot Skills Framework. Owns
the `robot_skills_msgs/action/ImagePlate` action; ships a sim backend that
writes placeholder PNGs and a live NDI receiver node that republishes a
remote NDI source (e.g. ZowieBox-40435) as `sensor_msgs/Image`.

## Packages

- **`imaging_station`** (ament_python) — the action server + bridge node.
  - `imaging_station_sim_node` — placeholder PNG sim, serves `ImagePlate`.
  - `imaging_station_ndi_node` — spawns the C++ NDI bridge, publishes frames.
- **`imaging_station_ndi_bridge`** (ament_cmake) — the native `ndi_av_bridge`
  binary. Vendored verbatim from the experiment_collector workspace.

## Sim backend

```
ros2 launch imaging_station imaging_station_sim.launch.py
```

Writes one PNG per requested site under
`~/.local/state/imaging_station/{plate_name}/{epoch_ms}_{counter}/site_{i}.png`.
No hardware required.

## NDI bridge node (live ZowieBox / any NDI source)

Connects to a named NDI source and publishes BGR8 `sensor_msgs/Image` frames
on `/imaging_station/image_raw`.

```
ros2 launch imaging_station imaging_station_ndi.launch.py \
  ndi_source_name:="ZOWIEBOX-40435 (ZowieBox-40435)"
ros2 run rqt_image_view rqt_image_view /imaging_station/image_raw
```

### Prerequisites — NDI 6 SDK

The bridge `dlopen`s `libndi.so.6` at runtime; the build needs
`Processing.NDI.Lib.h` at compile time. Neither is on conda-forge (NDI is
license-gated by Vizrt), so you must install it once per pixi env:

1. Download `Install_NDI_SDK_v6_Linux.tar.gz` from
   <https://ndi.video/for-developers/ndi-sdk/download/>.
2. Drop it at `vendor/ndi/Install_NDI_SDK_v6_Linux.tar.gz`.
3. Run `pixi run setup-ndi` (or `bash scripts/setup-ndi-host.sh` directly —
   the script does not require an active pixi env).
4. Run `pixi run lite-native-install` so the bridge package picks up the
   newly-installed headers and produces the `ndi_av_bridge` binary.

The script ([scripts/setup-ndi-host.sh](../../scripts/setup-ndi-host.sh))
extracts the tarball, runs the Vizrt installer non-interactively, and copies
headers + `libndi.so.6` into `$HOME/.ndi-sdk/{include,lib}/`. That host-stable
per-user prefix is reachable both by the pixi-build-ros build sandbox (via
`FindNDISdk.cmake`) and by the runtime (the bridge dlopens `libndi.so.6`
through `IMAGING_STATION_NDI_RUNTIME_DIR`, which the `lite-native` activation
env points at the same dir).

If you skip step 3, `pixi install -e lite-native` still succeeds — the bridge
build prints a `STATUS` warning and the package installs without the
executable, so unrelated lite-native development isn't blocked.

### Discovery on multi-NIC hosts

libndi's built-in mDNS implementation can silently fail on hosts with multiple
network interfaces (Linux multicast picks one route, which may not be the one
the source advertises on). The node falls back to `avahi-browse` automatically
to resolve `host:port`, then passes that to the bridge as `--url-address` so
the bridge connects directly without going through libndi's mDNS at all.

If discovery still fails, force it with the `url_address` parameter:

```
ros2 launch imaging_station imaging_station_ndi.launch.py \
  url_address:=10.6.105.98:5961
```

### "Video decoder not found"

NDI 6.3.x dlopens `libavcodec.so.61` / `libavutil.so.59` (FFmpeg 7.x SONAMEs)
and a cluster of codec helpers (`dav1d`, `openjp2`, `speex`, `lcms2`, `libva`).
Ubuntu's system FFmpeg is too old (60/58); conda-forge has the right versions
but only when explicitly pulled in.

The fix is wired through pixi: `[feature.lite-native.dependencies]` declares
`gst-libav` (transitively pulls FFmpeg 7.x), plus `lcms2 / dav1d / openjpeg /
speex / avahi`. `pixi run setup-ndi` then assembles
`$PROJECT_ROOT/.ndi-runtime-bundle/lib/` by copying `libndi.so.6` from
`$HOME/.ndi-sdk/` and the FFmpeg + codec libs from `$CONDA_PREFIX/lib`,
creating the SONAME aliases NDI looks for (`libavcodec-ndi.so.61` →
`libavcodec.so.61.X.Y.Z` etc.). The bundle path is exported as
`IMAGING_STATION_NDI_RUNTIME_DIR` by the `lite-native` activation env.

If you bump conda-forge versions later (e.g. FFmpeg 8.x lands), rebuild the
bundle to refresh the symlinks:

```
pixi run setup-ndi-bundle
```

### Network

NDI uses mDNS for source discovery; the imager and the host running the node
must be on the same broadcast domain (or use NDI Discovery Server / explicit
`url_address` overrides). Confirm your ZowieBox is visible:

```
ros2 run imaging_station_ndi_bridge ndi_av_bridge --help    # bridge CLI
```

…and look for the source name in `imaging_station_ndi_node`'s startup log
("Visible NDI sources (N): [...]").

### Parameters

| name                    | default                                          | meaning                                      |
|-------------------------|--------------------------------------------------|----------------------------------------------|
| `ndi_source_name`       | `"ZOWIEBOX-40435 (ZowieBox-40435)"`              | NDI source to subscribe to                   |
| `frame_id`              | `imaging_station_camera_optical_frame`           | `frame_id` stamped on every Image            |
| `target_fps`            | `0.0`                                            | Cap publish rate (0 = pass through)          |
| `discovery_timeout_sec` | `5.0`                                            | mDNS discovery wait at startup               |
| `bandwidth`             | `highest`                                        | NDI receiver bandwidth (`highest`/`lowest`)  |
| `image_topic`           | `image_raw`                                      | Topic name (under `/imaging_station/`)       |
| `camera_info_topic`     | `camera_info`                                    | CameraInfo topic name (under `/imaging_station/`) |
| `camera_info_url`       | `<share>/imaging_station/calibration/zowiebox_camera_info.yaml` | Calibration YAML; empty/missing → 50° HFOV placeholder |
| `startup_timeout_sec`   | `12.0`                                           | Bridge waits this long for the source        |

The node publishes a `sensor_msgs/CameraInfo` paired 1:1 with each Image
(matching `header.stamp` + `frame_id`, both `BEST_EFFORT` QoS). When
`camera_info_url` points at a present file the standard
`camera_calibration_parsers` YAML is loaded; otherwise the node
synthesises a 50° HFOV pinhole `CameraInfo` from the live frame
dimensions — good for RViz frustum visualisation, **not** for metric
reconstruction. Replace
[calibration/zowiebox_camera_info.yaml](src/imaging_station/calibration/zowiebox_camera_info.yaml)
with real `camera_calibration` output once a checkerboard pass has been
done at the macro lens's nominal working distance.

The static TF `microscope_camera → imaging_station_camera_optical_frame`
(0.193 m along +Z, 180° roll so Z-forward points down through the lens —
ROS optical-frame convention X-right Y-down Z-forward; override
translation with `optical_offset_z:=…` and rotation with
`optical_roll:=…` / `optical_pitch:=…` / `optical_yaw:=…`) is
published by
[imaging_station_scene.launch.py](launch/imaging_station_scene.launch.py),
which `meca500_bringup/moveit.launch.py` already includes via
`show_imaging_station:=true`. Run `pixi run meca500-real-up` (or the sim
equivalent) and the optical frame appears in RViz relative to
`meca_base_link` without the bridge running.

## Follow-ups

- A real `ImagePlate` backend that captures N frames per site from the live
  NDI stream and writes them to `output_root` (replacing or living alongside
  `sim_backend`). Today the sim and the NDI bridge run independently.
- Audio packets from the bridge are dropped in v1; add a ROS publisher when
  a consumer materialises.
- Replace the placeholder
  [zowiebox_camera_info.yaml](src/imaging_station/calibration/zowiebox_camera_info.yaml)
  with real intrinsics from `camera_calibration` once the macro lens has
  been calibrated against a checkerboard at its nominal working distance.
