# NDI 6 SDK drop-in

Drop `Install_NDI_SDK_v6_Linux.tar.gz` here. The SDK is license-gated by Vizrt
so the tarball is gitignored — every host installs its own copy.

1. Download from <https://ndi.video/for-developers/ndi-sdk/download/>.
2. Save the tarball as `vendor/ndi/Install_NDI_SDK_v6_Linux.tar.gz`.
3. Run `pixi run setup-ndi` (or `bash scripts/setup-ndi-host.sh` directly).

The script extracts the tarball, runs the Vizrt installer non-interactively,
and copies headers + `libndi.so.6` into `$HOME/.ndi-sdk/{include,lib}/` —
a host-stable per-user prefix that the pixi-build-ros sandbox can see.
`FindNDISdk.cmake` picks the headers up from there at build time;
`imaging_station_ndi_node` resolves the library at runtime via the
`IMAGING_STATION_NDI_RUNTIME_DIR` activation env var (auto-set by the
`lite-native` pixi env).

After running this, `pixi run lite-native-install` will rebuild
`imaging_station_ndi_bridge` and produce the `ndi_av_bridge` binary.
