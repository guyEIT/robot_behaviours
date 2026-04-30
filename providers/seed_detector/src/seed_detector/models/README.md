# Seed-detector model weights

Drop the trained YOLO weights file here as `last.pt` (or override
`model_path` on the launch / node parameter).

The default `model_path` resolves to
`<install-share>/seed_detector/models/last.pt`, which is a colcon-installed
copy of this directory. With `colcon build --symlink-install`, edits to
this directory are picked up live.

`*.pt` is gitignored — weights are not committed; ship them out of band.
