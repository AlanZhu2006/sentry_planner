FAST-LIO/ICP map sets are stored here. The repository includes the latest
validated `arena` PCD/PGM/YAML set as a reproducible navigation fixture; newly
generated map names remain ignored until they are deliberately reviewed.

Verify the tracked set with `sha256sum -c SHA256SUMS` from this directory.

Create and save a map named `arena` with:

  just map arena
  just map-save arena

The resulting `arena.pcd`, `arena.pgm`, and `arena.yaml` must stay together.
The PCD is used by ICP localization; the PGM/YAML pair is used by Nav2.

Start navigation with:

  just nav arena

Keep the robot still while ICP searches within 2.25 m of the supplied initial
pose and checks all headings. The current 1225-candidate full-map search can
take about 50 seconds on the Jetson. It publishes `map -> odom` only after the
fitness, overlap, RMSE, and seed-correction gates pass. If the robot is outside
the search volume, use RViz `2D Pose Estimate` as a new seed.
