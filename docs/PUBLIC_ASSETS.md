# Public assets and provenance

The new `dynamic_nav_sim` package uses an original, synthetic robot built only from
boxes, cylinders and spheres. `scripts/generate_demo_robot.py` is its complete source.
Its chosen dimensions are teaching parameters, not measurements of a real vehicle.
The public robot has a 0.60 × 0.52 m physical envelope and a 2 cm planning margin.
Its ideal laser is at (0.20, 0, 0.25) m. No real sensor installation or calibration is included.

The public release does not include private robot URDF/STL/CAD files, derived physical
inertias or meshes, hardware sensor mount coordinates, original asset manifests,
private source hashes, hardware manuals, or photographs/screenshots of the private cart.
The original local hardware simulation stays outside this checkout.

README animations are rendered from a fresh simulation of the public primitive robot.
They show measured robot/obstacle positions and sampled global/local planner messages;
they are data replays, not Gazebo screen recordings. Geometry truth is used only to draw
and evaluate the experiment, not supplied as a prior navigation map. The dynamic obstacle
tracker consumes laser scans and TF, with ground-truth obstacle publication disabled.

Existing `teb_nav2_demo` assets belong to the earlier public demo and are unchanged.
Their existing licenses still apply. Third-party TEB, costmap_converter, and Nav2 sources
retain their license files and notices. The added `dynamic_nav_sim` package and new root
scripts are provided under Apache-2.0; see `src/dynamic_nav_sim/LICENSE`.
`src/nav2_costmap_2d/UPSTREAM.txt` identifies the exact upstream overlay and local patch.
