# Stock Roomba 675

The six photos show the chassis in its factory configuration. The [measurement sketch](measurements.HEIC) uses the floor below the drive-axle midpoint as the origin, with +X forward, +Y left, and +Z up.

| Dimension | Value | Basis |
| --- | ---: | --- |
| Overall width and length | 340 × 340 mm | The two final dimension lines on the sketch. A separate note says approximately 335 × 335 mm. |
| Floor to top | 80 mm | Measured with the robot sitting on the floor. |
| Drive tread width | 15 mm | Marked for each wheel. |
| Inside tread edge to centerline | 112 mm | Marked for each wheel. |
| Drive wheel track | 239 mm | Derived from two 112 mm offsets plus one 15 mm tread width. |
| Front caster center to front edge | 40 mm | Marked on the sketch. This puts the caster at X = +130 mm if the axle is centered in the 340 mm envelope. |

The [colored USD](roomba_675.usd) contains the body, two drive wheels, and front caster as separate components. Materials and labels are embedded in the file. The 80 mm height includes the top of the front IR receiver in this reconstruction. The bumper, buttons, bin, cleaning head, and other smaller details are estimated from the photos. The body assumes a circular envelope centered on the axle. Drive wheel diameter (72 mm), caster diameter (24 mm), and ground clearance (10 mm) come from the existing simulator and are provisional. The brush bristles can extend beyond the rigid footprint.

The [Gazebo SDF](../ros2_ws/src/roomba_bringup/models/roomba_675/model.sdf) is a static model with colored COLLADA visuals and primitive collisions. Add `ros2_ws/src/roomba_bringup/models` to `GZ_SIM_RESOURCE_PATH` to include it in a world as `model://roomba_675`. Keep its `meshes` directory next to the SDF when moving the model. The [stock Xacro](../ros2_ws/src/roomba_bringup/urdf/roomba_675.urdf.xacro) uses the same visuals and retains the wheel joints. The standalone model is for visual inspection and scene placement. Drive simulation requires mass and inertia, wheel control, and contact parameters.

The USD uses meters, with Z up and +X forward on the floor below the drive axle. The stock assembly has an estimated mass of **2.72 kg** based on the owner's approximate 6 lb reading. Wheel and chassis inertia require estimates or separate measurements.

Expand the standalone description from the repository root with ROS 2 sourced.

```sh
xacro ros2_ws/src/roomba_bringup/urdf/roomba_675.urdf.xacro > /tmp/roomba_675.urdf
```

Regenerate the USD and COLLADA assets from the measured Xacro dimensions with `python3 description/generate_roomba_675.py`. The generator requires `usdcat` from OpenUSD.
