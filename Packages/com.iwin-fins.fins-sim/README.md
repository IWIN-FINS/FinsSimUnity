# FinsSim Unity Platform

`com.iwin-fins.fins-sim` is the embedded Unity simulation platform used by
FinsSimUnity. It contains the FinsSim runtime, sensors, actuators, ROS/gRPC
integration, hydrodynamics, editor tooling, and platform tests.

Task-specific RL/IRL logic and scenes live in the host project under
`Assets/FinsSimUnity/Tasks`.

The optional DWP2 integration requires a separately licensed installation of
`com.nwh.dynamicwaterphysics`; it is not distributed by this package.
