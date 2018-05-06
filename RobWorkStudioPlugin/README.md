# ROVI2 Obstacle Avoidance: RobWorkStudio Simulation Plugin

### Adding Shared Libraries from other project.

To be able to link AnytimePlanning lib, created for path planning in the planner package, add the following lines to CMakeLists.txt before the `add_library()` part:

```
### Linkig AnytimePlanning to the Plugin

# include directories for shared libraries
include_directories( /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/planner/include )

# Find Library
find_package(
  AnytimePlanning
)
```

Then, in the .hpp ad the library header as ususal:

```cpp
#include <AnytimePlanning.h>
```
