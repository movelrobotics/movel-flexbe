# movel_flexbe_states
Contains flexbe states for executing Seirios RNS tasks.

## List of Available States

### `SeiriosRunWaypointState`
State for waypoint navigation on seirios. Make sure the correct map that contains waypoint is loaded.

**Parameters**  
| Parameter Name            | Type   | Description                                          |
|---------------------------|--------|------------------------------------------------------|
| `waypoint_name`           | string | Waypoint name. Make sure waypoint exists in the map. |
| `linear_vel`              | float  | Linear velocity.                                     |
| `angular_vel`             | float  | Angular velocity.                                    |
| `start_at_nearest_point`  | bool   | Whether to start task at nearest point.              |
| `enable_velocity_limiter` | bool   | Whether to enable velocity limiter.                  |

**Input Key**  
N/A

**Output Key**  
N/A

**Outcome**  
| Outcome   | Description                                                 |
|-----------|-------------------------------------------------------------|
| `arrived` | Waypoint task succeeds, robot's arrived to the destination. |
| `failed`  | Waypoint task fails.                                        |

### `SeiriosRunTrailState`
State for trail execution on seirios. Make sure the correct map that contains trail is loaded.

**Parameters**  
| Parameter Name            | Type   | Description                                    |
|---------------------------|--------|------------------------------------------------|
| `trail_name`              | string | Trail name. Make sure trail exists in the map. |
| `linear_vel`              | float  | Linear velocity.                               |
| `angular_vel`             | float  | Angular velocity.                              |
| `start_at_nearest_point`  | bool   | Whether to start task at nearest point.        |
| `enable_velocity_limiter` | bool   | Whether to enable velocity limiter.            |

**Input Key**  
N/A

**Output Key**  
N/A

**Outcome**  
| Outcome   | Description                                              |
|-----------|----------------------------------------------------------|
| `arrived` | Trail task succeeds, robot's arrived to the destination. |
| `failed`  | Trail task fails.                                        |

### `DynamicReconfigureState`
Modify parameter values at runtime with Dynamic Reconfigure (only for parameters with Dynamic Reconfigure enabled).

**Parameters**  
| Parameter Name          | Type   | Description                                                                                                                                                                       |
|-------------------------|--------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `reconfigure_namespace` | string | The namespace in which the parameter is located. For example, if we have rosparams `/my_node/my_params/A` and `/my_node/my_params/B`, the namespace will be `/my_node/my_params`. |
| `parameter_dict`        | dict   | The dynamically-reconfigurable parameter and value pairs. From the previous example, `parameter_dict` will be, for example, `{"A": 0.5, "B": True}`.                              |

**Input Key**  
N/A

**Output Key**  
N/A

**Outcome**  
| Outcome  | Description                                               |
|----------|-----------------------------------------------------------|
| `done`   | Dynamic reconfigure done; parameter successfully updated. |
| `failed` | Failed to update parameter.                               |

### `ToggleCostmapLayerState`
Toggle costmap layer on or off.

**Parameters**  
| Parameter Name | Type   | Description                                                           |
|----------------|--------|-----------------------------------------------------------------------|
| `costmap_name` | string | The costmap name. For example, `/move_base/local_costmap`.            |
| `layer`        | string | Costmap layer to be enabled/disabled. For example, `inflation_layer`. |
| `enable`       | bool   | Enable or disable costmap layer.                                      |

**Input Key**  
N/A

**Output Key**  
N/A

**Outcome**  
| Outcome  | Description                                  |
|----------|----------------------------------------------|
| `done`   | Costmap layer successfully enabled/disabled. |
| `failed` | Failed to enable/disable costmap layer.      |

## Deprecated States
- `SeiriosRunNavigationState`  
Deprecated in favour of [`SeiriosRunWaypointState`](#seiriosrunwaypointstate). Use that state instead.
- `SeiriosRunPathState`  
Deprecated in favour of [`SeiriosRunTrailState`](#seiriosruntrailstate). Use that state instead.