# acs_route_planning

This package contains an implementation of Ant Colony System for Path Planning

## Dependencies

- [tuw_multi_robot](http://wiki.ros.org/tuw_multi_robot)
- [map2gazebo](https://github.com/shilohc/map2gazebo)

## Testing

### Generate maps for tests

```bash
python3 scripts/jody_map_creator/generate_maps.py
```

### Run tests for ACS algorithm

```bash
./tests/test_acs.sh
```

This will run the test enviroment, using the generated maps (in scripts/jody_map_creator/generated_maps), and the tuw_voronoi_graph package

### Load simulation in Gazebo

```bash
roslaunch acs_route_planning  world.launch map:=map_name.world room:=room_name
```
Ex:
```bash
roslaunch acs_route_planning  world.launch room:=randomMap56_FixDisc_5_50_2_1
```

<h1 align="center">
  <img alt="" width="60%" height="auto" src="./images/gazebo.gif"/>
</h4>

After roslaunch ACS, you have to launch the move_base, thwn, use rviz to publish the goal point for run acs:

```bash
roslaunch acs_route_planning move_base_mapless.launch
```

If ACS could found a path, run the move_robot node to send the path to Navigation Stack:

```bash
rosrun acs_route_planning move_robot.py
```

<h1 align="center">
  <img alt="" width="60%" height="auto" src="./images/simulation.gif"/>
</h4>
