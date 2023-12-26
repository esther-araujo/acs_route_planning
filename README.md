# acs_route_planning

This package contains an implementation of Ant Colony System for Path Planning

## Dependencies

- [tuw_multi_robot](http://wiki.ros.org/tuw_multi_robot)
- [heightmap_generation](https://github.com/blackcoffeerobotics/heightmap_generation)

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

### Launch ACS route planning

```bash
roslaunch acs_route_planning  load_graph.launch room:=room_name
```

After roslaunch, you have to publish two points to run the ACS: the start point and the end point for the robot

<h1 align="center">
  <img alt="" width="60%" height="auto" src="./images/cave.png"/>
</h4>

### Load simulation in Gazebo

```bash
roslaunch acs_route_planning  world.launch world:=world_name
```

<h1 align="center">
  <video width="320" height="240" controls>
  <source src="./images/gazebo.mp4" type="video/mp4">
</video>
</h4>
