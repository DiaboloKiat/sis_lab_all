## How to run
### Visualization
```
$ roslaunch sis_mobile viz.launch
```
### Mobile platform config
```
$ roslaunch sis_mobile kaku_config.launch
```
### move_base related 
```
$ roslaunch sis_mobile move_base.launch
```

In RViz, you will see a robot model as our mobile platform, click '2D Nav Goal' above
and set a goal for navigation in grid, the trajectory will show up and the car start to navigate
to the goal you set.

TODO: Use action client to set a goal
