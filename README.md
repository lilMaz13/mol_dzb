# `mol_dzb_turtle_heartbreak` package
A TurtlesimHeartBreak egy ROS 2 node, amely egy turtlesim teknőc mozgását vezérli, először egy szív alakzatot rajzolva, majd egy cikcakk mintát hoz létre. 
A node a következő funkciókat valósítja meg:

Pen beállítások: Változtatja a toll színét, vastagságát és állapotát (rajzolás be/ki).
Teleportálás: A teknőcöt egy meghatározott kezdőpontba teleportálja.
Mozgás: Az alakzatok megrajzolásához különböző mozgási parancsokat küld a teknőcnek.
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/sze-info/ros2_cpp_template
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select ros2_cpp_template --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch ros2_cpp_template launch_example1.launch.py
```

# Delete this part if you are using it as a template

ROS 2 pacage template, to get started, use template by clicking on the Green button labeled [`Use this template`](https://github.com/sze-info/ros2_cpp_template/generate) / [`Create new repository`](https://github.com/sze-info/ros2_cpp_template/generate). 

<p align="center"><img src="img/use_this_template01.png" width="60%" /></p>


Let's assume 
- your Github username is `mycoolusername`
- your ROS 2 repo shold be `cool_ros2_package`

Replace everything in the cloned repo:

- `ros2_cpp_template` >> `cool_ros2_package` (the folder was already renamed after `Use this template`)
- `sze-info` >> `mycoolusername`
- find all `todo` strings and fill the blanks

The easiest way is VS code:

<p align="center"><img src="img/replace01.png" width="60%" /></p>

Now `colcon build` your ROS 2 package and you can start wokring.
