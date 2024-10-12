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
git clone https://github.com/lilMaz13/mol_dzb
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select mol_dzb --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 run mol_dzb turtle_heartbreak
```
