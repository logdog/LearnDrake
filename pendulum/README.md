# Pendulum (Plant)
This example is an attempt to understand [the pendulum example](https://github.com/RobotLocomotion/drake/tree/master/examples/pendulum) in Drake. Specifically, we strip down the example into its most basic components, eliminating unnecessary code and files.

## Running this example
Ensure that the code files are located in `drake/logdog/pendulum`. Then `cd` into the `drake` folder. In the first terminal, run
```
bazel run //tools:meldis -- -w
```
This will open a new window in browser, probably on localhost:7000. In the second terminal, run
```
bazel run //logdog/pendulum:passive_simulation
```
In the browser window, you should see the pendulum swinging back and forth.

# How it works, and what each file does

## `BUILD.bazel and .yaml files`
The `BUILD.bazel` file tells `bazel` how to build our program. Looking into its code, we see some `load` statements followed by three different function calls:
* `drake_cc_vector_gen_library` 
* `drake_cc_library`
* `drake_cc_binary` 

These python functions are part of Drake, and they are located in 
```
drake/tools/vector_gen/vector_gen.bzl
```
and 
```
drake/tools/skylark/drake_cc.bzl
```
The job of the `vector_gen` function is to automatically create 6 files (3 headers and 3 source files) named:
* `pendulum_input`
* `pendulum_params`
* `pendulum_state`

On my machine, they are located in 
```
drake/bazel-out/k8-opt/bin/logdog/pendulum/gen/
```
These files are generated from their respective `.yaml` files. It is important to note that the namespace provided in these `.yaml` files must match the namespaces provided in the other `.h` and `.cc` files throughout this example. See the `.yaml` files for more details.

## `pendulum_plant.*`
This file describes the dynamcis of the pendulum, given by the equation
$$ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = \tau $$
where $m$ is the mass of the point-mass, $l$ is the length of the (massless) rod, $b$ is the damping coefficient, $g$ is the acceleration due to gravity, $\theta$ is the angle of the pendulum from the downward position, and $\tau$ is the torque applied to the base of the pendulum. The dynamics are described in the function 
```c++
void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const final;
```
by writing the dynamics in state-space form, and modifying the `derivatives` state. There is also a function to get the energy of the system. The rest of the file is more or less boilerplate code which passes around (mutable) states and (mutable) parameters. 

You may ask yourself what is going on with these includes? Where are these files located? 
```c++
#include "drake/logdog/pendulum/gen/pendulum_input.h"
#include "drake/logdog/pendulum/gen/pendulum_params.h"
#include "drake/logdog/pendulum/gen/pendulum_state.h"
```
These generated files are created by `drake_cc_vector_gen_library` in the `Bazel` file, as described above.

## `pendulum_geometry.*`
This file describes the physical appearnace of the pendulum. This file has one public static function and no public constructor:
```c++
static const PendulumGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& pendulum_state_port,
      geometry::SceneGraph<double>* scene_graph);
```
The appearance of the pendulum is inside its constructor, which we can change to modify the appearance of the simulation. To be honest, this file still confuses me, but its static function is only called one time in `passive_simulation.cc`.

## `passive_simulation.cc`
TODO