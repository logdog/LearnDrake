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

## BUILD.bazel and .yaml files
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

## pendulum_plant.*
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
// pendulum_plant.h
#include "drake/logdog/pendulum/gen/pendulum_input.h"
#include "drake/logdog/pendulum/gen/pendulum_params.h"
#include "drake/logdog/pendulum/gen/pendulum_state.h"
```
These generated files are created by `drake_cc_vector_gen_library` in the `Bazel` file, as described above.

## pendulum_geometry.*
This file describes the physical appearnace of the pendulum. This file has one public static function and no public constructor:
```c++
// pendulum_geometry.h
static const PendulumGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& pendulum_state_port,
      geometry::SceneGraph<double>* scene_graph);
```
The appearance of the pendulum is inside its constructor, which we can change to modify the appearance of the simulation. To be honest, this file still confuses me, but its static function is only called one time in `passive_simulation.cc`.

## passive_simulation.cc
This is the actual simulation file. First, notice that there is an additional namespace present. This is standard practice on [the official Drake github page](https://github.com/RobotLocomotion/drake/tree/master/examples/pendulum), probably to avoid name collisions, but this did not cause any problems in my testing.

The `DEFINE_double` is part of the `gflags` package which allows us to easily parse command line arguments. This is good because we can change small simulation settings without the need to recompile (works great for tuning parameters). In the `main()` function call, we parse the command line flags, which is reflected by updating the `FLAGS_target_realtime_rate` variable in our code. For completeness, if we run
```
bazel build //logdog/pendulum:passive_simulation
bazel-bin/logdog/pendulum/passive_simulation -- -target_realtime_rate=0.5
```
then the value of `FLAGS_target_realtime_rate` would be 0.5 for that particular run. Of course, we could also just do
```
bazel run //logdog/pendulum:passive_simulation
```
but then we have to recompile. 

After the command line variables are set, the familiar Drake simulation pattern emerges. First, we construct a `DiagramBuilder`. We add a [`ConstantVectorSource`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_constant_vector_source.html) system for the pendulum input. This is equilvalent to the `zero` block in Simulink. For reference, the `PendulumInput.h` header file looks like this:
```c++
// drake/bazel-out/k8-opt/bin/logdog/pendulum/gen/PendulumInput.h
template <typename T>
class PendulumInput final : public drake::systems::BasicVector<T> {
    // ...
    /// Fluent setter that matches tau().
    /// Returns a copy of `this` with tau set to a new value.
    [[nodiscard]] PendulumInput<T> with_tau(const T& tau) const {
        PendulumInput<T> result(*this);
        result.set_tau(tau);
        return result;
    }
    // ...
  };
```
Recall that this file is auto-generated from the `pendulum_input_named_vector.yaml` file, which looks like this:
```yaml
# pendulum_input_named_vector.yaml
namespace: drake::logdog::pendulum
elements:
  -
    name: tau
    doc: Torque at the joint.
    doc_units: Newton-meters
    default_value: 0.0
```
Next, we add the pendulum to the builder, and connect the zero output from the `ConstantVectorSource` to the input of the pendulum. Then, a `SceneGraph` is created, and the `PendulumGeometry` is added to the builder. Finally, the `DrakeVisualizerd` is added to the builder, and the `diagram` is built. Note that `DrakeVisualizerd` is a short-hand for the `DrakeVisualizer` class when using a `double` scalar type.
```c++
// drake/geometry/drake_visualizer.h
using DrakeVisualizerd = DrakeVisualizer<double>;
```
A `Simulator` is created from the diagram, and a mutable `pendulum_context` is obtained. This allows us to get the mutable `pendulum_state` and set the initial conditions. The target realtime rate is set, the simulator is initialized, and the simulator is run for 10 seconds. As a sanity check, we ensure that the initial energy of the pendulum is greater than the final energy (since the pendulum is being damped). If, for example, we modified the damping coefficient in `pendulum_params_named_vector.yaml` to be negative instead of position, the system would gain energy, and this check would fail.
```
abort: Failure at logdog/pendulum/passive_simulation.cc:50 in DoMain(): condition 'initial_energy > 2.0 * final_energy' failed.
Aborted (core dumped)
```