# Hello World (Drake)
A simple hello world example, modified slightly from [this tutorial](https://drake.guzhaoyuan.com/thing-to-do-in-drake/hello-drake). 

## To run this example...
First, install [Drake](https://drake.mit.edu/from_source.html). Make sure `hello` folder is in the appropriate location on your machine. On my machine, it is `/home/ldihel/drake/logdog/hello`. Then, in a terminal window, you can do any of the following:
```
cd /path/to/drake
bazel run //logdog/hello:hello_exe
bazel run //logdog/hello:hello_exe -- --your_name Logan
bazel run //logdog/hello:hello_exe -- --your_name=Logan
```
Alternatively, you can build the program once and then execute the program. This is faster, but requires two commands.
```
bazel build //logdog/hello:hello_exe
bazel-bin/logdog/hello/hello_exe --your_name Logan
```
