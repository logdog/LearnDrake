#include <gflags/gflags.h>
#include "drake/common/text_logging.h"

namespace drake {
namespace logdog {
namespace hello {

DEFINE_string(your_name, "Zion", "Putting your name here so Drake recognizes you.");

void DoMain() {
    drake::log()->info("Hello " + FLAGS_your_name + " from Drake!");
}

} // namespace hello
} // namespace logdog
} // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage("A simple hello Drake example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logdog::hello::DoMain();
    return 0;
}