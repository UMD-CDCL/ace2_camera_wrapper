// Standalone executable wrapper around the component, so the node can still be
// run on its own with `ros2 run` or a plain launch Node action. Composition into
// a container is the same class, loaded by rclcpp_components instead.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ace2_camera_wrapper/ace2_camera_wrapper_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    // Intra-process comms buys nothing for a lone process, but enabling it keeps
    // the standalone and composed paths behaving identically for anything that
    // inspects the option.
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<ace2_camera_wrapper::Ace2CameraWrapperNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // Without this, a bad parameter surfaces as a bare "terminate called after
    // throwing" and a core dump, which says nothing about what was wrong. The
    // launch file respawns the node, so the reason needs to be legible.
    RCLCPP_FATAL(rclcpp::get_logger("ace2_camera_wrapper"), "Startup failed: %s", e.what());
    exit_code = 1;
  } catch (const Pylon::GenericException & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ace2_camera_wrapper"), "Startup failed in pylon: %s", e.GetDescription());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
