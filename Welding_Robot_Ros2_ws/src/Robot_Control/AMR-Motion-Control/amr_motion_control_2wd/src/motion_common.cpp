#include "amr_motion_control_2wd/motion_common.hpp"

namespace amr_motion_control_2wd
{

std::atomic<ActiveAction> g_active_action{ActiveAction::NONE};

}  // namespace amr_motion_control_2wd
