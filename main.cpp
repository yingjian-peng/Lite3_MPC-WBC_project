
#include "state_machine.hpp"
#ifdef BUILD_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif
using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    try {
        StateMachine state_machine(RobotType::Lite3);
        state_machine.Run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 2;
    }
}
