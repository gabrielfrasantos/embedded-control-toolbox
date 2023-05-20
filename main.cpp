#include "hal_st/instantiations/StmEventInfrastructure.hpp"
#include "hal_st/stm32fxxx/DefaultClockDiscoveryF407G.hpp"
#include "services/util/DebugLed.hpp"
#include "infra/timer/Timer.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "control_toolbox/include/pid.hpp"

unsigned int hse_value = 8000000;

int main()
{
    HAL_Init();

    ConfigureDefaultClockDiscoveryF407G();

    main_::StmEventInfrastructure eventInfrastructure;
    hal::GpioPinStm buttonOne{ hal::Port::A, 0, hal::Drive::Default, hal::Speed::Default, hal::WeakPull::Up };
    hal::GpioPinStm ledRed{ hal::Port::D, 13 };
    services::DebugLed debugLed(ledRed, std::chrono::milliseconds(100), std::chrono::milliseconds(1900));
    
    control_toolbox::Pid<float> controller({0.0f, 0.0f, 0.0f}, std::chrono::microseconds(10000), {-10.0f, 10.0f});
    infra::TimerRepeating timer;

    controller.SetPoint(10.0f);
    timer.Start(std::chrono::microseconds(10000), [&controller](){ controller.Process(0); });

    eventInfrastructure.Run();
    __builtin_unreachable();
}
