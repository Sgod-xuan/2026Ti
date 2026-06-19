#include <stdint.h>

#include "app/car_app.h"
#include "app_config.h"
#include "platform/platform.h"

int main(void)
{
    platform_init();
    car_app_init();

    uint32_t last_ms = platform_millis();

    for (;;)
    {
        const uint32_t now_ms = platform_millis();
        if ((uint32_t)(now_ms - last_ms) >= APP_CONTROL_PERIOD_MS)
        {
            last_ms += APP_CONTROL_PERIOD_MS;
            car_app_update(now_ms);
        }
    }
}
