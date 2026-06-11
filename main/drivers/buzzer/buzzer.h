#pragma once
#include "esp_err.h"

esp_err_t buzzer_init(void);
void buzzer_set_level(int level);
void buzzer_beep_boot(void);
