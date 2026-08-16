#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <Arduino.h>

// =====================================================
// Configuración general del firmware d3im3r_bot
// =====================================================
//
// Este archivo actúa como punto central de inclusión.
// Los parámetros reales están separados por responsabilidad
// dentro de include/config/.
//
// =====================================================

#include "config/pins_config.h"
#include "config/robot_params.h"
#include "config/timing_config.h"
#include "config/control_config.h"
#include "config/sync_config.h"
#include "config/network_defaults.h"
#include "config/ros_topics_config.h"
#include "config/odom_config.h"
#include "config/safety_config.h"

#endif