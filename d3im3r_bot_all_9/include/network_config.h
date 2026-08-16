#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include "config/network_defaults.h"


struct NetworkConfig
{
    String ssid;
    String password;
    IPAddress agent_ip;
};

bool setupMicroRosNetwork();

bool loadNetworkConfig(NetworkConfig &config);
bool connectToWiFi(const NetworkConfig &config);
void clearNetworkConfig();

#endif