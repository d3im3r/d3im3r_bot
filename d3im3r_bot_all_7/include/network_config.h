#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>

#define MICROROS_AGENT_PORT 8888

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