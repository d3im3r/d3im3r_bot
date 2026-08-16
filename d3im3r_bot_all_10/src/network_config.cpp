#include "network_config.h"
#include "oled.h"

#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

// --------------------------------------------------
// Objetos internos
// --------------------------------------------------
static WebServer server(80);
static DNSServer dnsServer;
static Preferences preferences;

static char ssid_buffer[64];
static char password_buffer[64];

static bool agent_only_mode = false;
static NetworkConfig current_config;

// --------------------------------------------------
// Prototipos internos
// --------------------------------------------------
static void saveFullNetworkConfig(const String &ssid, const String &password, const String &agent_ip);
static void saveAgentIPOnly(const String &agent_ip);
static void startFullConfigPortal();
static void startAgentConfigPortal(const NetworkConfig &config);

static String buildFullConfigPage();
static String buildAgentConfigPage(const NetworkConfig &config);
static String buildCss();

static void startCaptiveDNS();
static void configureCaptiveRoutes();
static void redirectToPortal();

static bool configureMicroRosTransport(const NetworkConfig &config);
static bool pingMicroRosAgent();

// --------------------------------------------------
// Función principal de red
// --------------------------------------------------
bool setupMicroRosNetwork()
{
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    delay(50);

    if (digitalRead(CONFIG_BUTTON_PIN) == LOW)
    {
        Serial.println("Botón BOOT presionado. Portal completo.");
        clearNetworkConfig();
        startFullConfigPortal();
        return false;
    }

    if (!loadNetworkConfig(current_config))
    {
        Serial.println("Sin configuración guardada. Portal completo.");
        startFullConfigPortal();
        return false;
    }

    Serial.println("\nConfiguración cargada:");
    Serial.print("SSID: "); Serial.println(current_config.ssid);
    Serial.print("Agent IP: "); Serial.println(current_config.agent_ip);
    Serial.print("Agent Port: "); Serial.println(MICROROS_AGENT_PORT);

    if (!connectToWiFi(current_config))
    {
        Serial.println("Fallo WiFi guardado. Portal completo.");
        startFullConfigPortal();
        return false;
    }

    if (!configureMicroRosTransport(current_config))
    {
        Serial.println("Fallo al configurar transporte micro-ROS. Portal parcial.");
        startAgentConfigPortal(current_config);
        return false;
    }

    if (!pingMicroRosAgent())
    {
        Serial.println("Agent no responde. Portal parcial.");
        startAgentConfigPortal(current_config);
        return false;
    }

    oled_show_message(
        "MICRO-ROS OK",
        "WiFi conectado",
        "Agent encontrado",
        current_config.agent_ip.toString().c_str(),
        "Port: 8888"
    );

    Serial.println("Red WiFi y micro-ROS Agent listos.");
    return true;
}

// --------------------------------------------------
// Cargar configuración NVS
// --------------------------------------------------
bool loadNetworkConfig(NetworkConfig &config)
{
    preferences.begin("netcfg", true);
    String ssid = preferences.getString("ssid", "");
    String password = preferences.getString("pass", "");
    String agent_ip_str = preferences.getString("agent_ip", "");
    preferences.end();

    if (ssid.length() == 0 || agent_ip_str.length() == 0) {
        return false;
    }

    IPAddress agent_ip;
    if (!agent_ip.fromString(agent_ip_str)) {
        return false;
    }

    config.ssid = ssid;
    config.password = password;
    config.agent_ip = agent_ip;
    return true;
}

// --------------------------------------------------
// Conectar a WiFi
// --------------------------------------------------
bool connectToWiFi(const NetworkConfig &config)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid.c_str(), config.password.c_str());

    Serial.print("Conectando a WiFi: ");
    Serial.println(config.ssid);

    oled_show_message("WIFI", "Conectando a:", config.ssid.c_str(), "Espere...", "");

    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_TIMEOUT_MS)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi conectado.");
        Serial.print("IP: "); Serial.println(WiFi.localIP());

        oled_show_message(
            "WIFI OK",
            "Red conectada",
            config.ssid.c_str(),
            WiFi.localIP().toString().c_str(),
            ""
        );
        return true;
    }

    Serial.println("Fallo de conexión WiFi.");
    return false;
}

// --------------------------------------------------
// Borrar configuración guardada
// --------------------------------------------------
void clearNetworkConfig()
{
    preferences.begin("netcfg", false);
    preferences.clear();
    preferences.end();
    Serial.println("Configuración borrada.");
}

// --------------------------------------------------
// Guardar configuraciones
// --------------------------------------------------
static void saveFullNetworkConfig(const String &ssid, const String &password, const String &agent_ip)
{
    preferences.begin("netcfg", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", password);
    preferences.putString("agent_ip", agent_ip);
    preferences.end();
}

static void saveAgentIPOnly(const String &agent_ip)
{
    preferences.begin("netcfg", false);
    preferences.putString("agent_ip", agent_ip);
    preferences.end();
}

// --------------------------------------------------
// Configurar transporte micro-ROS & Ping
// --------------------------------------------------
static bool configureMicroRosTransport(const NetworkConfig &config)
{
    config.ssid.toCharArray(ssid_buffer, sizeof(ssid_buffer));
    config.password.toCharArray(password_buffer, sizeof(password_buffer));

    set_microros_wifi_transports(
        ssid_buffer,
        password_buffer,
        config.agent_ip,
        MICROROS_AGENT_PORT
    );

    delay(2000);
    return true;
}

static bool pingMicroRosAgent()
{
    Serial.println("Verificando micro-ROS Agent...");
    oled_show_message("MICRO-ROS", "Buscando Agent", "Puerto: 8888", "Espere...", "");

    rmw_ret_t ret = rmw_uros_ping_agent(1000, 5);
    if (ret == RMW_RET_OK) {
        Serial.println("micro-ROS Agent encontrado.");
        return true;
    }

    Serial.println("micro-ROS Agent no encontrado.");
    return false;
}

// --------------------------------------------------
// Iniciar Access Point
// --------------------------------------------------
static void setupAccessPoint()
{
    WiFi.disconnect(true);
    delay(300);
    WiFi.mode(WIFI_AP);

    IPAddress ap_ip(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ap_ip, ap_ip, subnet);
    WiFi.softAP(CONFIG_AP_SSID, AP_PASSWORD);
    delay(500);

    startCaptiveDNS();
}

// --------------------------------------------------
// Portales de configuración
// --------------------------------------------------
static void startFullConfigPortal()
{
    agent_only_mode = false;
    setupAccessPoint();

    oled_show_message("CONFIG WIFI", "AP: d3im3r_bot", "PASS: 12345678", "WEB: d3im3r.bot", "IP: 192.168.4.1");

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", buildFullConfigPage());
    });

    configureCaptiveRoutes();

    server.on("/save", HTTP_POST, []() {
        String ssid = server.arg("ssid");
        String password = server.arg("password");
        String agent_ip = server.arg("agent_ip");
        IPAddress test_ip;

        if (ssid.length() == 0 || !test_ip.fromString(agent_ip)) {
            server.send(400, "text/html", "<h2>Datos inválidos</h2><p>Verifica el SSID y la IP del Agent.</p>");
            return;
        }

        saveFullNetworkConfig(ssid, password, agent_ip);
        server.send(200, "text/html", "<h2>Configuración guardada</h2><p>Reiniciando ESP32...</p>");
        oled_show_message("CONFIG OK", "Datos guardados", "Reiniciando...", "", "");
        delay(2000);
        ESP.restart();
    });

    server.onNotFound([]() { redirectToPortal(); });
    server.begin();

    while (true) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }
}

static void startAgentConfigPortal(const NetworkConfig &config)
{
    agent_only_mode = true;
    current_config = config;
    setupAccessPoint();

    oled_show_message("CONFIG AGENT", "AP: d3im3r_bot", "PASS: 12345678", "WEB: d3im3r.bot", "IP: 192.168.4.1");

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", buildAgentConfigPage(current_config));
    });

    configureCaptiveRoutes();

    server.on("/save_agent", HTTP_POST, []() {
        String agent_ip = server.arg("agent_ip");
        IPAddress test_ip;

        if (!test_ip.fromString(agent_ip)) {
            server.send(400, "text/html", "<h2>IP inválida</h2><p>Verifica la IP del micro-ROS Agent.</p>");
            return;
        }

        saveAgentIPOnly(agent_ip);
        server.send(200, "text/html", "<h2>IP del Agent actualizada</h2><p>Reiniciando ESP32...</p>");
        oled_show_message("AGENT OK", "IP actualizada", "Reiniciando...", "", "");
        delay(2000);
        ESP.restart();
    });

    server.onNotFound([]() { redirectToPortal(); });
    server.begin();

    while (true) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }
}

static void startCaptiveDNS()
{
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
}

static void configureCaptiveRoutes()
{
    static const char* routes[] = {
        "/generate_204", "/gen_204", "/fwlink", "/connecttest.txt",
        "/hotspot-detect.html", "/library/test/success.html", "/canonical.html"
    };
    for (const char* route : routes) {
        server.on(route, HTTP_GET, []() { redirectToPortal(); });
    }
}

static void redirectToPortal()
{
    if (agent_only_mode) {
        server.send(200, "text/html", buildAgentConfigPage(current_config));
    } else {
        server.send(200, "text/html", buildFullConfigPage());
    }
}

// --------------------------------------------------
// Generación HTML/CSS simplificada
// --------------------------------------------------
static String buildCss()
{
    return R"rawliteral(<style>
:root{--card:#ffffff;--primary:#2563eb;--primary-dark:#1d4ed8;--text:#1e293b;--muted:#64748b;--border:#dbe3ef;}
*{box-sizing:border-box;}
body{margin:0;min-height:100vh;font-family:Arial,sans-serif;background:linear-gradient(135deg,#0f172a,#1e3a8a);display:flex;align-items:center;justify-content:center;padding:20px;color:var(--text);}
.card{width:100%;max-width:430px;background:var(--card);border-radius:18px;box-shadow:0 20px 40px rgba(0,0,0,.28);overflow:hidden;}
.header{padding:24px;background:linear-gradient(135deg,#2563eb,#1d4ed8);color:white;text-align:center;}
.header h1{margin:0;font-size:26px;}.header p{margin:8px 0 0;font-size:14px;opacity:.92;}
.content{padding:24px;}
.status{background:#eff6ff;border:1px solid #bfdbfe;color:#1e40af;padding:12px;border-radius:12px;font-size:14px;line-height:1.4;margin-bottom:18px;}
.warning{background:#fff7ed;border:1px solid #fed7aa;color:#9a3412;}
label{font-weight:700;font-size:14px;display:block;margin:14px 0 6px;}
input{width:100%;padding:12px 13px;border:1px solid var(--border);border-radius:10px;font-size:15px;outline:none;}
input:focus{border-color:var(--primary);box-shadow:0 0 0 3px rgba(37,99,235,.18);}
button{width:100%;margin-top:22px;padding:13px;background:var(--primary);color:white;border:none;border-radius:12px;font-size:16px;font-weight:700;cursor:pointer;}
button:hover{background:var(--primary-dark);}
.note{margin-top:20px;font-size:14px;color:var(--muted);line-height:1.45;}
.code{background:#0f172a;color:#e2e8f0;padding:12px;border-radius:10px;font-family:Consolas,monospace;font-size:13px;overflow-x:auto;margin-top:8px;}
.small{font-size:12px;color:var(--muted);margin-top:6px;}
.pill{display:inline-block;padding:4px 9px;border-radius:999px;background:#e0f2fe;color:#0369a1;font-weight:700;font-size:12px;}
</style>)rawliteral";
}

static String buildFullConfigPage()
{
    String html = "<!DOCTYPE html><html lang='es'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>d3im3r-bot Config</title>";
    html += buildCss();
    html += R"rawliteral(</head><body><div class='card'><div class='header'><h1>d3im3r-bot</h1><p>Configuración WiFi y micro-ROS</p></div><div class='content'><div class='status'><b>Modo configuración completa</b><br>Configura la red WiFi y la IP del micro-ROS Agent.</div><form action='/save' method='POST'><label>SSID WiFi</label><input name='ssid' placeholder='Ej: Turtlebot' required><label>Password WiFi</label><input name='password' type='password' placeholder='Contraseña de la red'><label>IP del micro-ROS Agent</label><input name='agent_ip' placeholder='Ej: 192.168.1.107' required><div class='small'>Puerto fijo del Agent: <span class='pill'>8888</span></div><button type='submit'>Guardar y reiniciar</button></form><div class='note'><b>Dirección principal:</b><div class='code'>http://d3im3r.bot</div><b>Dirección de respaldo:</b><div class='code'>http://192.168.4.1</div><b>Comando en el PC:</b><div class='code'>ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888</div></div></div></div></body></html>)rawliteral";
    return html;
}

static String buildAgentConfigPage(const NetworkConfig &config)
{
    String html = "<!DOCTYPE html><html lang='es'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>d3im3r-bot Agent Config</title>";
    html += buildCss();
    html += "<div class='card'><div class='header'><h1>d3im3r-bot</h1><p>Actualizar IP del micro-ROS Agent</p></div><div class='content'><div class='status warning'><b>WiFi conectado, Agent no encontrado</b><br>La red guardada funciona, pero el ESP32 no pudo comunicarse con el Agent.</div><div class='note'><b>WiFi conservado:</b><br>";
    html += config.ssid;
    html += "<br><br><b>IP anterior del Agent:</b><br>";
    html += config.agent_ip.toString();
    html += R"rawliteral(<br><br>Puerto fijo: <span class='pill'>8888</span></div><form action='/save_agent' method='POST'><label>Nueva IP del micro-ROS Agent</label><input name='agent_ip' placeholder='Ej: 192.168.1.115' required><button type='submit'>Actualizar IP y reiniciar</button></form><div class='note'><b>Dirección principal:</b><div class='code'>http://d3im3r.bot</div><b>Dirección de respaldo:</b><div class='code'>http://192.168.4.1</div><b>Comando en el PC:</b><div class='code'>ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888</div></div></div></div></body></html>)rawliteral";
    return html;
}