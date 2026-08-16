#include "network_config.h"
#include "oled.h"

#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

// --------------------------------------------------
// Configuración del portal
// --------------------------------------------------
#define CONFIG_BUTTON_PIN 0

#define CONFIG_AP_SSID "d3im3r_bot"
#define AP_PASSWORD "12345678"

#define PORTAL_DOMAIN "d3im3r.bot"
#define PORTAL_IP "192.168.4.1"

#define WIFI_TIMEOUT_MS 15000
#define DNS_PORT 53

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
static void saveFullNetworkConfig(
    const String &ssid,
    const String &password,
    const String &agent_ip
);

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
// Función principal reutilizable
// --------------------------------------------------
bool setupMicroRosNetwork()
{
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    delay(50);

    if (digitalRead(CONFIG_BUTTON_PIN) == LOW)
    {
        Serial.println("Botón BOOT presionado.");
        Serial.println("Abriendo portal de configuración completa.");

        clearNetworkConfig();
        startFullConfigPortal();

        return false;
    }

    if (!loadNetworkConfig(current_config))
    {
        Serial.println("No hay configuración guardada.");
        Serial.println("Abriendo portal de configuración completa.");

        startFullConfigPortal();

        return false;
    }

    Serial.println();
    Serial.println("Configuración cargada:");
    Serial.print("SSID: ");
    Serial.println(current_config.ssid);
    Serial.print("Agent IP: ");
    Serial.println(current_config.agent_ip);
    Serial.print("Agent Port: ");
    Serial.println(MICROROS_AGENT_PORT);

    if (!connectToWiFi(current_config))
    {
        Serial.println("No fue posible conectar al WiFi guardado.");
        Serial.println("Abriendo portal de configuración completa.");

        startFullConfigPortal();

        return false;
    }

    if (!configureMicroRosTransport(current_config))
    {
        Serial.println("No fue posible configurar transporte micro-ROS.");
        Serial.println("Abriendo portal parcial.");

        startAgentConfigPortal(current_config);

        return false;
    }

    if (!pingMicroRosAgent())
    {
        Serial.println("WiFi conectado, pero el micro-ROS Agent no responde.");
        Serial.println("Abriendo portal parcial para actualizar solo la IP del Agent.");

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
// Cargar configuración desde NVS
// --------------------------------------------------
bool loadNetworkConfig(NetworkConfig &config)
{
    preferences.begin("netcfg", true);

    String ssid = preferences.getString("ssid", "");
    String password = preferences.getString("pass", "");
    String agent_ip_str = preferences.getString("agent_ip", "");

    preferences.end();

    if (ssid.length() == 0 || agent_ip_str.length() == 0)
    {
        return false;
    }

    IPAddress agent_ip;

    if (!agent_ip.fromString(agent_ip_str))
    {
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

    WiFi.begin(
        config.ssid.c_str(),
        config.password.c_str()
    );

    Serial.print("Conectando a WiFi: ");
    Serial.println(config.ssid);

    oled_show_message(
        "WIFI",
        "Conectando a:",
        config.ssid.c_str(),
        "Espere...",
        ""
    );

    unsigned long start_time = millis();

    while (WiFi.status() != WL_CONNECTED &&
           millis() - start_time < WIFI_TIMEOUT_MS)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi conectado correctamente.");
        Serial.print("IP local ESP32: ");
        Serial.println(WiFi.localIP());

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
// Guardar configuración completa
// --------------------------------------------------
static void saveFullNetworkConfig(
    const String &ssid,
    const String &password,
    const String &agent_ip
)
{
    preferences.begin("netcfg", false);

    preferences.putString("ssid", ssid);
    preferences.putString("pass", password);
    preferences.putString("agent_ip", agent_ip);

    preferences.end();
}

// --------------------------------------------------
// Guardar solo IP del Agent
// --------------------------------------------------
static void saveAgentIPOnly(const String &agent_ip)
{
    preferences.begin("netcfg", false);
    preferences.putString("agent_ip", agent_ip);
    preferences.end();
}

// --------------------------------------------------
// Configurar transporte micro-ROS
// --------------------------------------------------
static bool configureMicroRosTransport(const NetworkConfig &config)
{
    config.ssid.toCharArray(
        ssid_buffer,
        sizeof(ssid_buffer)
    );

    config.password.toCharArray(
        password_buffer,
        sizeof(password_buffer)
    );

    set_microros_wifi_transports(
        ssid_buffer,
        password_buffer,
        config.agent_ip,
        MICROROS_AGENT_PORT
    );

    delay(2000);

    return true;
}

// --------------------------------------------------
// Verificar micro-ROS Agent
// --------------------------------------------------
static bool pingMicroRosAgent()
{
    Serial.println("Verificando micro-ROS Agent...");

    oled_show_message(
        "MICRO-ROS",
        "Buscando Agent",
        "Puerto: 8888",
        "Espere...",
        ""
    );

    rmw_ret_t ret = rmw_uros_ping_agent(
        1000,
        5
    );

    if (ret == RMW_RET_OK)
    {
        Serial.println("micro-ROS Agent encontrado.");
        return true;
    }

    Serial.println("micro-ROS Agent no encontrado.");
    return false;
}

// --------------------------------------------------
// Portal completo: WiFi + IP del Agent
// --------------------------------------------------
static void startFullConfigPortal()
{
    agent_only_mode = false;

    WiFi.disconnect(true);
    delay(300);

    WiFi.mode(WIFI_AP);

    IPAddress ap_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(ap_ip, gateway, subnet);
    WiFi.softAP(CONFIG_AP_SSID, AP_PASSWORD);

    delay(500);

    startCaptiveDNS();

    oled_show_message(
        "CONFIG WIFI",
        "AP: d3im3r_bot",
        "PASS: 12345678",
        "WEB: d3im3r.bot",
        "IP: 192.168.4.1"
    );

    Serial.println();
    Serial.println("=================================");
    Serial.println(" Portal completo d3im3r-bot");
    Serial.println("=================================");
    Serial.print("Red WiFi: ");
    Serial.println(CONFIG_AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("URL principal: http://");
    Serial.println(PORTAL_DOMAIN);
    Serial.print("URL respaldo: http://");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, []()
    {
        server.send(
            200,
            "text/html",
            buildFullConfigPage()
        );
    });

    configureCaptiveRoutes();

    server.on("/save", HTTP_POST, []()
    {
        String ssid = server.arg("ssid");
        String password = server.arg("password");
        String agent_ip = server.arg("agent_ip");

        IPAddress test_ip;

        if (ssid.length() == 0 || !test_ip.fromString(agent_ip))
        {
            server.send(
                400,
                "text/html",
                "<h2>Datos inválidos</h2><p>Verifica el SSID y la IP del Agent.</p>"
            );

            return;
        }

        saveFullNetworkConfig(
            ssid,
            password,
            agent_ip
        );

        server.send(
            200,
            "text/html",
            "<h2>Configuración guardada</h2><p>Reiniciando ESP32...</p>"
        );

        oled_show_message(
            "CONFIG OK",
            "Datos guardados",
            "Reiniciando...",
            "",
            ""
        );

        delay(2000);
        ESP.restart();
    });

    server.onNotFound([]()
    {
        redirectToPortal();
    });

    server.begin();

    while (true)
    {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }
}

// --------------------------------------------------
// Portal parcial: solo IP del Agent
// --------------------------------------------------
static void startAgentConfigPortal(const NetworkConfig &config)
{
    agent_only_mode = true;
    current_config = config;

    WiFi.disconnect(true);
    delay(300);

    WiFi.mode(WIFI_AP);

    IPAddress ap_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(ap_ip, gateway, subnet);
    WiFi.softAP(CONFIG_AP_SSID, AP_PASSWORD);

    delay(500);

    startCaptiveDNS();

    oled_show_message(
        "CONFIG AGENT",
        "AP: d3im3r_bot",
        "PASS: 12345678",
        "WEB: d3im3r.bot",
        "IP: 192.168.4.1"
    );

    Serial.println();
    Serial.println("=================================");
    Serial.println(" Portal parcial d3im3r-bot");
    Serial.println("=================================");
    Serial.print("Red WiFi: ");
    Serial.println(CONFIG_AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("URL principal: http://");
    Serial.println(PORTAL_DOMAIN);
    Serial.print("URL respaldo: http://");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, []()
    {
        server.send(
            200,
            "text/html",
            buildAgentConfigPage(current_config)
        );
    });

    configureCaptiveRoutes();

    server.on("/save_agent", HTTP_POST, []()
    {
        String agent_ip = server.arg("agent_ip");

        IPAddress test_ip;

        if (!test_ip.fromString(agent_ip))
        {
            server.send(
                400,
                "text/html",
                "<h2>IP inválida</h2><p>Verifica la IP del micro-ROS Agent.</p>"
            );

            return;
        }

        saveAgentIPOnly(agent_ip);

        server.send(
            200,
            "text/html",
            "<h2>IP del Agent actualizada</h2><p>Reiniciando ESP32...</p>"
        );

        oled_show_message(
            "AGENT OK",
            "IP actualizada",
            "Reiniciando...",
            "",
            ""
        );

        delay(2000);
        ESP.restart();
    });

    server.onNotFound([]()
    {
        redirectToPortal();
    });

    server.begin();

    while (true)
    {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }
}

// --------------------------------------------------
// DNS para Captive Portal
// --------------------------------------------------
static void startCaptiveDNS()
{
    dnsServer.start(
        DNS_PORT,
        "*",
        WiFi.softAPIP()
    );
}

// --------------------------------------------------
// Rutas típicas de Captive Portal
// --------------------------------------------------
static void configureCaptiveRoutes()
{
    server.on("/generate_204", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/gen_204", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/fwlink", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/connecttest.txt", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/hotspot-detect.html", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/library/test/success.html", HTTP_GET, []()
    {
        redirectToPortal();
    });

    server.on("/canonical.html", HTTP_GET, []()
    {
        redirectToPortal();
    });
}

// --------------------------------------------------
// Redirección del Captive Portal
// --------------------------------------------------
static void redirectToPortal()
{
    if (agent_only_mode)
    {
        server.send(
            200,
            "text/html",
            buildAgentConfigPage(current_config)
        );

        return;
    }

    server.send(
        200,
        "text/html",
        buildFullConfigPage()
    );
}

// --------------------------------------------------
// CSS del portal
// --------------------------------------------------
static String buildCss()
{
    String css;

    css += "<style>";

    css += ":root{";
    css += "--card:#ffffff;";
    css += "--primary:#2563eb;";
    css += "--primary-dark:#1d4ed8;";
    css += "--text:#1e293b;";
    css += "--muted:#64748b;";
    css += "--border:#dbe3ef;";
    css += "}";

    css += "*{box-sizing:border-box;}";

    css += "body{";
    css += "margin:0;";
    css += "min-height:100vh;";
    css += "font-family:Arial,Helvetica,sans-serif;";
    css += "background:linear-gradient(135deg,#0f172a,#1e3a8a);";
    css += "display:flex;";
    css += "align-items:center;";
    css += "justify-content:center;";
    css += "padding:20px;";
    css += "color:var(--text);";
    css += "}";

    css += ".card{";
    css += "width:100%;";
    css += "max-width:430px;";
    css += "background:var(--card);";
    css += "border-radius:18px;";
    css += "box-shadow:0 20px 40px rgba(0,0,0,.28);";
    css += "overflow:hidden;";
    css += "}";

    css += ".header{";
    css += "padding:24px;";
    css += "background:linear-gradient(135deg,#2563eb,#1d4ed8);";
    css += "color:white;";
    css += "text-align:center;";
    css += "}";

    css += ".header h1{margin:0;font-size:26px;}";
    css += ".header p{margin:8px 0 0;font-size:14px;opacity:.92;}";
    css += ".content{padding:24px;}";

    css += ".status{";
    css += "background:#eff6ff;";
    css += "border:1px solid #bfdbfe;";
    css += "color:#1e40af;";
    css += "padding:12px;";
    css += "border-radius:12px;";
    css += "font-size:14px;";
    css += "line-height:1.4;";
    css += "margin-bottom:18px;";
    css += "}";

    css += ".warning{";
    css += "background:#fff7ed;";
    css += "border:1px solid #fed7aa;";
    css += "color:#9a3412;";
    css += "}";

    css += "label{";
    css += "font-weight:700;";
    css += "font-size:14px;";
    css += "display:block;";
    css += "margin:14px 0 6px;";
    css += "}";

    css += "input{";
    css += "width:100%;";
    css += "padding:12px 13px;";
    css += "border:1px solid var(--border);";
    css += "border-radius:10px;";
    css += "font-size:15px;";
    css += "outline:none;";
    css += "}";

    css += "input:focus{";
    css += "border-color:var(--primary);";
    css += "box-shadow:0 0 0 3px rgba(37,99,235,.18);";
    css += "}";

    css += "button{";
    css += "width:100%;";
    css += "margin-top:22px;";
    css += "padding:13px;";
    css += "background:var(--primary);";
    css += "color:white;";
    css += "border:none;";
    css += "border-radius:12px;";
    css += "font-size:16px;";
    css += "font-weight:700;";
    css += "cursor:pointer;";
    css += "}";

    css += "button:hover{background:var(--primary-dark);}";

    css += ".note{";
    css += "margin-top:20px;";
    css += "font-size:14px;";
    css += "color:var(--muted);";
    css += "line-height:1.45;";
    css += "}";

    css += ".code{";
    css += "background:#0f172a;";
    css += "color:#e2e8f0;";
    css += "padding:12px;";
    css += "border-radius:10px;";
    css += "font-family:Consolas,monospace;";
    css += "font-size:13px;";
    css += "overflow-x:auto;";
    css += "margin-top:8px;";
    css += "}";

    css += ".small{font-size:12px;color:var(--muted);margin-top:6px;}";

    css += ".pill{";
    css += "display:inline-block;";
    css += "padding:4px 9px;";
    css += "border-radius:999px;";
    css += "background:#e0f2fe;";
    css += "color:#0369a1;";
    css += "font-weight:700;";
    css += "font-size:12px;";
    css += "}";

    css += "</style>";

    return css;
}

// --------------------------------------------------
// Página de configuración completa
// --------------------------------------------------
static String buildFullConfigPage()
{
    String html;

    html += "<!DOCTYPE html>";
    html += "<html lang='es'>";
    html += "<head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>d3im3r-bot Config</title>";
    html += buildCss();
    html += "</head>";

    html += "<body>";
    html += "<div class='card'>";

    html += "<div class='header'>";
    html += "<h1>d3im3r-bot</h1>";
    html += "<p>Configuración WiFi y micro-ROS</p>";
    html += "</div>";

    html += "<div class='content'>";

    html += "<div class='status'>";
    html += "<b>Modo configuración completa</b><br>";
    html += "Configura la red WiFi y la IP del micro-ROS Agent.";
    html += "</div>";

    html += "<form action='/save' method='POST'>";

    html += "<label>SSID WiFi</label>";
    html += "<input name='ssid' placeholder='Ej: Turtlebot' required>";

    html += "<label>Password WiFi</label>";
    html += "<input name='password' type='password' placeholder='Contraseña de la red'>";

    html += "<label>IP del micro-ROS Agent</label>";
    html += "<input name='agent_ip' placeholder='Ej: 192.168.1.107' required>";

    html += "<div class='small'>";
    html += "Puerto fijo del Agent: <span class='pill'>8888</span>";
    html += "</div>";

    html += "<button type='submit'>Guardar y reiniciar</button>";

    html += "</form>";

    html += "<div class='note'>";
    html += "<b>Dirección principal:</b>";
    html += "<div class='code'>http://d3im3r.bot</div>";
    html += "<b>Dirección de respaldo:</b>";
    html += "<div class='code'>http://192.168.4.1</div>";
    html += "<b>Comando en el PC:</b>";
    html += "<div class='code'>ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888</div>";
    html += "</div>";

    html += "</div>";
    html += "</div>";
    html += "</body>";
    html += "</html>";

    return html;
}

// --------------------------------------------------
// Página de configuración parcial del Agent
// --------------------------------------------------
static String buildAgentConfigPage(const NetworkConfig &config)
{
    String html;

    html += "<!DOCTYPE html>";
    html += "<html lang='es'>";
    html += "<head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>d3im3r-bot Agent Config</title>";
    html += buildCss();
    html += "</head>";

    html += "<body>";
    html += "<div class='card'>";

    html += "<div class='header'>";
    html += "<h1>d3im3r-bot</h1>";
    html += "<p>Actualizar IP del micro-ROS Agent</p>";
    html += "</div>";

    html += "<div class='content'>";

    html += "<div class='status warning'>";
    html += "<b>WiFi conectado, Agent no encontrado</b><br>";
    html += "La red guardada funciona, pero el ESP32 no pudo comunicarse con el Agent.";
    html += "</div>";

    html += "<div class='note'>";
    html += "<b>WiFi conservado:</b><br>";
    html += config.ssid;
    html += "<br><br>";
    html += "<b>IP anterior del Agent:</b><br>";
    html += config.agent_ip.toString();
    html += "<br><br>";
    html += "Puerto fijo: <span class='pill'>8888</span>";
    html += "</div>";

    html += "<form action='/save_agent' method='POST'>";

    html += "<label>Nueva IP del micro-ROS Agent</label>";
    html += "<input name='agent_ip' placeholder='Ej: 192.168.1.115' required>";

    html += "<button type='submit'>Actualizar IP y reiniciar</button>";

    html += "</form>";

    html += "<div class='note'>";
    html += "<b>Dirección principal:</b>";
    html += "<div class='code'>http://d3im3r.bot</div>";
    html += "<b>Dirección de respaldo:</b>";
    html += "<div class='code'>http://192.168.4.1</div>";
    html += "<b>Comando en el PC:</b>";
    html += "<div class='code'>ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888</div>";
    html += "</div>";

    html += "</div>";
    html += "</div>";
    html += "</body>";
    html += "</html>";

    return html;
}