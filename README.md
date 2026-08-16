# 🤖 D3IM3R Bot

> **D**ifferential **3**-Layer **I**ntelligent **M**obile **3**-nactive **R**obot

Repositorio unificado con todas las iteraciones y versiones del proyecto **D3IM3R Bot**, combinando el código de control ROS 2 Humble y los firmwares para ESP32 desarrollados con PlatformIO y micro-ROS.

---

## 🔤 Desglose del Acrónimo

| Símbolo | Significado | Justificación Técnica |
| :---: | :--- | :--- |
| **D** | **Differential** | Configuración cinemática de dos ruedas motrices |
| **3** ≡ E | **Three-Layer** | Arquitectura robótica jerárquica de tres capas |
| **I** | **Intelligent** | Capacidad de percepción, decisión y adaptación |
| **3** ≡ E | **Enactive** | Inteligencia basada en bucles percepción–acción |
| **M** | **Mobile** | Robot móvil terrestre |
| **R** | **Robot** | Sistema robótico autónomo |

---

## ⚙️ Funcionalidades del Firmware Principal (`d3im3r_bot_all_9`)

El firmware en `d3im3r_bot_all_9` representa la versión de producción del sistema embebido para ESP32 (Denky32). Sus módulos integrados ofrecen las siguientes funcionalidades clave:

### 1. 🤖 Cliente micro-ROS sobre Wi-Fi UDP (`uros.cpp` / `uros.h`)
* **Transporte:** Comunicación por UDP sobre Wi-Fi de alta velocidad hacia el `micro_ros_agent`.
* **Suscripción (`/cmd_vel`):** Recibe órdenes de velocidad lineal y angular (`geometry_msgs/msg/Twist`).
* **Publicación de Odometría (`/odom` / `/odom_raw`):** Envía lecturas continuas del estado del robot (`nav_msgs/msg/Odometry`).
* **Reconexión Automática:** Máquina de estados para reestablecer la sesión micro-ROS automáticamente en caso de desconexión.

### 2. 📐 Cinemática Diferencial (`kinematics.cpp` / `kinematics.h`)
* **Conversión Directa:** Transforma la velocidad lineal $v$ y angular $\omega$ de `/cmd_vel` en velocidades deseadas para cada rueda (izquierda y derecha).
* **Parámetros Físicos:** Basado en el radio de rueda, la distancia entre ejes (*wheelbase*) y la resolución PPR de los encoders.

### 3. 🎯 Control PID de Velocidad (`control.cpp` / `control.h`)
* **Lazo Cerrado:** Algoritmo PID independiente para cada rueda.
* **Respuesta Suave:** Compara la velocidad deseada con la real obtenida por los encoders, aplica corrección PWM con anti-windup y filtrado de ruido.

### 4. ⚡ Driver H-Bridge y PWM (`motors.cpp` / `motors.h`)
* **Control de Motores DC:** Manejo directo de pines de dirección (IN1-IN4 / DIR) y canales PWM (`ledc` de ESP32).
* **Dirección y Frenado:** Gestión del sentido de giro (adelante/atrás) y frenado dinámico.

### 5. ⏱️ Lectura de Encoders por Interrupción (`encoders.cpp` / `encoders.h`)
* **Alta Precisión:** Manejo de interrupciones de hardware (ISR GPIO) para los canales A y B de ambos encoders de cuadratura.
* **Cálculo de Velocidad:** Mide frecuencias y pulsos por revolución (PPR) en tiempo real para estimar velocidad y posición articular.

### 6. 📍 Estimador de Odometría Local (`odometry.cpp` / `odometry.h`)
* **Integración Temporal:** Integra las lecturas de los encoders en el tiempo ($\Delta t$) para calcular la pose $(x, y, \theta)$ del robot.
* **Mensajes ROS 2:** Prepara y formatea los datos de odometría y transformadas TF (`odom` ➔ `base_link`).

### 7. 🛡️ Módulo de Seguridad y Watchdog (`safety.cpp` / `safety.h`)
* **Frenado de Emergencia:** Detención automática inmediata de los motores si no se reciben comandos en `/cmd_vel` dentro del tiempo límite (*timeout* de seguridad).
* **Protección ante Desconexión:** Apaga las salidas de motor si se interrumpe la comunicación Wi-Fi o micro-ROS para evitar comportamientos descontrolados.

### 8. 🌐 Gestión de Red Wi-Fi (`network_config.cpp` / `network_config.h`)
* **Conexión Robusta:** Administración de credenciales de red Wi-Fi e IP del agente micro-ROS.
* **Auto-Reconexión:** Reconexión en segundo plano sin bloquear el bucle principal del robot.

### 9. 🕒 Sincronización de Tiempo (`sync.cpp` / `sync.h`)
* **Sincronización NTP / micro-ROS:** Ajusta el reloj del ESP32 con el agente ROS 2 para asegurar que los estampados de tiempo (*timestamps*) de la odometría sean precisos.

### 10. 🖥️ Interfaz Visual OLED I2C (`oled.cpp` / `oled.h`)
* **Pantalla SSD1306/SH110x:** Despliega en tiempo real la IP asignada, estado de la señal Wi-Fi, estado de la sesión micro-ROS, velocidad actual e indicadores de seguridad.

### 11. 🔋 Monitoreo de Batería y Sensores (`sensors.cpp` / `sensors.h`)
* **Diagnóstico:** Lectura por ADC del estado de voltaje de la batería y diagnósticos secundarios del sistema.

---

## 📂 Contenido del Repositorio (17 Carpetas `d3im3r_bot*`)

### 🚀 1. Paquete ROS 2
* **`d3im3r_bot_bringup/`**: Paquete Python de arranque para ROS 2 Humble (launchers, odometría y teleoperación segura).

### ⚡ 2. Firmwares PlatformIO (ESP32 / micro-ROS)
* **Versión Principal Actual:**
  * **`d3im3r_bot_all_9/`**: Versión más completa y refinada del firmware (control PID, odometría, OLED, Wi-Fi y módulo de seguridad `safety.cpp`).
* **Iteraciones Anteriores de Integración:**
  * `d3im3r_bot_all/` a `d3im3r_bot_all_8/`: Evolución de la integración de componentes y modularización de la red y cinemática.
* **Módulos y Pruebas de Control de Motores:**
  * `d3im3r_bot_uros_motors/`, `_motors2/`, `_motors3/`, `_motors_control/`: Pruebas de encoders y lazos de control PID.
* **Pruebas Iniciales de micro-ROS:**
  * `d3im3r_bot_uros/`, `_uros2/`, `_uros3/`: Integración inicial de comunicación micro-ROS por Wi-Fi y pantalla OLED.

---

## 🛠️ Requisitos e Instalación

* **Firmware:** Usar PlatformIO IDE o CLI sobre la carpeta `d3im3r_bot_all_9/`.
* **ROS 2:** Copiar `d3im3r_bot_bringup/` en el `src` de tu espacio de trabajo `colcon`.

```bash
# Compilación en ROS 2
cd ros2_ws
colcon build --packages-select d3im3r_bot_bringup
source install/setup.bash
```

---

## 📜 Licencia

Distribuido bajo la Licencia MIT.
