# 🤖 D3IM3R Bot

> **D**ifferential **3**-Layer **I**ntelligent **M**obile **3**-nactive **R**obot

Repositorio unificado con todas las iteraciones y versiones del proyecto **D3IM3R Bot**, combinando el paquete de arranque en ROS 2 Humble y los firmwares para ESP32 desarrollados con PlatformIO y micro-ROS.

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

## ⚙️ Funcionalidades del Firmware Principal (`d3im3r_bot_all_10`)

El firmware en **`d3im3r_bot_all_10`** representa la versión de producción optimizada y refactorizada del sistema embebido para ESP32 (Denky32). Sus módulos integrados ofrecen las siguientes funcionalidades clave:

### 1. 🤖 Cliente micro-ROS sobre Wi-Fi UDP (`uros.cpp` / `uros.h`)
* **Transporte:** Comunicación UDP de alta velocidad sobre Wi-Fi hacia el `micro_ros_agent` (Puerto 8888).
* **Suscripción:**
  * `/cmd_vel` (`geometry_msgs/msg/Twist`): Comandos de velocidad lineal ($v$) y angular ($\omega$).
  * `/wheel_refs` (`geometry_msgs/msg/Vector3`): Referencias directas de bajo nivel para las ruedas.
* **Publicación:**
  * `/wheel_velocities` (`geometry_msgs/msg/Vector3`): Velocidades reales medidas de las ruedas en rad/s.
  * `/left_control_action` & `/right_control_action` (`std_msgs/msg/Float32`): Acción de control PI en %.
  * `/imu_yaw` (`std_msgs/msg/Float32`): Orientación Yaw en radianes desde la IMU BNO055.
  * `/tof_distances` (`geometry_msgs/msg/Vector3`): Lectura de los 3 sensores Time-of-Flight en metros (Centro, Izquierda, Derecha).
  * `/odom_pose` & `/odom_twist` (`geometry_msgs/msg/Vector3`): Estimación de pose $(x, y, \theta)$ y velocidades locales.
  * `/safety_status` (`geometry_msgs/msg/Vector3`): Estado de limitación, paro de emergencia y código de razón de seguridad.

### 2. 📐 Cinemática Diferencial (`kinematics.cpp` / `kinematics.h`)
* **Conversión Cinemática:** Transforma la velocidad lineal $v$ y angular $\omega$ en referencias angulares para las ruedas (izquierda y derecha).
* **Cinemática Directa:** Calcula la velocidad lineal y angular del chasis a partir de la rotación real de las ruedas.

### 3. 🎯 Control PI Incremental (`control.cpp` / `control.h`)
* **Lazo Cerrado:** Algoritmo PI incremental desacoplado e independiente para cada rueda.
* **Watchdog de Seguridad:** Lleva automáticamente las referencias a cero si dejan de recibir comandos en `/cmd_vel` o `/wheel_refs` tras el tiempo límite configurado (`MOTOR_TIMEOUT_MS`).

### 4. ⚡ Driver H-Bridge Optimizado (`motors.cpp` / `motors.h`)
* **Control PWM (`ledc` ESP32):** Gestión directa de pines de dirección (IN1/IN2) y canales PWM a 20 kHz.
* **Conmutación Inteligente de Dirección:** Evita retardos de bloqueo en la CPU durante la marcha continua y aplica un *dead-time* microsegundado (300 µs) solo al invertir el sentido de marcha para proteger los transistores del puente H.

### 5. ⏱️ Lectura de Encoders por Interrupción (`encoders.cpp` / `encoders.h`)
* **Alta Precisión:** Interrupciones por cambio de flanco en GPIO (ISR) para los canales A y B de ambos encoders de cuadratura.
* **Muestreo:** Mide pulsos acumulados en periodos de 20 ms con protección de secciones críticas (`portMUX`).

### 6. 📍 Estimador de Odometría Local e Híbrida (`odometry.cpp` / `odometry.h`)
* **Integración Temporal:** Integra velocimetría de rueda y fusiona opcionalmente la orientación de la IMU BNO055 para corregir la deriva angular.

### 7. 🛡️ Filtro de Seguridad Avanzado ToF (`safety.cpp` / `safety.h`)
* **Detección Perimetral:** Monitorea 3 sensores ToF VL53L0X (frontal y laterales).
* **Paro y Reducción Progresiva:** Reduce la velocidad lineal ante obstáculos frontales cercanos y bloquea o limita los giros si detecta obstrucciones laterales.

### 8. 🌐 Portal Cautivo y Red Wi-Fi (`network_config.cpp` / `network_config.h`)
* **Portal de Configuración:** Genera una red AP (`d3im3r_bot`) y un portal cautivo web en `http://d3im3r.bot` (o `192.168.4.1`) para configurar el SSID, password y la IP del agente micro-ROS.
* **Persistencia NVS:** Guarda credenciales de forma permanente usando `Preferences.h`.

### 9. 🕒 Sincronización de Línea Recta (`sync.cpp` / `sync.h`)
* **Corrección Híbrida:** Mantiene la trayectoria recta del robot comparando la diferencia de velocidad entre encoders y la desviación angular reportada por la IMU.

### 10. 🖥️ Interfaz Visual OLED I2C (`oled.cpp` / `oled.h`)
* **Pantalla SH1106G (128x64):** Muestra diagnósticos de arranque, IP de red, estado de sesión micro-ROS, lecturas de sensores y velocidad actual.

### 11. 🔋 Gestión de Sensores I2C (`sensors.cpp` / `sensors.h`)
* **Bus I2C:** Administración y reasignación dinámica de direcciones I2C para los 3 sensores VL53L0X (`0x30`, `0x31`, `0x32`) y la IMU BNO055 (`0x28`).

---

## 📈 Historial de Refactorización (`d3im3r_bot_all_9` ➔ `d3im3r_bot_all_10`)

La versión `d3im3r_bot_all_10` introduce mejoras sustanciales en arquitectura, consumo de recursos y estabilidad en tiempo real:

1. **Optimización de Memoria y Portal Cautivo (`network_config.cpp`):**
   * Eliminadas más de 400 líneas de código mediante el uso de *raw string literals* para HTML/CSS.
   * Reducción de la fragmentación en la RAM y eliminación de redefiniciones `#define` duplicadas.
2. **Desacoplamiento DRY en Control (`control.cpp`):**
   * Unificación de los controladores izquierdo y derecho bajo la estructura genérica `PIController`.
3. **Optimización de Tiempos Real-Time (`motors.cpp`):**
   * Eliminados retardos de bloqueo innecesarios (`delayMicroseconds(300)`) en cada actualización PWM durante la marcha continua.
4. **Cálculo Eficiente en Encoders (`encoders.cpp`):**
   * Eliminación de cálculos flotantes redundantes de RPM y m/s en el bucle de interrupción, realizándolos bajo demanda (*on-demand*).
5. **Corrección de Lógica de Seguridad (`safety.cpp`):**
   * Corregida la asignación de motivos de limitación cuando el robot entra en zona de reducción progresiva lateral (*slow zone*).
6. **Métricas de Compilación:**
   * **Flash:** Reducción de **~4.5 KB** de memoria programa (`964,257 bytes` vs `968,765 bytes`).
   * **Tiempo de Compilación incremental:** 9.19 segundos.

---

## 📂 Contenido del Repositorio (18 Carpetas)

### 🚀 1. Paquete ROS 2
* **`d3im3r_bot_bringup/`**: Paquete Python de arranque para ROS 2 Humble (launchers, odometría y teleoperación segura).

### ⚡ 2. Firmwares PlatformIO (ESP32 / micro-ROS)
* **Versión Principal Actual:**
  * **`d3im3r_bot_all_10/`**: Firmware de producción refactorizado, optimizado y libre de código muerto.
  * **`d3im3r_bot_all_9/`**: Iteración previa de producción.
* **Iteraciones Anteriores de Integración:**
  * `d3im3r_bot_all/` a `d3im3r_bot_all_8/`: Evolución del desarrollo, pruebas de integración y modularización.
* **Módulos y Pruebas de Control de Motores:**
  * `d3im3r_bot_uros_motors/`, `_motors2/`, `_motors3/`, `_motors_control/`: Pruebas de encoders y lazos de control PID.
* **Pruebas Iniciales de micro-ROS:**
  * `d3im3r_bot_uros/`, `_uros2/`, `_uros3/`: Integración inicial de comunicación micro-ROS por Wi-Fi y pantalla OLED.

---

## 🛠️ Requisitos e Instalación

### Compilación y Carga del Firmware (PlatformIO)
Para compilar y cargar la versión optimizada en el ESP32 (Denky32):

```bash
cd d3im3r_bot_all_10
pio run --target upload
```

### Espacio de Trabajo ROS 2
Copiar `d3im3r_bot_bringup/` dentro del directorio `src` de tu workspace de ROS 2:

```bash
cd ros2_ws
colcon build --packages-select d3im3r_bot_bringup
source install/setup.bash
```

---

## 📜 Licencia

Distribuido bajo la Licencia MIT.
