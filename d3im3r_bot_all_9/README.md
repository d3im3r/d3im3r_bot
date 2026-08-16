# d3im3r_bot

Firmware modular para el robot móvil diferencial **d3im3r_bot**, desarrollado sobre **ESP32 + PlatformIO + Arduino Framework + micro-ROS**, con control de velocidad por rueda, cinemática diferencial, corrección de avance recto mediante encoders e IMU, odometría ligera, seguridad local por sensores ToF y comunicación con ROS 2 Humble.

El proyecto está diseñado como una plataforma educativa y experimental para robótica móvil, control embebido, micro-ROS, navegación autónoma y aprendizaje por refuerzo.

---

# 1. Descripción general

`d3im3r_bot` es un robot móvil diferencial controlado por un ESP32. El firmware permite recibir comandos desde ROS 2 mediante micro-ROS, controlar la velocidad angular de cada rueda, estimar la pose del robot y publicar información de sensores y estado.

El sistema integra:

- Control independiente de velocidad para rueda izquierda y derecha.
- Control PI incremental en dominio PWM.
- Cinemática inversa para convertir `/cmd_vel` en velocidades de rueda.
- Cinemática directa para estimar velocidad lineal y angular real.
- Corrección de avance recto usando encoders e IMU.
- Odometría ligera publicada como `geometry_msgs/msg/Vector3`.
- Capa de seguridad local por sensores ToF.
- Lectura de encoders incrementales.
- Lectura de IMU BNO055.
- Lectura de sensores ToF VL53L0X.
- Pantalla OLED SH110X.
- Portal de configuración WiFi y agente micro-ROS.
- Comunicación con ROS 2 Humble mediante micro-ROS por WiFi.

---

# 2. Arquitectura general del sistema

La arquitectura del proyecto está organizada por capas.

```text
ROS 2 / micro-ROS
    ↓
/cmd_vel o /wheel_refs_rad_s
    ↓
Filtro de seguridad ToF
    ↓
Cinemática inversa
    ↓
Corrección de avance recto encoder + IMU
    ↓
Control PI incremental por rueda
    ↓
PWM motores
    ↓
Robot físico
    ↓
Encoders + IMU + ToF
    ↓
Cinemática directa
    ↓
Odometría ligera
    ↓
Publicación ROS 2
````

---

## 2.1 Flujo de comando de movimiento

```text
/cmd_vel
    ↓
safety_filter_cmd_vel()
    ↓
kinematics_inverse()
    ↓
sync_apply_hybrid_straight_correction()
    ↓
control_update()
    ↓
motors_update()
```

---

## 2.2 Flujo de estimación de movimiento

```text
encoders_update()
    ↓
kinematics_forward()
    ↓
odometry_update()
    ↓
/odom_pose
/odom_twist
```

---

# 3. Estructura del proyecto

```text
d3im3r_bot/
├── include/
│   ├── app_config.h
│   ├── config/
│   │   ├── pins_config.h
│   │   ├── robot_params.h
│   │   ├── timing_config.h
│   │   ├── control_config.h
│   │   ├── sync_config.h
│   │   ├── network_defaults.h
│   │   ├── ros_topics_config.h
│   │   ├── odom_config.h
│   │   └── safety_config.h
│   ├── control.h
│   ├── encoders.h
│   ├── kinematics.h
│   ├── motors.h
│   ├── network_config.h
│   ├── odometry.h
│   ├── oled.h
│   ├── safety.h
│   ├── sensors.h
│   ├── sync.h
│   └── uros.h
├── src/
│   ├── main.cpp
│   ├── control.cpp
│   ├── encoders.cpp
│   ├── kinematics.cpp
│   ├── motors.cpp
│   ├── network_config.cpp
│   ├── odometry.cpp
│   ├── oled.cpp
│   ├── safety.cpp
│   ├── sensors.cpp
│   ├── sync.cpp
│   └── uros.cpp
├── platformio.ini
├── README.md
├── notes.md
├── plot_robot_pose.py
├── plot_d3im3r_pose.py
└── .gitignore
```

---

# 4. Módulos principales

## 4.1 `main.cpp`

Archivo principal del firmware.

Responsabilidades:

* Inicializar comunicación serial.
* Inicializar bus I2C.
* Inicializar pantalla OLED.
* Inicializar motores.
* Inicializar encoders.
* Inicializar control PI.
* Inicializar sensores.
* Inicializar odometría.
* Inicializar capa de seguridad.
* Inicializar micro-ROS.
* Ejecutar el ciclo principal del robot.

Flujo principal del `loop()`:

```text
1. Actualizar encoders.
2. Actualizar sensores.
3. Actualizar odometría.
4. Ejecutar micro-ROS.
5. Actualizar comando de movimiento.
6. Actualizar control PI.
7. Actualizar motores.
8. Actualizar OLED.
```

---

## 4.2 `motors.cpp` / `motors.h`

Controla directamente los motores DC.

Responsabilidades:

* Configurar pines de dirección.
* Configurar canales PWM.
* Aplicar PWM a motor izquierdo.
* Aplicar PWM a motor derecho.
* Detener motores.
* Actualizar salida física hacia los motores.

El PWM se limita al rango:

```text
-255 ≤ PWM ≤ 255
```

Significado:

```text
PWM > 0   Giro en un sentido
PWM < 0   Giro en sentido contrario
PWM = 0   Motor detenido
```

---

## 4.3 `encoders.cpp` / `encoders.h`

Módulo encargado de leer los encoders incrementales.

Responsabilidades:

* Configurar interrupciones.
* Contar ticks de encoder.
* Calcular velocidad angular de cada rueda.
* Entregar velocidades en rad/s.

Las velocidades medidas se usan en:

* Control PI.
* Sincronía encoder + IMU.
* Cinemática directa.
* Odometría.
* Publicación ROS 2.

---

## 4.4 `control.cpp` / `control.h`

Implementa el control de velocidad por rueda.

El controlador actual es un **PI incremental** en dominio PWM.

La forma implementada es:

```text
u[k] = u[k-1] + Kp · (e[k] - e[k-1]) + Ki · Ts · e[k]
```

Donde:

```text
e[k] = referencia_rad_s - velocidad_medida_rad_s
u[k] = acción PWM calculada
Ts   = tiempo de muestreo real
```

Cada rueda tiene su propio controlador:

```text
Rueda izquierda:
    ref_left_rad_s → PI izquierdo → PWM izquierdo

Rueda derecha:
    ref_right_rad_s → PI derecho → PWM derecho
```

Este módulo no decide hacia dónde debe ir el robot.
Su función es hacer que cada rueda siga su referencia de velocidad angular.

---

## 4.5 `kinematics.cpp` / `kinematics.h`

Contiene la cinemática diferencial del robot.

---

### 4.5.1 Cinemática inversa

Convierte velocidad lineal y angular del robot en velocidades de rueda:

```text
v, ω → ωL, ωR
```

Ecuaciones:

```text
ωL = (v - ω · L/2) / R
ωR = (v + ω · L/2) / R
```

Donde:

```text
v  = velocidad lineal del robot [m/s]
ω  = velocidad angular del robot [rad/s]
L  = distancia entre ruedas [m]
R  = radio de rueda [m]
ωL = velocidad angular rueda izquierda [rad/s]
ωR = velocidad angular rueda derecha [rad/s]
```

Se usa principalmente cuando llega un comando por:

```text
/cmd_vel
```

---

### 4.5.2 Cinemática directa

Convierte velocidades medidas de rueda en velocidad real del robot:

```text
ωL, ωR → v, ω
```

Ecuaciones:

```text
v = R/2 · (ωR + ωL)
ω = R/L · (ωR - ωL)
```

Se usa para:

* Estimar velocidad lineal real.
* Estimar velocidad angular real.
* Alimentar la odometría.

---

## 4.6 `sync.cpp` / `sync.h`

Capa de sincronía para mejorar el avance recto.

Esta capa no reemplaza al PI.
Su función es corregir las referencias de rueda antes de entregarlas al controlador.

Se activa cuando el robot recibe un comando recto:

```text
linear.x ≠ 0
angular.z ≈ 0
```

La corrección combina:

```text
1. Diferencia de velocidades medidas por encoders.
2. Error de orientación medido por IMU.
```

Flujo:

```text
Referencias base de rueda
    ↓
Corrección encoder
    ↓
Corrección IMU
    ↓
Referencias corregidas
    ↓
Control PI incremental
```

Esto permite que el robot conserve mejor su trayectoria recta, incluso si un motor, rueda o superficie produce una desviación leve.

---

## 4.7 `odometry.cpp` / `odometry.h`

Capa de odometría ligera.

Responsabilidades:

* Estimar posición `x`.
* Estimar posición `y`.
* Estimar orientación `theta`.
* Estimar velocidad lineal `v`.
* Estimar velocidad angular `ω`.

La odometría usa:

```text
Encoders → velocidad lineal y angular
IMU      → orientación theta, si está habilitada
```

Pose estimada:

```text
x [m]
y [m]
theta [rad]
```

Velocidad estimada:

```text
v [m/s]
ω [rad/s]
```

La odometría no controla el robot.
Solo estima y publica su estado.

---

## 4.8 `safety.cpp` / `safety.h`

Capa de seguridad local por sensores ToF.

Esta capa se ejecuta en el ESP32 y filtra los comandos recibidos por `/cmd_vel` antes de enviarlos a la cinemática inversa.

Flujo:

```text
/cmd_vel recibido
    ↓
Filtro de seguridad ToF
    ↓
cmd_vel seguro
    ↓
Cinemática inversa
    ↓
Corrección encoder + IMU
    ↓
PI incremental
    ↓
Motores
```

La seguridad local permite que el robot limite o bloquee comandos peligrosos aunque ROS 2 envíe una orden insegura.

---

## 4.9 `sensors.cpp` / `sensors.h`

Módulo de sensores.

Integra:

* IMU BNO055.
* Sensores ToF VL53L0X.
* Estado de calibración.
* Distancias frontales y laterales.

Distribución usada:

```text
tof_distances[0] → sensor frontal
tof_distances[1] → sensor izquierdo
tof_distances[2] → sensor derecho
```

Si un sensor no está disponible, su valor se publica como:

```text
-1.0
```

---

## 4.10 `oled.cpp` / `oled.h`

Controla la pantalla OLED.

Se usa para mostrar:

* Estado de arranque.
* Estado de sensores.
* Estado WiFi.
* Estado micro-ROS.
* Distancias ToF.
* Yaw de la IMU.
* Velocidades de rueda.
* Estado de calibración.

---

## 4.11 `network_config.cpp` / `network_config.h`

Gestiona la conexión WiFi y la configuración del agente micro-ROS.

El proyecto permite configurar:

* SSID WiFi.
* Contraseña WiFi.
* IP del agente micro-ROS.
* Puerto del agente micro-ROS.

La configuración se almacena usando `Preferences` del ESP32.

Si no existe configuración válida, el ESP32 puede levantar un punto de acceso para configuración.

---

## 4.12 `uros.cpp` / `uros.h`

Capa micro-ROS.

Responsabilidades:

* Inicializar red.
* Inicializar nodo micro-ROS.
* Crear publicadores.
* Crear suscriptores.
* Recibir comandos.
* Publicar estado del robot.
* Ejecutar el `executor`.

---

# 5. Parámetros principales del robot

Los parámetros físicos del robot están en:

```text
include/config/robot_params.h
```

Parámetros principales:

```cpp
#define ENCODER_PPR          2112.0f
#define WHEEL_RADIUS_M       0.0217f
#define WHEEL_BASE_M         0.10144f
#define WHEEL_MAX_RAD_S      13.0f
```

| Parámetro         | Descripción                                 |
| ----------------- | ------------------------------------------- |
| `ENCODER_PPR`     | Pulsos efectivos por revolución de rueda    |
| `WHEEL_RADIUS_M`  | Radio de la rueda en metros                 |
| `WHEEL_BASE_M`    | Distancia entre ruedas en metros            |
| `WHEEL_MAX_RAD_S` | Límite máximo de velocidad angular de rueda |

Estos valores afectan directamente:

* Conversión de ticks a rad/s.
* Cinemática inversa.
* Cinemática directa.
* Odometría.
* Control de velocidad.
* Estimación de trayectoria.

---

# 6. Configuración del control PI

Archivo:

```text
include/config/control_config.h
```

Parámetros principales:

```cpp
#define CTRL_LEFT_KP_PWM       9.6667f
#define CTRL_RIGHT_KP_PWM      9.6667f

#define CTRL_LEFT_KI_PWM       9.0f
#define CTRL_RIGHT_KI_PWM      9.0f
```

El controlador es incremental:

```text
u[k] = u[k-1] + Kp · (e[k] - e[k-1]) + Ki · Ts · e[k]
```

Ventajas del PI incremental:

* Es liviano para el microcontrolador.
* No requiere almacenar una integral acumulada grande.
* Trabaja directamente sobre la variación de la acción de control.
* Es adecuado para control discreto en sistemas embebidos.

---

# 7. Configuración de sincronía encoder + IMU

Archivo:

```text
include/config/sync_config.h
```

La sincronía se activa cuando:

```text
|linear.x| > SYNC_MIN_LINEAR_CMD_M_S
|angular.z| < SYNC_ANGULAR_EPS_RAD_S
```

Objetivo:

```text
Mantener el robot avanzando en línea recta.
```

Parámetros principales:

```cpp
#define SYNC_ENABLE                    true
#define SYNC_ENC_ENABLE                true
#define SYNC_IMU_ENABLE                true

#define SYNC_ENC_KP                    0.20f
#define SYNC_IMU_KP                    0.70f
#define SYNC_TOTAL_MAX_CORR_RAD_S      1.20f
```

Recomendaciones:

| Comportamiento                         | Ajuste sugerido                                             |
| -------------------------------------- | ----------------------------------------------------------- |
| Corrige muy poco                       | Aumentar ligeramente `SYNC_IMU_KP`                          |
| Oscila al avanzar                      | Reducir `SYNC_IMU_KP`                                       |
| Corrige en sentido contrario           | Invertir signo de corrección en `sync.cpp`                  |
| Se desvía solo con `/wheel_refs_rad_s` | Revisar motores, ruedas, encoders o calibración             |
| Se desvía solo con `/cmd_vel`          | Revisar cinemática, radio de rueda o distancia entre ruedas |

---

# 8. Configuración de odometría

Archivo:

```text
include/config/odom_config.h
```

Parámetros principales:

```cpp
#define ODOM_USE_IMU_YAW              true
#define ODOM_ZERO_YAW_ON_INIT         true
#define ODOM_MAX_DT_S                 0.20f
#define ODOM_MIN_DT_S                 0.001f
```

Cuando `ODOM_USE_IMU_YAW` está en `true`, la orientación de la odometría se toma desde la IMU.

Cuando `ODOM_ZERO_YAW_ON_INIT` está en `true`, el yaw inicial se toma como cero.

Ejemplo:

```text
imu_yaw real = 1.35 rad
odom theta   = 0.00 rad
```

Esto facilita interpretar la pose desde el punto de inicio del robot.

---

# 9. Configuración de seguridad ToF

Archivo:

```text
include/config/safety_config.h
```

Configuración actual recomendada:

```cpp
#define SAFETY_ENABLE                      true

#define SAFETY_FRONT_STOP_M                0.12f
#define SAFETY_FRONT_SLOW_M                0.30f
#define SAFETY_MIN_SLOW_FACTOR             0.20f

#define SAFETY_SIDE_STOP_M                 0.10f
#define SAFETY_SIDE_SLOW_M                 0.18f

#define SAFETY_FAILSAFE_ON_FRONT_INVALID   true
#define SAFETY_FAILSAFE_ON_SIDE_INVALID    false

#define SAFETY_MAX_LINEAR_M_S              0.15f
#define SAFETY_MAX_ANGULAR_RAD_S           1.20f

#define SAFETY_ALLOW_TURN_ON_FRONT_STOP    true
#define SAFETY_FRONT_STOP_MAX_TURN_RAD_S   0.60f
```

---

## 9.1 Comportamiento frontal

```text
Distancia frontal > 0.30 m
    El robot avanza normal.

0.12 m < distancia frontal <= 0.30 m
    El robot reduce la velocidad lineal progresivamente.

Distancia frontal <= 0.12 m
    El robot bloquea el avance.
```

La zona de reducción permite que el robot disminuya su velocidad antes de llegar a una distancia crítica.

---

## 9.2 Comportamiento lateral

```text
Distancia lateral > 0.18 m
    El giro hacia ese lado se permite normalmente.

0.10 m < distancia lateral <= 0.18 m
    El giro hacia ese lado se reduce.

Distancia lateral <= 0.10 m
    El giro hacia ese lado se bloquea.
```

Convención usada:

```text
angular.z > 0  → giro hacia la izquierda
angular.z < 0  → giro hacia la derecha
```

Por tanto:

```text
Si el sensor izquierdo detecta obstáculo cercano:
    Se limita o bloquea el giro con angular.z > 0.

Si el sensor derecho detecta obstáculo cercano:
    Se limita o bloquea el giro con angular.z < 0.
```

---

## 9.3 Sensores inválidos

Si el sensor frontal no está disponible o entrega una lectura inválida, el firmware bloquea el avance por seguridad:

```cpp
#define SAFETY_FAILSAFE_ON_FRONT_INVALID   true
```

En cambio, si un sensor lateral falla, el robot no bloquea automáticamente todos los giros laterales:

```cpp
#define SAFETY_FAILSAFE_ON_SIDE_INVALID    false
```

Esto evita que el robot quede demasiado limitado si falla solo uno de los sensores laterales.

---

## 9.4 Modo de operación de la seguridad

La capa de seguridad se aplica únicamente al tópico:

```text
/cmd_vel
```

No se aplica sobre:

```text
/wheel_refs_rad_s
```

porque `/wheel_refs_rad_s` es un modo de prueba directa de bajo nivel para motores y controladores.

Para operación normal y segura del robot, se recomienda usar siempre:

```text
/cmd_vel
```

---

# 10. Configuración de red y portal web

Archivo:

```text
include/config/network_defaults.h
```

Parámetros principales:

```cpp
#define CONFIG_BUTTON_PIN 0

#define CONFIG_AP_SSID "d3im3r_bot"
#define AP_PASSWORD "12345678"

#define PORTAL_DOMAIN "d3im3r.bot"
#define PORTAL_IP "192.168.4.1"

#define WIFI_TIMEOUT_MS 15000
#define DNS_PORT 53

#define MICROROS_AGENT_PORT 8888
```

Si no existe configuración de red válida, el ESP32 puede iniciar un punto de acceso:

```text
SSID: d3im3r_bot
PASS: 12345678
WEB : d3im3r.bot
IP  : 192.168.4.1
```

Desde el portal se configura:

* Red WiFi.
* Contraseña WiFi.
* IP del computador donde corre el agente micro-ROS.
* Puerto del agente micro-ROS.

---

# 11. Tópicos ROS 2

Los nombres de los tópicos están centralizados en:

```text
include/config/ros_topics_config.h
```

---

## 11.1 Tópicos publicados por el ESP32

| Tópico                    | Tipo                        | Descripción                        |
| ------------------------- | --------------------------- | ---------------------------------- |
| `/wheel_velocities_rad_s` | `geometry_msgs/msg/Vector3` | Velocidades medidas de las ruedas  |
| `/left_control_action`    | `std_msgs/msg/Float32`      | Acción de control izquierda        |
| `/right_control_action`   | `std_msgs/msg/Float32`      | Acción de control derecha          |
| `/imu_yaw_rad`            | `std_msgs/msg/Float32`      | Orientación yaw medida por IMU     |
| `/tof_distances_m`        | `geometry_msgs/msg/Vector3` | Distancias ToF                     |
| `/odom_pose`              | `geometry_msgs/msg/Vector3` | Pose estimada                      |
| `/odom_twist`             | `geometry_msgs/msg/Vector3` | Velocidad estimada del robot       |
| `/safety_status`          | `geometry_msgs/msg/Vector3` | Estado de la capa de seguridad ToF |

---

## 11.2 `/wheel_velocities_rad_s`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = velocidad rueda izquierda [rad/s]
y = velocidad rueda derecha [rad/s]
z = reservado
```

---

## 11.3 `/left_control_action`

Tipo:

```text
std_msgs/msg/Float32
```

Contenido:

```text
data = acción de control izquierda [%]
```

---

## 11.4 `/right_control_action`

Tipo:

```text
std_msgs/msg/Float32
```

Contenido:

```text
data = acción de control derecha [%]
```

---

## 11.5 `/imu_yaw_rad`

Tipo:

```text
std_msgs/msg/Float32
```

Contenido:

```text
data = yaw de la IMU [rad]
```

---

## 11.6 `/tof_distances_m`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = distancia frontal [m]
y = distancia izquierda [m]
z = distancia derecha [m]
```

Si un sensor no está disponible:

```text
valor = -1.0
```

---

## 11.7 `/odom_pose`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = posición x estimada [m]
y = posición y estimada [m]
z = orientación theta/yaw estimada [rad]
```

---

## 11.8 `/odom_twist`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = velocidad lineal estimada [m/s]
y = velocidad angular estimada [rad/s]
z = reservado
```

---

## 11.9 `/safety_status`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = limited
    0.0 = comando sin limitar
    1.0 = comando limitado por seguridad

y = emergency_stop
    0.0 = sin parada de emergencia
    1.0 = parada de emergencia activa

z = reason
    0 = none
    1 = front slow
    2 = front stop
    3 = front invalid
    4 = left stop
    5 = right stop
    6 = left invalid
    7 = right invalid
    8 = command limited
```

Este tópico permite monitorear cuándo la capa de seguridad está modificando o bloqueando un comando recibido por `/cmd_vel`.

---

## 11.10 Tópicos suscritos por el ESP32

| Tópico              | Tipo                        | Descripción                    |
| ------------------- | --------------------------- | ------------------------------ |
| `/cmd_vel`          | `geometry_msgs/msg/Twist`   | Comando de velocidad del robot |
| `/wheel_refs_rad_s` | `geometry_msgs/msg/Vector3` | Referencias directas de rueda  |

---

## 11.11 `/cmd_vel`

Tipo:

```text
geometry_msgs/msg/Twist
```

Contenido usado:

```text
linear.x  = velocidad lineal [m/s]
angular.z = velocidad angular [rad/s]
```

Este es el modo recomendado para operar el robot.

Cuando se usa `/cmd_vel`, el firmware ejecuta:

```text
/cmd_vel
    ↓
seguridad ToF
    ↓
cinemática inversa
    ↓
corrección encoder + IMU
    ↓
PI incremental
    ↓
motores
```

---

## 11.12 `/wheel_refs_rad_s`

Tipo:

```text
geometry_msgs/msg/Vector3
```

Contenido:

```text
x = referencia rueda izquierda [rad/s]
y = referencia rueda derecha [rad/s]
z = reservado
```

Este tópico se recomienda para pruebas de bajo nivel.

Cuando se usa `/wheel_refs_rad_s`, se envían referencias directamente al PI de cada rueda.

En este modo no se aplica la corrección de línea recta por IMU ni la seguridad ToF, porque se asume que el usuario desea controlar directamente cada rueda.

---

# 12. Requisitos

## 12.1 Hardware

* ESP32 compatible con PlatformIO.
* Driver de motores DC.
* Dos motores DC con encoder.
* IMU BNO055.
* Sensores VL53L0X.
* Pantalla OLED SH110X.
* Batería o fuente de alimentación adecuada.
* Computador con ROS 2 Humble.

---

## 12.2 Software

* PlatformIO.
* Visual Studio Code.
* ROS 2 Humble.
* micro-ROS Agent.
* Python 3, opcional para scripts de visualización.

---

# 13. Instalación del proyecto

Clonar el repositorio:

```bash
git clone <URL_DEL_REPOSITORIO>
cd d3im3r_bot
```

Compilar:

```bash
pio run
```

Cargar al ESP32:

```bash
pio run --target upload
```

Abrir monitor serial:

```bash
pio device monitor
```

O:

```bash
pio device monitor -b 115200
```

---

# 14. Ejecución del agente micro-ROS

En el computador con ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Ejecutar el agente:

```bash
source microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Si el ESP32 está conectado correctamente a la red y configurado con la IP del computador, el agente debería mostrar la creación de sesión micro-ROS.

---

# 15. Verificación en ROS 2

Listar tópicos:

```bash
ros2 topic list
```

Deberían aparecer tópicos como:

```text
/cmd_vel
/wheel_refs_rad_s
/wheel_velocities_rad_s
/left_control_action
/right_control_action
/imu_yaw_rad
/tof_distances_m
/odom_pose
/odom_twist
/safety_status
```

Ver tipos:

```bash
ros2 topic info /odom_pose
ros2 topic info /odom_twist
ros2 topic info /wheel_velocities_rad_s
ros2 topic info /safety_status
```

---

# 16. Comandos de prueba básicos

## 16.1 Probar avance con `/cmd_vel`

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 16.2 Probar avance más rápido

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.12, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 16.3 Probar giro en sitio

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## 16.4 Detener el robot

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

---

## 16.5 Probar referencias directas de rueda

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 \
"{x: 4.0, y: 4.0, z: 0.0}"
```

---

## 16.6 Detener referencias directas

```bash
ros2 topic pub --once /wheel_refs_rad_s geometry_msgs/msg/Vector3 \
"{x: 0.0, y: 0.0, z: 0.0}"
```

---

# 17. Lectura de tópicos

## 17.1 Velocidades de rueda

```bash
ros2 topic echo /wheel_velocities_rad_s
```

Salida esperada:

```text
x: velocidad rueda izquierda [rad/s]
y: velocidad rueda derecha [rad/s]
z: 0.0
```

---

## 17.2 Acciones de control

```bash
ros2 topic echo /left_control_action
```

```bash
ros2 topic echo /right_control_action
```

---

## 17.3 IMU

```bash
ros2 topic echo /imu_yaw_rad
```

---

## 17.4 Sensores ToF

```bash
ros2 topic echo /tof_distances_m
```

Salida:

```text
x: distancia frontal [m]
y: distancia izquierda [m]
z: distancia derecha [m]
```

---

## 17.5 Odometría

Pose:

```bash
ros2 topic echo /odom_pose
```

Twist:

```bash
ros2 topic echo /odom_twist
```

---

## 17.6 Seguridad

```bash
ros2 topic echo /safety_status
```

---

# 18. Interpretación de la odometría

El tópico `/odom_pose` publica:

```text
x = posición estimada en metros
y = posición estimada en metros
z = orientación estimada en radianes
```

Ejemplo:

```text
x: 0.52
y: 0.03
z: 0.01
```

Interpretación:

```text
El robot avanzó aproximadamente 52 cm.
Tiene una desviación lateral aproximada de 3 cm.
Su orientación está casi recta respecto al inicio.
```

El tópico `/odom_twist` publica:

```text
x = velocidad lineal estimada [m/s]
y = velocidad angular estimada [rad/s]
z = reservado
```

Ejemplo:

```text
x: 0.08
y: 0.01
z: 0.0
```

Interpretación:

```text
El robot avanza a 0.08 m/s.
Tiene una velocidad angular residual muy baja.
```

---

# 19. Pruebas de seguridad ToF

## 19.1 Leer estado de seguridad

```bash
ros2 topic echo /safety_status
```

---

## 19.2 Frente libre

Con el robot libre al frente:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.08}, angular: {z: 0.0}}"
```

Esperado:

```text
/safety_status:
x: 0.0
y: 0.0
z: 0.0
```

---

## 19.3 Obstáculo frontal en zona lenta

Ubicar un obstáculo entre 12 cm y 30 cm frente al robot.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Esperado:

```text
El robot reduce su velocidad.

/safety_status:
x: 1.0
y: 0.0
z: 1.0
```

Donde:

```text
z = 1 → SAFETY_REASON_FRONT_SLOW
```

---

## 19.4 Obstáculo frontal en zona de parada

Ubicar un obstáculo a menos de 12 cm frente al robot.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Esperado:

```text
El robot bloquea el avance.

/safety_status:
x: 1.0
y: 1.0
z: 2.0
```

Donde:

```text
z = 2 → SAFETY_REASON_FRONT_STOP
```

---

## 19.5 Obstáculo lateral izquierdo

Ubicar un obstáculo cerca del sensor izquierdo y ejecutar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Esperado:

```text
El giro hacia la izquierda se reduce o se bloquea.
```

---

## 19.6 Obstáculo lateral derecho

Ubicar un obstáculo cerca del sensor derecho y ejecutar:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: -0.5}}"
```

Esperado:

```text
El giro hacia la derecha se reduce o se bloquea.
```

---

# 20. Modos de operación

## 20.1 Modo recomendado: `/cmd_vel`

Usar este modo para navegación normal.

Ventajas:

* Usa seguridad ToF.
* Usa cinemática inversa.
* Permite enviar velocidades del robot.
* Activa corrección de línea recta.
* Es compatible con nodos ROS 2 de navegación o teleoperación.

Ejemplo:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

---

## 20.2 Modo de prueba: `/wheel_refs_rad_s`

Usar este modo para probar motores y controladores de rueda.

Ventajas:

* Permite mandar directamente velocidad angular a cada rueda.
* Sirve para calibración de PI.
* Sirve para pruebas de encoders.

Limitaciones:

* No aplica seguridad ToF.
* No aplica corrección de línea recta con IMU.
* No representa un comando de velocidad del cuerpo del robot.

Ejemplo:

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 \
"{x: 5.0, y: 5.0, z: 0.0}"
```

---

# 21. Flujo recomendado de prueba

## 21.1 Verificar conexión micro-ROS

```bash
source microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

En otra terminal:

```bash
ros2 topic list
```

---

## 21.2 Verificar sensores

```bash
ros2 topic echo /imu_yaw_rad
```

```bash
ros2 topic echo /tof_distances_m
```

---

## 21.3 Verificar encoders

Levantar el robot o mover suavemente las ruedas y ejecutar:

```bash
ros2 topic echo /wheel_velocities_rad_s
```

---

## 21.4 Probar control por rueda

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 \
"{x: 3.0, y: 3.0, z: 0.0}"
```

---

## 21.5 Probar control por `/cmd_vel`

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.08}, angular: {z: 0.0}}"
```

---

## 21.6 Verificar odometría

```bash
ros2 topic echo /odom_pose
```

```bash
ros2 topic echo /odom_twist
```

---

## 21.7 Verificar seguridad

```bash
ros2 topic echo /safety_status
```

---

# 22. Ajuste del PI

Si la velocidad de una rueda oscila:

```text
Reducir Kp.
```

Si la velocidad tarda mucho en alcanzar la referencia:

```text
Aumentar Ki ligeramente.
```

Si la respuesta es lenta pero estable:

```text
Aumentar Kp en pasos pequeños.
```

Si hay tirones fuertes:

```text
Reducir Kp y revisar ruido de encoders.
```

Archivo:

```text
include/config/control_config.h
```

---

# 23. Consideraciones sobre odometría

La odometría de este proyecto es ligera y útil para:

* Visualizar movimiento.
* Registrar trayectorias.
* Analizar desempeño.
* Alimentar scripts externos.
* Desarrollar navegación básica.
* Desarrollar aprendizaje por refuerzo.

Sin embargo, no es equivalente todavía a un `/odom` estándar de ROS 2 con `nav_msgs/msg/Odometry` y TF.

Para integración futura con navegación ROS 2 completa, se recomienda crear un nodo puente en el computador:

```text
/odom_pose + /odom_twist
    ↓
nodo puente ROS 2
    ↓
/odom nav_msgs/msg/Odometry
/tf odom → base_link
```

Esto mantiene liviano el firmware del ESP32 y delega mensajes pesados al computador.

---

# 24. Navegación autónoma futura

La navegación autónoma completa se recomienda implementarla en un paquete ROS 2 Python, no directamente en el ESP32.

División recomendada:

| Responsabilidad           | Ubicación recomendada |
| ------------------------- | --------------------- |
| Lectura de encoders       | ESP32                 |
| Lectura ToF               | ESP32                 |
| Lectura IMU               | ESP32                 |
| PWM motores               | ESP32                 |
| PI de velocidad           | ESP32                 |
| Corrección de línea recta | ESP32                 |
| Seguridad inmediata       | ESP32                 |
| Odometría ligera          | ESP32                 |
| Decisión de movimiento    | ROS 2 Python          |
| Navegación reactiva       | ROS 2 Python          |
| Navegación hacia objetivo | ROS 2 Python          |
| Lógica difusa             | ROS 2 Python          |
| Aprendizaje por refuerzo  | ROS 2 Python          |
| Visualización             | ROS 2 Python / RViz   |
| Registro de datos         | ROS 2 Python          |

Arquitectura futura recomendada:

```text
┌────────────────────────────────────────────┐
│              ROS 2 - Computador            │
│                                            │
│  Nodo navegación autónoma Python           │
│                                            │
│  Lee:                                      │
│  - /odom_pose                              │
│  - /odom_twist                             │
│  - /tof_distances_m                        │
│  - /safety_status                          │
│                                            │
│  Publica:                                  │
│  - /cmd_vel                                │
└───────────────────────┬────────────────────┘
                        │ micro-ROS WiFi
                        ↓
┌─────────────────────────────────────────────┐
│              ESP32 - d3im3r_bot             │
│                                             │
│  Seguridad ToF                              │
│  Cinemática inversa                         │
│  Corrección encoder + IMU                   │
│  PI incremental                             │
│  Motores                                    │
│  Encoders                                   │
│  IMU                                        │
│  Odometría ligera                           │
└─────────────────────────────────────────────┘
```

---

# 25. Archivos de configuración

Resumen de archivos principales en `include/config/`:

| Archivo               | Propósito                      |
| --------------------- | ------------------------------ |
| `pins_config.h`       | Pines del hardware             |
| `robot_params.h`      | Parámetros físicos del robot   |
| `timing_config.h`     | Periodos de actualización      |
| `control_config.h`    | Ganancias y límites del PI     |
| `sync_config.h`       | Corrección de avance recto     |
| `network_defaults.h`  | Portal WiFi y micro-ROS        |
| `ros_topics_config.h` | Nombres de tópicos ROS 2       |
| `odom_config.h`       | Configuración de odometría     |
| `safety_config.h`     | Configuración de seguridad ToF |

---

# 26. Recomendaciones para GitHub

Antes de subir el proyecto, verificar que no se incluyan archivos generados:

```text
.pio/
*.zip
*.bin
*.elf
*.map
```

El `.gitignore` debe excluir al menos:

```gitignore
.pio/
.pioenvs/
.piolibdeps/
*.bin
*.elf
*.map
*.zip
__pycache__/
*.pyc
.DS_Store
Thumbs.db
.env
secrets.h
credentials.h
wifi_credentials.h
```

---

# 27. Estado actual del proyecto

El proyecto cuenta actualmente con:

* Control PI incremental funcional.
* Movimiento estable sin oscilaciones fuertes.
* Corrección de línea recta mediante encoder + IMU.
* Cinemática inversa para `/cmd_vel`.
* Cinemática directa para estimación de velocidades.
* Odometría ligera.
* Capa de seguridad local por sensores ToF.
* Filtro de comandos `/cmd_vel` antes de la cinemática inversa.
* Publicación del estado de seguridad en `/safety_status`.
* Reducción progresiva de velocidad ante obstáculos frontales.
* Bloqueo de avance ante obstáculos frontales críticos.
* Limitación de giro ante obstáculos laterales.
* Publicación de sensores, control y estado.
* Configuración WiFi mediante portal.
* Estructura modular lista para expansión.

---

# 28. Próximas mejoras

Posibles mejoras futuras:

* Crear paquete ROS 2 Python para navegación autónoma.
* Crear nodo `reactive_navigator.py`.
* Crear nodo `go_to_goal.py`.
* Crear nodo puente para convertir `/odom_pose` y `/odom_twist` en `/odom`.
* Publicar transformación TF `odom → base_link`.
* Añadir reset de odometría desde un tópico o servicio.
* Añadir modo de calibración de encoders.
* Añadir registro CSV de pruebas.
* Añadir visualización en RViz.
* Integrar lógica difusa.
* Integrar aprendizaje por refuerzo.
* Añadir detección de bloqueo de rueda.
* Añadir diagnóstico de batería.
* Añadir modo de teleoperación segura.

---

# 29. Licencia

Este proyecto puede ser usado con fines educativos, experimentales y de investigación.

Se recomienda definir una licencia formal antes de publicar el repositorio, por ejemplo:

```text
MIT License
```

---

# 30. Autor

### Proyecto desarrollado por **Deimer Miranda** como parte del desarrollo del robot móvil diferencial **d3im3r_bot**, integrando control embebido, micro-ROS, robótica móvil y ROS 2.
