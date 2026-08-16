
---

# 🤖 d3im3r Bot — Sistema Integrado (ESP32 + micro-ROS)

Sistema embebido para robot móvil diferencial basado en **ESP32**, con arquitectura modular que integra:

* Control de motores (PI incremental)
* Lectura de encoders
* Sensores (IMU BNO055 + ToF VL53L0X x3)
* Interfaz OLED
* Comunicación con ROS 2 mediante **micro-ROS**

---

## 🧠 Arquitectura del sistema

El robot sigue una arquitectura en capas:

```
+-----------------------------+
|        micro-ROS            |
|  (comunicación ROS 2)       |
+-------------+---------------+
              |
+-------------v---------------+
|        Control PI           |
| (velocidad ruedas)          |
+-------------+---------------+
              |
+-------------v---------------+
|   Motores + Encoders        |
+-------------+---------------+
              |
+-------------v---------------+
|       Sensores              |
| (IMU + ToF + OLED)          |
+-----------------------------+
```

---

## ⚙️ Características principales

✅ Control PI incremental estable
✅ Medición de velocidad en rad/s
✅ Fusión de sensores (orientación + distancia)
✅ Comunicación ROS 2 en tiempo real
✅ Visualización en pantalla OLED
✅ Arquitectura modular y escalable

---

## 📁 Estructura del proyecto

```
src/
├── main.cpp        → Lazo principal del robot
├── app_config.h    → Configuración global

├── motors.*        → Control PWM motores
├── encoders.*      → Lectura de encoders
├── control.*       → Control PI

├── sensors.*       → IMU + ToF
├── oled.*          → Interfaz visual

├── uros.*          → Comunicación micro-ROS
```

---

## 🔁 Flujo de ejecución

El sistema funciona con un lazo principal en `loop()`:

```cpp
encoders_update();
control_update();
motors_update();
```

Y en paralelo:

* Sensores cada `50 ms`
* OLED cada `200 ms`
* micro-ROS en ejecución continua

---

## 📡 Comunicación ROS 2

### 🔽 Suscriptores

| Topic                    | Tipo    | Descripción                |
| ------------------------ | ------- | -------------------------- |
| `/left_wheel_ref_rad_s`  | Float32 | Referencia rueda izquierda |
| `/right_wheel_ref_rad_s` | Float32 | Referencia rueda derecha   |

---

### 🔼 Publicadores

| Topic                   | Tipo    | Descripción         |
| ----------------------- | ------- | ------------------- |
| `/left_wheel_rad_s`     | Float32 | Velocidad medida    |
| `/right_wheel_rad_s`    | Float32 | Velocidad medida    |
| `/left_control_action`  | Float32 | Señal de control    |
| `/right_control_action` | Float32 | Señal de control    |
| `/imu_yaw_rad`          | Float32 | Orientación (yaw)   |
| `/tof_center_m`         | Float32 | Distancia frontal   |
| `/tof_left_m`           | Float32 | Distancia izquierda |
| `/tof_right_m`          | Float32 | Distancia derecha   |

---

## 🎛️ Control de motores

Se implementa un **control PI incremental**:

[
u[k] = u[k-1] + q_0 e[k] + q_1 e[k-1]
]

Ventajas:

* Suavidad en respuesta
* Menor saturación
* Mejor comportamiento en sistemas discretos

---

## 📏 Sensores

### 🧭 IMU — BNO055

* Salida: cuaterniones
* Conversión a yaw (rad)
* Estado de calibración incluido

### 📡 ToF — VL53L0X (x3)

* Centro, izquierda y derecha
* Distancia en metros
* Direcciones I2C configuradas manualmente

---

## 🖥️ OLED

Muestra en tiempo real:

* Distancias ToF
* Ángulo yaw
* Estado de calibración IMU
* Estado de sensores

---

## 🔌 Configuración hardware

### I2C

* SDA → GPIO 21
* SCL → GPIO 22

### ToF (XSHUT)

* Centro → GPIO 5
* Izquierda → GPIO 14
* Derecha → GPIO 23

### Encoders

* Left A/B → GPIO 4 / 2
* Right A/B → GPIO 18 / 19

### Motores

* PWM + dirección configurados en `app_config.h`

---

## 📶 micro-ROS

* Transporte: WiFi
* Agente configurado en:

```cpp
AGENT_IP = 192.168.1.102
AGENT_PORT = 8888
```

---

## 🚀 Ejecución

### 1. Levantar micro-ROS agent

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

### 2. Enviar referencias

```bash
ros2 topic pub /left_wheel_ref_rad_s std_msgs/msg/Float32 "{data: 6.28}"
ros2 topic pub /right_wheel_ref_rad_s std_msgs/msg/Float32 "{data: 6.28}"
```

---

### 3. Ver datos

```bash
ros2 topic echo /imu_yaw_rad
ros2 topic echo /tof_center_m
```

---

## 🧪 Validación

Se verificó:

* Referencia: **6.28 rad/s**
* Medición externa: **≈ 60 RPM**
* Error bajo y comportamiento estable

---

## ⚠️ Consideraciones importantes

* El control **NO depende de micro-ROS**
* El robot sigue funcionando aunque se pierda conexión
* Se implementa parada limpia cuando referencia → 0
* Deadband para evitar vibración en motores

---

## 🔮 Trabajo futuro

* Control de trayectoria (cinemática diferencial)
* SLAM
* Navegación autónoma
* Evitación de obstáculos
* Integración con RL (Reinforcement Learning)

---

## 👨‍💻 Autor

**Deymer Miranda Montoya**
Robótica móvil · Sistemas embebidos · ROS 2

---

## 🧩 Proyecto

**d3im3r**
*Differential Three-layer Intelligent Mobile Robot*

---