
# 🤖 d3im3r_bot – Control de Velocidad en Bajo Nivel (ESP32 + micro-ROS)

Sistema de control de velocidad para robot diferencial basado en ESP32, utilizando encoders incrementales y un controlador PI discreto, integrado con ROS 2 mediante micro-ROS.

---

## 📌 Descripción

Este módulo implementa el control de velocidad de cada rueda del robot mediante:

- Lectura de encoders (ticks → velocidad)
- Controlador PI incremental en tiempo discreto
- Generación de señales PWM para motores DC
- Comunicación con ROS 2 usando micro-ROS

El sistema recibe referencias de velocidad angular en rad/s y regula automáticamente el PWM para alcanzar dicha velocidad.

---

## ⚙️ Arquitectura

El sistema está dividido en módulos independientes:

```

encoders   → medición de velocidad
control    → cálculo de acción de control (PI)
motors     → aplicación de PWM
uros       → comunicación ROS 2
main       → integración

```

---

## 🔁 Flujo de control

```

Referencia (ROS 2)
↓
control.cpp
↓
PWM (%)
↓
motors.cpp
↓
Motor DC
↓
Encoder
↓
encoders.cpp
↓
Realimentación

```

---

## 📡 Interfaz ROS 2

### 🔽 Suscriptores

| Topic | Tipo | Descripción |
|------|------|------------|
| `/left_wheel_ref_rad_s` | `std_msgs/Float32` | Referencia rueda izquierda [rad/s] |
| `/right_wheel_ref_rad_s` | `std_msgs/Float32` | Referencia rueda derecha [rad/s] |

---

### 🔼 Publicadores

| Topic | Tipo | Descripción |
|------|------|------------|
| `/left_wheel_rad_s` | `std_msgs/Float32` | Velocidad real izquierda |
| `/right_wheel_rad_s` | `std_msgs/Float32` | Velocidad real derecha |
| `/left_control_action` | `std_msgs/Float32` | PWM aplicado [%] |
| `/right_control_action` | `std_msgs/Float32` | PWM aplicado [%] |

---

## 🧮 Modelo de control

Se implementa un controlador PI discreto en forma incremental:

```

u[k] = u[k-1] + q0·e[k] + q1·e[k-1]

````

Donde:

- `e[k] = referencia - velocidad_actual`
- `u[k] = señal de control (PWM %)`

---

## 🔧 Parámetros actuales

```cpp
#define CTRL_LEFT_Q0   9.6667
#define CTRL_LEFT_Q1  -7.8333

#define CTRL_RIGHT_Q0  9.6667
#define CTRL_RIGHT_Q1 -7.8333
````

---

## 📐 Conversión de unidades

El sistema trabaja en:

* Entrada: **rad/s**
* Medición: **rad/s**
* Validación externa: **rpm**

Relación:

```

RPM = (rad/s * 60) / (2π)

```

Ejemplo:

```

6.283 rad/s ≈ 60 RPM

```

---

## 🧪 Validación experimental

Se realizaron pruebas con referencia:

```

6.283 rad/s

```

Resultados:

* Medición externa: **59.5 – 60 RPM**
* Error estacionario: **≈ 0**
* Comportamiento: **estable**

---

## ▶️ Ejecución

### 1. Iniciar agente micro-ROS

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

### 2. Enviar referencia de velocidad

```bash
ros2 topic pub /left_wheel_ref_rad_s std_msgs/msg/Float32 "{data: 6.283}" -r 10
ros2 topic pub /right_wheel_ref_rad_s std_msgs/msg/Float32 "{data: 6.283}" -r 10
```

---

### 3. Monitorear velocidad

```bash
ros2 topic echo /left_wheel_rad_s
ros2 topic echo /right_wheel_rad_s
```

---

## 🧠 Características logradas

* ✔️ Control en lazo cerrado por rueda
* ✔️ Eliminación de error estacionario (PI)
* ✔️ Medición confiable de velocidad
* ✔️ Integración con ROS 2
* ✔️ Arquitectura modular escalable

---

## 🚀 Próximos pasos

* Implementar `/cmd_vel` (cinemática diferencial)
* Estimar odometría (x, y, θ)
* Integración con navegación (`nav2`)
* SLAM y percepción

---

## 👨‍💻 Autor

Desarrollado como parte del proyecto:

**d3im3r_bot**
(Differential Three-layer Intelligent Mobile Robot)

---
