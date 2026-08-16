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
| **M** | **Mobile** | Robot móvil terrestre |
| **3** ≡ E | **Enactive** | Inteligencia basada en bucles percepción–acción |
| **R** | **Robot** | Sistema robótico autónomo |

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
