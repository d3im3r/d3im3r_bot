# Topics ROS 2 finales

- Publicador de velocidad (RPM y Rad/s) subscriptor a comandos de velocidad

## Comandos

```bash
/cmd_left_pwm
/cmd_right_pwm
```

## Carga manual de conteos

```bash
/load_count_left
/load_count_right
```

## Estado publicado

```bash
/left_ticks
/right_ticks
/left_wheel_rad_s
/right_wheel_rad_s
/left_wheel_rpm
/right_wheel_rpm
/left_wheel_m_s
/right_wheel_m_s
/left_pwm_applied
/right_pwm_applied
```

---

# Pruebas rápidas

## 1. Ver que el nodo existe

```bash
ros2 node list
```

## 2. Mandar PWM a una rueda

```bash
ros2 topic pub /cmd_left_pwm std_msgs/msg/Float32 "{data: 20.0}" -r 5
```

```bash
ros2 topic pub /cmd_right_pwm std_msgs/msg/Float32 "{data: 20.0}" -r 5
```

## 3. Ver velocidad

```bash
ros2 topic echo /left_wheel_rpm
ros2 topic echo /right_wheel_rpm
```

## 4. Frenar

```bash
ros2 topic pub /cmd_left_pwm std_msgs/msg/Float32 "{data: 0.0}" --once
ros2 topic pub /cmd_right_pwm std_msgs/msg/Float32 "{data: 0.0}" --once
```

---

# Observación importante

Como tienes `MOTOR_TIMEOUT_MS = 500`, si dejas de publicar comandos, el motor se apaga solo. Eso está bien para seguridad.

