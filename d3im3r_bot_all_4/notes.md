En la versión final se debe añadir un jumper para activar o desactivar el GPIO2 para el proceso de carga.

## Cómo publicar ahora

En vez de publicar dos tópicos por separado, haces uno solo:

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 "{x: 4.0, y: 4.0, z: 0.0}" -r 10
```

Para girar en sitio, por ejemplo:

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 "{x: -3.0, y: 3.0, z: 0.0}" -r 10
```

Para una curva:

```bash
ros2 topic pub /wheel_refs_rad_s geometry_msgs/msg/Vector3 "{x: 2.0, y: 4.0, z: 0.0}" -r 10
```

---

## Comandos de prueba
Avance recto
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```
Giro en sitio
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 10
```
Curva suave
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}" -r 10
```