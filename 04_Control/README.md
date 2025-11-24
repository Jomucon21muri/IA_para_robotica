# Control y Cinemática

Propósito:
- Implementar controladores para robots: PID, controladores basados en modelos, control predictivo (MPC), inversa cinemática.

Contenido sugerido:
- `low_level/` : controladores para actuadores, bucles de torque/velocidad/posición
- `kinematics/` : modelos directos e inversos para robots móviles y manipuladores
- `mpc/` : ejemplos y notebooks de control predictivo
- `tests/` : bench de estabilidad y respuesta temporal

Cómo usar:
- Interfaces para enviar comandos a simuladores y hardware. Incluir seguridad y límites de saturación.