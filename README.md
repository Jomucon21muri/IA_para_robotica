# 🤖 Inteligencia Artificial para Robots

Este proyecto tiene como objetivo explorar, diseñar e implementar modelos de Inteligencia Artificial aplicados a la robótica, integrando todos los elementos de control necesarios para que los robots puedan percibir, razonar, decidir y actuar en entornos dinámicos.

La IA aplicada a robots no solo permite que ejecuten tareas programadas, sino que aprendan, se adapten y mejoren con la experiencia, aumentando sus capacidades con el tiempo.

Este trabajo constituye la base fundamental para **investigaciones de doctorado (PhD) centradas en la integración de la Inteligencia Artificial en robots de servicio**. El objetivo es avanzar en el desarrollo de sistemas robóticos autónomos capaces de operar eficientemente en entornos humanos, mejorando la interacción, adaptabilidad y rendimiento de los robots de servicio en aplicaciones del mundo real.

## Proyecto Principal: Robot Humanoide

Este repositorio documenta el desarrollo completo de un **robot humanoide desde cero**, integrando:

- **Diseño mecánico**: Estructura con ~25-30 grados de libertad
- **Electrónica**: Sistema de control, sensores y actuadores
- **Software**: Control de movimiento, percepción, inteligencia artificial
- **IA y ML**: Reconocimiento de objetos, voz, aprendizaje por refuerzo
- **Seguridad y ética**: Procedimientos y normativas

### Características del Robot Humanoide

- **Altura**: 150-180 cm
- **Peso**: 8-15 kg
- **DOF**: 25-30 grados de libertad
- **Autonomía**: 2-4 horas
- **Capacidades**:
  - Locomoción bípeda
  - Manipulación de objetos
  - Reconocimiento visual y facial
  - Interacción por voz (español)
  - Aprendizaje y adaptación

## 📂 Estructura del Proyecto

```
IA_para_robotica/
├── 00_Gestion_Proyecto/       # 📋 Gestión, planificación y metodología
├── 01_Percepcion/              # 👁️ Sensores, fusión sensorial, filtrado
├── 02_Localizacion_Mapeo/      # 🗺️ SLAM, odometría, mapeo
├── 03_Planificacion/           # 🎯 Planificación de trayectorias
├── 04_Control/                 # 🎮 Control de movimiento, cinemática
├── 05_Aprendizaje_Maquina/     # 🧠 IA, ML, DL, RL
├── 06_Vision/                  # 👀 Visión por computadora
├── 07_Simulacion_Pruebas/      # 🔬 Gazebo, PyBullet, testing
├── 08_Integracion_Hardware/    # 🔧 Mecánica, electrónica, materiales
├── 09_Comunicaciones_Interfaces/ # 📡 Redes, APIs, interfaces
├── 10_Datasets_Experimentos/   # 📊 Datos de entrenamiento
├── 11_Herramientas_Utilidades/ # 🛠️ Scripts y utilidades
└── 12_Etica_Seguridad/         # 🛡️ Seguridad, ética, normativas
```


## 🚀 Getting Started

### Requisitos Previos

**Software**:
- Python 3.8+
- ROS Noetic (recomendado)
- CAD: SolidWorks / Fusion 360 / FreeCAD
- Simuladores: Gazebo, PyBullet

**Hardware** (para construcción física):
- Microcontroladores: Raspberry Pi 4, Arduino Mega
- Actuadores: 20-30 servomotores
- Sensores: IMU, cámaras, ultrasonido
- Baterías LiPo 3S
- Impresora 3D (o acceso a servicio)

### Instalación

```bash
# Clonar repositorio
git clone https://github.com/Jomucon21muri/IA_para_robotica.git
cd IA_para_robotica

# Crear entorno virtual
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# Instalar dependencias (cuando estén disponibles)
pip install -r requirements.txt

# Configurar ROS (si aplica)
source /opt/ros/noetic/setup.bash
```
## 🛠️ Tecnologías Utilizadas

**Mecánica**:
- SolidWorks, Fusion 360, FreeCAD
- Impresión 3D (FDM), Mecanizado CNC

**Electrónica**:
- Raspberry Pi 4 / Nvidia Jetson Nano
- Arduino Mega / STM32
- KiCad, Fritzing

**Software**:
- Python, C++
- ROS (Robot Operating System)
- OpenCV, PyTorch, TensorFlow

**Simulación**:
- Gazebo, PyBullet, V-REP/CoppeliaSim

**Control de versiones**:
- Git, GitHub

## ⚠️ Seguridad

**IMPORTANTE**: Este proyecto involucra sistemas mecánicos y eléctricos potencialmente peligrosos.

- Siempre seguir procedimientos de seguridad en [12_Etica_Seguridad/](12_Etica_Seguridad/)
- Verificar checklist de seguridad antes de CADA prueba
- Nunca operar sin E-stop accesible
- Nunca dejar robot operando sin supervisión

Ver [safety procedures](12_Etica_Seguridad/README.md) completos.

## 📚 Recursos Adicionales

### Documentación
- [Gestión del Proyecto](00_Gestion_Proyecto/README.md)
- [Recursos de Conocimiento](00_Gestion_Proyecto/recursos_conocimientos.md)
- [Metodología](00_Gestion_Proyecto/metodologia.md)

### Carpetas Técnicas
- [Hardware](08_Integracion_Hardware/README.md): Mecánica y electrónica
- [IA y ML](05_Aprendizaje_Maquina/README.md): Inteligencia artificial
- [Seguridad](12_Etica_Seguridad/README.md): Ética y seguridad

### Comunidades
- r/robotics
- ROS Discourse
- OpenCV Forum
- Arduino Community

### Proyectos Inspiradores
- InMoov: Robot humanoide DIY
- Poppy Project: Plataforma educativa
- Boston Dynamics: Robots avanzados (comercial)

## 📖 Referencias

- **Papers**: Ver carpeta `10_Datasets_Experimentos/papers/`
- **Tutoriales**: Ver carpeta `11_Herramientas_Utilidades/tutorials/`
- **Libros recomendados**:
  - "Modern Robotics" - Lynch & Park
  - "Probabilistic Robotics" - Thrun, Burgard, Fox
  - "Deep Learning" - Goodfellow, Bengio, Courville

## 📧 Contacto

**Autor**: Jomucon21muri

**GitHub**: [@Jomucon21muri](https://github.com/Jomucon21muri)

**Proyecto**: [IA_para_robotica](https://github.com/Jomucon21muri/IA_para_robotica)

## 📄 Licencia

<div align="center">
  <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/">
    <img src="https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png" alt="CC BY-NC-SA 4.0" width="120" height="40">
  </a>
  <br>
  <p>Este proyecto está licenciado bajo <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons BY-NC-SA 4.0</a></p>
</div>

---

<div align="center">
  <p><strong>🤖 Construyendo el futuro de la robótica, un paso a la vez 🤖</strong></p>
  <p><em>"La construcción de un robot humanoide es un proceso complejo que requiere una combinación de habilidades mecánicas, electrónicas y de programación. El resultado final puede ser un robot capaz de realizar tareas impresionantes y mejorar la vida de las personas."</em></p>
</div>

