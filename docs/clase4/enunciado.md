# Robótica Móvil: Clase 3

## Alcances
- Introducción a nociones de control de un robot diferencial
- Navegación autónoma basada en **Teach and Repeat (TnR)**.

---

## Objetivo Principal

Diseñar e implementar un sistema de navegación autónoma Teach and Repeat para un robot móvil equipado únicamente con odometría de ruedas y un sensor LiDAR 2D.

> ⚠️ **ACLARACIÓN IMPORTANTE SOBRE EL ALCANCE:**
> * **NO se requiere ni se evalúa la generación o planificación de trayectorias:** El robot no debe calcular nuevas rutas, evitar obstáculos dinámicos globales mediante planificadores de caminos (A*, Dijkstra, TEB, etc.), ni tomar decisiones de ruteo abstracto. La trayectoria a seguir estará completamente definida por la demostración previa realizada en la fase *Teach*.

---

### 3. Requerimientos del Sistema

### 3.1. Fase de Aprendizaje (*Teach*)
* **Construcción del Mapa de Referencia:**

  * Almacenar los keyframes $a_k = (l_k))$, donde $l_k$ es la lectura cruda del LiDAR 2D.

  * Almacenar las transformaciones relativas entre keyframes $T_{(a,a+1)} = x_{a+1} \ominus x_a$.

  * Implementar un criterio de muestreo espacial parametrizable.

### 3.2. Fase de Repetición (*Repeat*)

* **Localización Relativa por Scan Matching:**

  * Para cada instante $t$, seleccionar la referencia $a = (l_a)$ más cercana.
  
  * Definir el siguiente keyframe como objetivo (*setpoint*): $s(t) = x_{a+1}$.
  
  * Reducir el error de control en el espacio de configuración: $e(t) = s_a(t) \ominus x(t)$.
