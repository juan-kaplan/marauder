# Robótica Móvil: Clase 3

## Objetivos
  - Odometria LiDAR-ICP
  - Nociones de localización Absoluta y Relativa

## ICP

El algoritmo **ICP (*Iterative Closest Point*)** para el caso *Point-to-Point* en 2D busca alinear una nube de puntos origen $S = \{s_i\}_{i=1}^{N}$ con una nube objetivo $T = \{t_j\}_{j=1}^{M}$ mediante una transformación rígida compuesta por una matriz de rotación $R \in SO(2)$ y un vector de traslación $t \in \mathbb{R}^2$.

El algoritmo funciona de manera iterativa alternando entre encontrar correspondencias y resolver un problema de optimización de mínimos cuadrados.

### 1. Asignación de Matches

En cada iteración $k$, la nube $S$ se transforma utilizando la estimación actual $(R_k, t_k)$. Para cada punto $s_i$ alineado, se busca su **vecino más cercano** en la nube objetivo $T$:

$$c(i) = \arg\min_{j} \Vert{} (R_k s_i + t_k) - t_j \Vert{}^2$$

### 2. Función de Error y Minimizador

Una vez fijadas las correspondencias $(s_i, p_i)$ donde $p_i = t_{c(i)}$, se busca la rotación $R$ y traslación $t$ que minimizan la suma de errores cuadráticos de las distancias punto a punto:

$$E(R, t) = \sum_{i=1}^{N} \Vert{} (R s_i + t) - p_i \Vert{}^2$$

### 3. Importancia de la Estimación Inicial (*Initial Guess*)

El algoritmo ICP es un método de **optimización local** basado en el descenso del error por mínimos cuadrados:

* **Mínimos Locales:** La función de costo $E(R,t)$ es no convexa debido a que el paso de asignación de matches es discontinuo. Si la estimación inicial $(R_0, t_0)$ está alejada de la pose real, los puntos de $S$ se emparejarán con puntos incorrectos de $T$, atrapando al algoritmo en un mínimo local erróneo.
* **Radio de Convergencia:** Para geometrías con simetrías o características repetitivas (como la forma de "L" de una esquina), una mala traslación o rotación inicial ($>30^\circ$ o grandes desplazamientos) puede hacer que una pared de la esquina se alinee con la pared equivocada.
* **Estrategias en la práctica:** Generalmente se utiliza odometría (ruedas, IMU) para proporcionar un *initial guess* lo suficientemente cercano al mínimo global.

## Implementación Open3D

**Open3D no es una librería nativa de 2D, pero soporta ICP en 2D procesando los puntos como un caso particular de 3D.**

Internamente, todas las estructuras de datos (`PointCloud`), métodos de búsqueda ($k\text{-d trees}$) y solvers de optimización de Open3D están implementados y optimizados en C++ para **tres dimensiones ($X, Y, Z$)**. Sin embargo, es perfectamente capaz de resolver problemas en 2D aplicando una **restricción geométrica simple**.

### Pasos a tener en cuenta y verificación

1. **Incrustación en 3D ($Z = 0$):**
Las nubes de puntos 2D se representan asignando un valor nulo a la tercera coordenada:

$$\mathbf{p}_{2D} = (x, y) \quad \longrightarrow \quad \mathbf{p}_{3D} = (x, y, 0)$$


2. **Geometría de la Transformación:**
En 2D, la transformación rígida consta de 3 grados de libertad (2 de traslación + 1 de rotación en el plano). En Open3D, esto se mapea automáticamente dentro de la matriz de transformación homogénea de $4 \times 4$:

$$T = \begin{bmatrix}     \cos\theta & -\sin\theta & 0 & t_x \\    \sin\theta & \cos\theta & 0 & t_y \\    0 & 0 & 1 & 0 \\    0 & 0 & 0 & 1     \end{bmatrix}$$

### Instalación

```bash
pip install open3d
```
