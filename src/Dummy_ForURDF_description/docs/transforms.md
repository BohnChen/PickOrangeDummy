# Transformation Matrices - Dummy_ForURDF

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | J1_2 |
| $L_{2}$ | J2J3_2 |
| $L_{3}$ | J4_2 |
| $L_{4}$ | 35_20Motor_Assembly_2 |
| $L_{5}$ | 35_20Motor_Assembly_1_2 |
| $L_{6}$ | 3 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | _1 | revolute (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | _2 | revolute (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | _3 | revolute (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | _4 | revolute (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | _5 | revolute (rad) | $L_{4}$ | $L_{5}$ |
| $q_{6}$ | _6 | continuous (rad) | $L_{5}$ | $L_{6}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [revolute] _1 (q1)
      L1: J1_2
        +-- [revolute] _2 (q2)
            L2: J2J3_2
              +-- [revolute] _3 (q3)
                  L3: J4_2
                    +-- [revolute] _4 (q4)
                        L4: 35_20Motor_Assembly_2
                          +-- [revolute] _5 (q5)
                              L5: 35_20Motor_Assembly_1_2
                                +-- [continuous] _6 (q6)
                                    L6: 3
```

## Transforms

## _1

$L_{0}$ **base_link** -> $L_{1}$ **J1_2** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (-0.565, -0.0705, 0.035) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 5.934119] rad ([0deg, 340deg])

### Local Transform

$$
T^{0}_{1}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & -0.565 \\
s_{1} & c_{1} & 0 & -0.0705 \\
0 & 0 & 1 & 0.035 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## _2

$L_{1}$ **J1_2** -> $L_{2}$ **J2J3_2** (revolute)
  Variable: $q_{2}$

- **origin xyz**: (-0.014, 0.04203, 0.071252) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [4.625123, 6.283185] rad ([265deg, 360deg])

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.014 \\
1 & 0 & 0 & 0.04203 \\
0 & -1 & 0 & 0.071252 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## _3

$L_{2}$ **J2J3_2** -> $L_{3}$ **J4_2** (revolute)
  Variable: $q_{3}$

- **origin xyz**: (0.313535, 0.035019, -1.097) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-2.530727, 0] rad ([-145deg, 0deg])

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.313535 \\
0 & -1 & 0 & 0.035019 \\
0 & 0 & -1 & -1.097 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## _4

$L_{3}$ **J4_2** -> $L_{4}$ **35_20Motor_Assembly_2** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (-0.255471, 0.57391, -0.377137) m
- **origin rpy**: (0.261799, -1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-3.141593, 3.141593] rad ([-180deg, 180deg])

### Local Transform

$T^{3}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.258819 & -0.965926 & -0.255471 \\
0 & 0.965926 & -0.258819 & 0.57391 \\
1 & 0 & 0 & -0.377137 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## _5

$L_{4}$ **35_20Motor_Assembly_2** -> $L_{5}$ **35_20Motor_Assembly_1_2** (revolute)
  Variable: $q_{5}$

- **origin xyz**: (-0.142941, 0.0186, 0.344777) m
- **origin rpy**: (-1.570796, 1.308997, -1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.658063, 1.658063] rad ([-95deg, 95deg])

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & -0.142941 \\
-0.258819 & 0.965926 & 0 & 0.0186 \\
-0.965926 & -0.258819 & 0 & 0.344777 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## _6

$L_{5}$ **35_20Motor_Assembly_1_2** -> $L_{6}$ **3** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (-0.037344, -0.02802, -0.06037) m
- **origin rpy**: (0.261799, -1.570796, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -0.258819 & -0.965926 & -0.037344 \\
0 & 0.965926 & -0.258819 & -0.02802 \\
1 & 0 & 0 & -0.06037 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{J2J3_2})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{J4_2})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{35_20Motor_Assembly_2})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{35_20Motor_Assembly_1_2})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{3})$$

