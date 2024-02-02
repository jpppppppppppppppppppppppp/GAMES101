# GAMES101

Möller Trumbore Algorithm


$$
\vec{O}+t\vec{D} = (1-b_1-b_2)\vec{P}_0 + b_1 \vec{P}_1 + b_2 \vec{P}_2\\
\left[\begin{matrix}t\\b_1\\b_2\end{matrix}\right] = \frac{1}{\vec{S}_1\cdot \vec{E}_1}\left[\begin{matrix}\vec{S}_2\cdot \vec{E}_2\\\vec{S}_1\cdot \vec{S}\\\vec{S}_2\cdot \vec{D}\end{matrix}\right]\\
\begin{aligned}\text{Where}:& \vec{E}_1 = \vec{P}_1-\vec{P}_0\\ & \vec{E}_2 = \vec{P}_2-\vec{P}_0\\ & \vec{S} = \vec{O}-\vec{P}_0\\ & \vec{S}_1 = \vec{D} \times \vec{E}_2\\&\vec{S}_2 = \vec{S}-\vec{E}_1\end{aligned}
$$
