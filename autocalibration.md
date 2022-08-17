Three Key Steps are involved in **auto-calibrating** the Firebird V robot. 

 - **Data Acquisition** 
	 - The Firebird V robot is driven using constant PWM signals varying from 100 to 255 in increments of 5 units for fixed durations.
	 -  Each PWM run (for a given duty cycle) is repeated thrice, and the corresponding robot's pose data $\left( \left[x \; y \; \theta \right] \right)$ are obtained from the Vicon Cameras and are recorded.
	 - It is to be noted that forward, and backward motions are carried out using the constant PWM signals.
	 - This data is stored permanently in .csv files to be processed further.
 
 - **Fitting the Data Points in a Circular Arc**
	 - Once the robot's pose data corresponding to each PWM run is available, the path traversed by the robot is fitted into a circular arc.
	 - The circular path curve fitting is formulated as a **non-linear least-squares problem**, and the Gauss-Newton method is employed to solve the same.
The problem formulation is as follows,
		 - Defining the **position states** of the robot during each PWM run as $P_i$ (*retrieved from the .csv files*), the unknown parameters ($\mathbf u$) of the circular arc that are to be computed are,
$$
\mathbf u = \left[ z_1 \quad z_2 \quad r \right]^T  
$$ Here, $\mathbf z = (z_1,z_2)$ is the center of the circular arc and the radius from this point is $r$.
		 - The **squared distances** ($d_i^2$) from each of the recorded position coordinates to the circular arc are given by,
$$
d_i^2 = (||\mathbf z - P_i || - r)^2
$$ The objective is to compute the unknown parameters $\mathbf u$ such that the following sum of the squared distances from each recorded point to the circular arc during the PWM run is minimized (Considering $'m'$ data points)
$$
\sum_{i=1}^m d_i^2
$$

	It is trivial to note that the distance of each recorded point to the circular arc is a function of the unknown parameters, so they are represented as $d_i(\mathbf u)$ subsequently. 
	
	The formulated non-linear least-squares problem is solved iteratively by a sequence of linear least square problems using the Gauss-Newton Method.  Defining,
	$$
	\mathbf d( \mathbf u) = \left[d_1(\mathbf u) \; d_2(\mathbf u) \;....\; d_m(\mathbf u) \right]^T
	$$
	Suppose the solution to the non-linear least-squares problem is $\hat u$, on approximating,
	$$
	\hat u = \tilde u + h
	$$ By using the Taylor Series expansion, we obtain,
	$$
	\mathbf d(\tilde u +h) \simeq \mathbf d(\tilde u) + J(\tilde u)h \approx  0
	$$ The above linear least squares problem is solved iteratively for the vector $h$ as 
	$$
	J(\tilde u)h = -\mathbf d(\tilde u)\\
	\implies h = - \left[J(\tilde u)\right]^{-1}d(\tilde u)
	$$ Here $J(\tilde u)$ is the Jacobian $\left(\frac{\partial\mathbf d(\mathbf u)}{\partial \mathbf u} \right)_{\mathbf u = \tilde u}$ and the Gauss-Newton iteration steps are as follows,
	
	- Solve the above expression for vector $h$
	- Update $\tilde u =\tilde u +h$  

It is to be noted that, since the Jacobian Matrix is non-square, its pseudo-inverse is computed.

During each Gauss-Newton iteration, position samples of batch size =10 are randomly sampled from the available position data for updating the vector $h$, and the iterations are repeated until $||h||$ becomes substantially small (In the order of $10^{-6}$). 

 - **To determine the Linear and Angular Velocities of the Robot**
	- Once the parameters (center and radius) of the circular arc are available, the linear and the angular velocities of the robot during a PWM run are computed as follows,
		- Since constant PWM is applied to the motors during a test PWM run, the robot's path is modeled as a circular arc. The computation of the parameters is as explained earlier. Here, since the robot's path is modeled as a circular arc, the net angular displacement of the robot with respect to the arc's center is identical to the change in the robot's orientation. 
Defining the unit vectors in the $x,y,z$ directions to be $\hat i,\hat j,\hat k$ respectively and the **median sample** from the recorded positions during a given PWM run to be $(x_{med},y_{med})$, (*The median sample is almost angularly equidistant from both the start and end vectors computed from the center of curvature of the arc*)
$$
\vec S = (x_{start} - z_1)\hat i + (y_{start} - z_2)\hat j\\
\vec M = (x_{med} - z_1)\hat i + (y_{med} - z_2)\hat j\\
\vec E = (x_{end} - z_1)\hat i + (y_{end} - z_2)\hat j\\
$$The **conditions** for the **counter-clockwise angular velocity** of the robot are, (Positive $\omega$)
$$
(\vec{S}\times\vec{M})\cdot \hat k >0 \quad ;\quad (\vec{M}\times\vec{E})\cdot \hat k >0 \\
$$ The **conditions** for the **clockwise angular velocity** of the robot are, (Negative $\omega$)
$$
(\vec{S}\times\vec{M})\cdot \hat k <0 \quad ;\quad (\vec{M}\times\vec{E})\cdot \hat k <0 
$$ Using these conditions, positive and negative signs are assigned for counter-clockwise and clockwise angular velocity, respectively. Also, a positive sign is assigned to $v$ when the robot is driven forward, and a negative sign is assigned when it's driven backward.
The angular distance $(\Delta \theta)$ between the starting and the ending vectors is computed as follows,
$$
\Delta \theta = \theta_{SM} + \theta_{ME} \\
\implies \Delta \theta = \cos^{-1}\left( \frac{\vec{S}\cdot\vec{M}}{||\vec{S}||.||\vec{M}||}\right) + \cos^{-1}\left( \frac{\vec{M}\cdot\vec{E}}{||\vec{M}||.||\vec{E}||}\right)
$$ In the above equations $\Delta \theta >0$ since,
$$
0\leq\theta_{SM}\leq \pi \;;\; 0\leq\theta_{ME}\leq  \pi
$$The angular $(\omega)$ and the linear velocity $(v)$ executed by the robot in the duration $(T)$ of the applied PWM is as follows,
$$
\omega = \frac{\Delta \theta}{T} \quad;\quad v = r \times \omega
$$ Here, appropriate signs for $(v,\omega)$ are to be assigned depending on the direction of the motion. The direction of the executed controls $(v,\omega)$ is significant since the robot's motor dynamics for forward and backward motions are considered to be different. Once the executed controls - $(v,\omega)$ are determined, the left and right wheel velocities $(V_L, V_R)$ are determined using the following transformation,
$$
\begin{align*}
\begin{bmatrix}v\\ \omega \end{bmatrix} = &\begin{bmatrix} \frac{r_w}{2} &\frac{r_w}{2}\\
\frac{r_w}{2l} &-\frac{r_w}{2l}
\end{bmatrix}\begin{bmatrix}\omega_R\\ \omega_L \end{bmatrix}\\
\implies \begin{bmatrix}v\\ \omega \end{bmatrix} = &\begin{bmatrix} \frac{1}{2} &\frac{1}{2}\\
\frac{1}{2l} &-\frac{1}{2l}
\end{bmatrix}\begin{bmatrix}V_R\\ V_L \end{bmatrix}
\end{align*}
$$ Here $(r_w,l)$ are the wheel radius and the length of the wheelbase respectively.
 - **Calibrating the Firebird V robot**
	 - The Firebird V robot is calibrated by creating the following maps,
		 - PWM vs Positive $V_R$
		 - PWM vs Negative $V_R$
		 - PWM vs Positive $V_L$
		 - PWM vs Negative $V_L$ 
	
		The above-mentioned maps are generated by virtue of quadratic regression. The quadratic regression model is as follows,
		$$
		PWM = a_0 + a_1V_i + a_2V_i^2 \quad;\quad i = \{L,R \}
		$$ For each PWM sample in the range of $100 \rightarrow255$ in the increments of 5 units, the vectors are arranged as follows,
		$$
		\left[PWM\right] = \begin{bmatrix}100\\105\\.\\.\\.\\.\\255 \end{bmatrix}\;;\;
		\Omega = \begin{bmatrix}1 &V_{i(1)} &V_{i(1)}^2\\
		1 &V_{i(2)} &V_{i(2)}^2\\
		. &. &.\\
		. &. &.\\
		. &. &.\\
		. &. &.\\
		1 &V_{i(32)} &V_{i(32)}^2
		 \end{bmatrix} \;;\; A = \begin{bmatrix}a_0\\a_1\\a_2 \end{bmatrix}
		$$ The unknown parameters $A$ in quadratic regression are obtained by determining the closest fit according to the following equation,
		$$
		\left[PWM\right] = \Omega A\\
		\implies  A = \Omega^{-1}\left[PWM\right] 
		$$
It is to be noted here that $\Omega$ is a non-square matrix and $\Omega^{-1}$ refers to its pseudo-inverse.
	
