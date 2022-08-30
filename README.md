# humanArmkinematics
This repository is a working repository for my final masters degree. The aim is to take data from IMU sensors and process it to estimate the position of a human arm in order to control an exoesqueleton. 

## Dependences
**LpSensorLib**
Install LpSensorLib [OPENMAT BINARIES](https://lp-research.com/support/)
libLpSensor depends on libbluetooth.so
- `sudo apt-get update`
- `sudo apt-get install libbluetooth-dev`
- `sudo dpkg -i liblpsensor-1.3.5-Linux.deb`
- `dpkg -L liblpsensor`

**LIBSERIALPORT**
- `sudo apt-get update`
- `sudo apt-get install libserialport-dev`

## Stored Data

In the folder `test/tst_data` some csv data files shall be found with different data to be use in different ways.

| Data                                              | Description |
| :--                                               | ----------- |
| data1_randomImuData.csv                           | Some random IMU data |
| data2_zRotations.csv                              | IMUs quaternions rotations over z axis | 
| data3_yRotations.csv                              | IMUs quaternions rotations over y axis | 
| data4_xRotations.csv                              | IMUs quaternions rotations over x axis | 
| data5_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions | 
| data6_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions | 
| data7_tst_cal_004_onArmArbitraryMotions.csv       | Measures of the IMUs while mounted on the arm and performing arbitrary motions (Longer period) | 

## Gauss-Newton Implementation
The error is calculated in the following way:
$$ error = \omega_{R} \cdot j_n = ([\omega_1]_\epsilon - [\omega_2]_\epsilon) \cdot ([j_1]_\epsilon \times [j_2]_\epsilon) $$

The rotation vectors $j_1$ and $j_2$ are converted to spherical coordinates using these conversions:
$$ j_i = \begin{bmatrix} \sin(\theta_i)\cos(\rho_i)\\ \sin(\theta_i)\sin(\rho_i)\\ \cos(\theta_i)
\end{bmatrix}, \; j_i = \begin{bmatrix} \cos(\theta_i)\\ \sin(\theta_i)\sin(\rho_i)\\ \sin(\theta_i)\cos(\rho_i)
\end{bmatrix} $$
The second option is used when the $|\sin(\theta_i)| < 0.5$ to avoid the singularity in $ \partial j_i/\partial\rho_i = 0$

With these we can arrange the Parameter vector as follows:
$$ \Phi = [\theta_1 \; \rho_1 \;\theta_2 \; \rho_2]^T $$

For $N$ measures, stablish the error vector as $\boldsymbol{e}:= [e_1, e_2, ..., e_N]^T $ and the Jacobian matrix $ J \in \reals^{N \times 4}$. The Jacobian is obtained as $[J]_{ij} = \frac{\partial e_i}{\partial \Phi_j}$ by using the derivatives:
$$ \frac{\partial j}{\partial \theta} = \begin{bmatrix} \cos(\theta)\cos(\rho)\\ \cos(\theta)\sin(\rho)\\ -\sin(\theta)
\end{bmatrix}, \; \frac{\partial j}{\partial \rho} = \begin{bmatrix} -\sin(\theta)\sin(\rho)\\ \sin(\theta)\cos(\rho)\\ 0
\end{bmatrix} $$
And for the second spherical convention we use:
$$ \frac{\partial j}{\partial \theta} = \begin{bmatrix} -\sin(\theta)\\ \cos(\theta)\sin(\rho)\\ \cos(\theta)\cos(\rho)
\end{bmatrix}, \; \frac{\partial j}{\partial \rho} = \begin{bmatrix} 0 \\ \sin(\theta)\cos(\rho)\\ -\sin(\theta)\sin(\rho)
\end{bmatrix} $$

Given that $\frac{\partial j_1}{\partial \theta_2} = \frac{\partial j_1}{\partial \rho_2} = \frac{\partial j_2}{\partial \theta_1} = \frac{\partial j_2}{\partial \rho_1} = 0$, the error derivative for the $i_{th}$ measure becomes:
$$ \frac{\partial e_i}{\partial \Phi} = [
    \omega_{R,i} \cdot (\frac{\partial j_1}{\partial \theta_1} \times j_2), \; 
    \omega_{R,i} \cdot (\frac{\partial j_1}{\partial \rho_1}   \times j_2), \;
    \omega_{R,i} \cdot (j_1 \times \frac{\partial j_2}{\partial \theta_2}), \;
    \omega_{R,i} \cdot (j_1 \times \frac{\partial j_2}{\partial \rho_2})]
$$

Having stablished the error vector and the Jacobian matix we can iterate by using the Gauss-Newton algorithm:
$$ \Phi_{k+1} = \Phi_{k} + (J^TJ)^{-1}J^Te $$
Which can also be understood as using pseudo-inverse of the Jacobian:
$$ \Phi_{k+1} = \Phi_{k} + pinv(J)e $$
