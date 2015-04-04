
float est_ang_velo =1;
float est_ang_acc  =1;



void Kalman_filter(float Torque_Input, float Mea_ang_velo, float Mea_ang_acc )
{
	float K[2][2] = {{ 1.5, 2.5 },{3.75, 6}};
	float F[2][2] = {{ 1.5, 2.5 },{3.75, 6}};
	float H[1][2] = {{ 1.5 ,3}};

	est_ang_velo = F[0][0]*est_ang_velo  +  F[0][1]*est_ang_acc + H[0][0]*Torque_Input + K[0][0]*Mea_ang_velo + K[0][1] * Mea_ang_acc;
	est_ang_acc = F[1][0]*est_ang_velo  +  F[1][1]*est_ang_acc + H[0][1]*Torque_Input + K[1][0]*Mea_ang_velo + K[1][1] * Mea_ang_acc;

 
}  


