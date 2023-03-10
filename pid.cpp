typedef struct
{
	double Kp;
	double Ki;
	double Kd;
	
	double tau;

	double lim_min;
	double lim_max;

	//sample time
	double T;

	//controller memory
	double integrator;
	double pre_err;
	double differentiator;
	double pre_measure;

	double out;
}pid_controller;

void pid_init(pid_controller *pid);
double pid_update(pid_controller *pid, double setpoint, double measure);

void pid_init(pid_controller *pid)
{
	pid->integrator = 0.0f;
	pid->differentiator = 0.0f;
	pid->pre_err = 0.0f;
	pid->pre_measure = 0.0f;
	pid->out = 0.0f;
	pid->T = 0.02;
	pid->lim_min = -3.0f;
	pid->lim_max = 3.0f;
	pid->Kp = 5.5;
	pid->Ki = 0.008;
	pid->Kd = 0.00;
	pid->tau = 0;
}


double pid_update(pid_controller *pid, double setpoint, double measure)
{
	//Error
	double err = setpoint - measure;

	if(err < -3.15)
	{
		err = (2*3.1415926+err);
	}
	if(err > 3.15)
	{
		err =-(2*3.1415926-err);
	}


	//P
	double proportional = pid->Kp * err;

	//I
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (err+pid->pre_err);

	double limMinInt, limMaxInt;

	if(pid->lim_max > proportional)
	{
		limMaxInt = pid->lim_max - proportional;
	}
	else
	{
		limMaxInt = 0.0f;
	}

	if(pid->lim_min < proportional)
	{
		limMinInt = pid->lim_min - proportional;
	}
	else
	{
		limMinInt = 0.0f;
	}

	if(pid->integrator > limMaxInt)
	{
		pid->integrator = limMaxInt;
	}
	else if(pid->integrator < limMinInt)
	{
		pid->integrator = limMinInt;
	}

	//D
	pid->differentiator = (2.0f * pid->Kd * (measure-pid->pre_measure)
											+	(2.0f * pid->tau - pid->T) * pid->differentiator)
											/ (2.0f * pid->tau + pid->T);

	//SUM
	pid->out = proportional + pid->integrator + pid->differentiator;
	
	if(pid->out > pid->lim_max)
	{
		pid->out = pid->lim_max;
	}
	else if(pid->out < pid->lim_min)
	{
		pid->out = pid->lim_min;
	}

	pid->pre_err = err;
	pid->pre_measure = measure;

	return pid->out;
}



