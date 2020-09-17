# Snake_Robot_CPG_STM32
蛇形机器人stm32主控使用Hopf_CPG控制算法

调用方法

	while(!pingServo (id))			               //如果不成功Ping,进入死循环
	{
		printf ("Ping failed\n");   
	}
	printf("Ping OK\n");
	
	
	while (!setServoReturnDelayMicros (id, 0))     //设置反馈延时时间,不成功即进入死循环
	{
		printf ("Set return delay failed\n");
	}
	printf ("Set return delay OK\n");
	
	while (!setServoBlinkConditions (id, SERVO_RANGE_ERROR | SERVO_ANGLE_LIMIT_ERROR))   //设置:当运动边界或者角度范围错误时，LED闪烁
	{
		fflush (stdout);
		printf ("Set blink conditions failed so that errors cannot be detected.\n");
	}
	printf ("Set blink conditions OK\n");
	
	while (!setServoShutdownConditions (id, SERVO_OVERLOAD_ERROR | SERVO_OVERHEAT_ERROR))//设置:当运动边界或者角度范围错误时，撤消扭矩
	{
		fflush (stdout);
		printf ("Set shutdown conditions failed so that torque cannot be removed.\n");
	}
	printf ("Set shutdown conditions OK\n");
	
	while(1)
	{
		delay_ms(200);
		LED1=!LED1;
		delay_ms(200);
		LED1=!LED1;	
	
		torque = 512;
		if (!setServoTorque (id, torque))							//初始化设置扭矩
		{
			fflush (stdout);
			printf ("Set servo torque failed\n");
		}
		printf ("Set torque OK\n");
		if (!getServoTorque (id, &torque))							//得到当前扭矩
		{
			fflush (stdout);
			printf ("Get servo torque failed\n");
		}
		printf ("Get torque OK: servo torque = %u\n", torque);
		
				
		speed = 1023;												//初始化设置最大速度
		if (!setServoMaxSpeed (id, speed))
		{
			fflush (stdout);
			printf ("Set servo max speed failed\n");
		}
		printf ("Set max speed OK\n");
		if (!getServoMaxSpeed (id, &speed))							//得到当前最大速度
		{
			fflush (stdout);
			printf ("Get servo max speed failed\n");
		}
		printf ("Get max speed OK: max speed = %u\n", speed);
		

		if (!getServoCurrentVelocity (id, &currentSpeed))			//得到当前速度
		{	
			fflush (stdout);
			printf ("Get servo current speed failed\n");
		}
		printf ("Get current speed OK: current speed = %d\n", currentSpeed);
		
		
		for (i = 0; i < 5; i++)
		{
			if (!setServoMaxSpeed (id, 128 * (i + 1)))
			{
				fflush (stdout);
				printf ("Set servo max speed failed\n");
				error();
			}
			
			
			if (!getServoAngle (id, &angle))
			{
				printf ("Get servo angle failed\n");
				error();
			}
			
			printf ("Angle = %f\n", angle);
			
			angle = 300 - angle;
			if (!setServoAngle (id, angle))
			{
				printf ("Set servo angle failed\n");
				error();
			}
			
			printf ("Set angle to %f\n", angle);
			
			for (i = 0; i < 1 + (4 - i); i++)
				delay_ms(500);
		}
		
		printf ("Done\n");