	package org.usfirst.frc.team5188.robot;

import modules.PID_Sensor;

public class PixySensor implements PID_Sensor {
	M_I2C i2c;
	
	public PixySensor(M_I2C i2c){
		this.i2c = i2c;
	}
	
	@Override
	public double read() {
		return i2c.getPixy().y;
	}

}
