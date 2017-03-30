package org.usfirst.frc.team5188.robot;

import com.kauailabs.navx.frc.AHRS;

import modules.PID_Actuator;
import modules.PID_Sensor;

public class GyroSensorActuator implements PID_Actuator, PID_Sensor {
	Drive drive; 
	AHRS gyro;
	public  GyroSensorActuator(Drive drive, AHRS gyro){
		this.drive = drive;
		this.gyro = gyro;
	}
	

	@Override
	public double read() {
		return gyro.getAngle();
	}

	@Override
	public void set(double value) {
		drive.setLDrive(-value);
		drive.setRDrive(value);
	}
}
