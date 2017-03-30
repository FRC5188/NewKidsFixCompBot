package org.usfirst.frc.team5188.robot;

import modules.PID_Actuator;

public class EncoderActuator implements PID_Actuator {
	
	private boolean enabled = true;
	private Drive drive;
	
	public EncoderActuator(Drive drive){
		this.drive = drive;
	}
	
	public void setEnabled(boolean value){
		enabled = value;
	}
	
	@Override
	public void set(double value) {
		if(!enabled)
			return;
		drive.setLDrive(.3 - value);
		drive.setRDrive(.3 + value);
		System.out.println(value);
	}

}
