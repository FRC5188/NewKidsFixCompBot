package org.usfirst.frc.team5188.robot;

import edu.wpi.first.wpilibj.Encoder;
import modules.PID_Sensor;

public class EncoderSensor implements PID_Sensor {
	private Encoder enc0, enc1;
	private double lastRead = 0;
	
	public EncoderSensor(Encoder enc0, Encoder enc1){
		this.enc0 = enc0;
		this.enc1 = enc1;
	}
	
	public double lastRead(){
		return lastRead;
	}
	
	@Override
	public double read() {
		lastRead = enc1.getDistance();
		return (enc0.getDistance()-lastRead);
	}

}
