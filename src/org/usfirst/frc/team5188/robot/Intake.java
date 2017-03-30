package org.usfirst.frc.team5188.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class Intake {
	private VictorSP motor;

	
public Intake (int motorPWM){
	motor = new VictorSP(motorPWM);
}

public void setIntake(double speed){
	motor.set(speed);
}

}
