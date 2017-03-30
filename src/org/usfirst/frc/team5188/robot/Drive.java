package org.usfirst.frc.team5188.robot;
import edu.wpi.first.wpilibj.*;

public class Drive  {
	private VictorSP leftDrive;
	private VictorSP rightDrive;
	
	public Drive(int lPWM, int rPWM){
		leftDrive = new VictorSP(lPWM);
		rightDrive = new VictorSP(rPWM);
		
	}
	
	public void setLDrive(double speed){
		if(Control.drive.get(CTRL_AXIS.RTrigger) < 0.4){
			leftDrive.set(speed);//set for practice
		}else{
			rightDrive.set(-speed);
		}
	}
	
	public void setRDrive(double speed){
		if(Control.drive.get(CTRL_AXIS.RTrigger) < 0.4){
			rightDrive.set(-speed);//set for practice
		}else{
			leftDrive.set(speed);
		}
	}
	
	public void stop(){
		setLDrive(0);
		setRDrive(0);
	}
	
}

