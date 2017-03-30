package org.usfirst.frc.team5188.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PDB extends PowerDistributionPanel {
	SuperJoystickPlus joys = new SuperJoystickPlus(0);
	int leftMotors = 14;
	int rightMotors = 1;
	public void rumbleController(){
		double Lcurrent = this.getCurrent(leftMotors);	
		double Rcurrent = this.getCurrent(rightMotors);	
		//get speed and if speed is < given amount && current is high then rumble 
		//if (Lcurrent > 0){
			joys.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
			joys.setRumble(GenericHID.RumbleType.kRightRumble, 1);

		//}

		System.out.println("Left Current = " + Lcurrent);
		System.out.println("Right Current = " + Rcurrent);

		//return spiked;
	}
	
}
