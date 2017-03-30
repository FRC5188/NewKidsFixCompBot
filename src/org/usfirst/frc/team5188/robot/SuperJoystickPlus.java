package org.usfirst.frc.team5188.robot;

import edu.wpi.first.wpilibj.Joystick;

public class SuperJoystickPlus extends Joystick {
	
	private boolean flags[] = new boolean[CTRL_BTN.TOTAL_BUTTONS.value + 1];
	   
	    SuperJoystickPlus(int port){
	        super(port); //also need to clear joy-stick class
	        clearButtons();
	    }
	    
	    public double get(CTRL_AXIS ax) {//Last called deadband
	    	double x = this.getRawAxis(ax.value);
	    	return Math.abs(x) < 0.1 ? 0 : x;
	    }
	    
	     public boolean isButtonPushed(CTRL_BTN button){
	    	 int btn = button.value;
	        if (getRawButton(btn)){
	        	if(!flags[btn]){
	        		flags[btn] = true;
	        		return true;
	        	}
	        } else {
	            flags[btn] = false;
	        }
	        return false;
	    }
	     
	     public boolean isButtonHeld(CTRL_BTN button){
	    	 return getRawButton(button.value);
	     }
	    
	    public void clearButtons(){
	    	for(int i=1;i<=CTRL_BTN.TOTAL_BUTTONS.value;i++){
	    		if(flags[i] && !getRawButton(i)){
	    			flags[i] = false;
	    		}
	    	}
	    }
}
