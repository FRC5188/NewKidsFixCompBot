
package org.usfirst.frc.team5188.robot;

import java.util.Timer;
import java.util.TimerTask;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import modules.Pats_PID_Controller;

public class Robot extends IterativeRobot {
	
	final String defaultAuto = "default";
	final String gear = "gear";
	final String baseline = "baseline";
	final String gearTime = "gearTime";
	int counter = 0;
	String auto;
	double error = 0;
	
	Encoder rEncoder = new Encoder(0,1);
	Encoder lEncoder = new Encoder(2,3);
	Encoder shooterEncoder = new Encoder(4,5);
	
	M_I2C i2c = new M_I2C();
	AHRS gyro = new AHRS(SerialPort.Port.kMXP);
	PixySensor pidPixy = new PixySensor(i2c);
	
	DriverStation driverStation = DriverStation.getInstance();
	
	Drive drive = new Drive(0, 1);
	VictorSP climber = new VictorSP(2);
	VictorSP elevator = new VictorSP(3);
	VictorSP shooter = new VictorSP(4);
//	Shooter shooter = new Shooter(4, true, new Pats_PID_Controller(), shooterEncoder);
	
	GyroSensorActuator pidGyro = new GyroSensorActuator(drive, gyro);
	EncoderSensor encoderSense = new EncoderSensor(lEncoder, rEncoder);
	EncoderActuator encoderActuator = new EncoderActuator(drive);
	SendableChooser<String> chooser = new SendableChooser<>();
	
	Pats_PID_Controller pixyPid = new Pats_PID_Controller(0.5, .006 , .0006 , 10, pidPixy, pidGyro);
	Pats_PID_Controller encoderPid = new Pats_PID_Controller(0.5, 0.006 , 0.0006, 10, encoderSense, encoderActuator); 

	Timer time = new Timer();
	boolean base = false, timer = false;
	
	
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Gear", gear);
		chooser.addObject("Base Line", baseline);		
		chooser.addObject("Gear Time", gearTime);

		
		SmartDashboard.putData("Auto Choices", chooser);
		rEncoder.setDistancePerPulse(0.0071585709635417);
		lEncoder.setDistancePerPulse(0.0071585709635417);
//		rEncoder.setReverseDirection(true);
		lEncoder.reset();
		rEncoder.reset();
		gyro.reset();
//		shooterEncoder.setDistancePerPulse(1/128);
		
	}
	@Override
	public void autonomousInit() {
		lEncoder.reset();
		rEncoder.reset();
		rEncoder.setDistancePerPulse(.00716);
		lEncoder.setDistancePerPulse(.00716);
		encoderPid.stop();
		base = false;
		timer = false;
		auto = chooser.getSelected();
	}
	
	
	
	
	@Override
	public void autonomousPeriodic() {
		auto = chooser.getSelected();
		switch (auto){
		
		case gearTime:
				while (!timer && driverStation.isAutonomous()){
					drive.setLDrive(0.29);
					drive.setRDrive(0.3);
					
					time.schedule(new TimerTask(){
					@Override	
					public void run() {
						timer = true;
					}
					}, 2850);
				
				}
				drive.stop();
			
				while (!timer && driverStation.isAutonomous()){
					drive.setLDrive(0.2);
					drive.setRDrive(0.2);
					
					time.schedule(new TimerTask(){
					@Override	
					public void run() {
						timer = true;
					}
					}, 2850);
				
				}
				drive.stop();
				
			break;
		
		case baseline:

			while (!timer){
				drive.setLDrive(0.496);
				drive.setRDrive(0.5);
				
				time.schedule(new TimerTask(){
				@Override	
				public void run() {
					timer = true;
				}
				}, 2250);
			
			}
			drive.stop();
			
			
			break;
			
		case gear:
			resetEncoders();
//			if(timer)
//				return;
			double encoderDistance = rEncoder.getDistance();
			encoderPid.set(0);
			encoderPid.start();
			drive.setLDrive(.3);
			drive.setRDrive(.3);			
			int counter = 0;
			while(encoderDistance < 7.7 && driverStation.isAutonomous()){
				encoderDistance = rEncoder.getDistance();
				counter++;
				if(counter >= 500){
					counter = 0;
					System.out.println("R Encoder: " + encoderDistance + " Error: " + encoderPid.getError());
					
				}
			}
			encoderPid.stop();
			drive.stop();
			timer = false;
//			while (!timer){
//				drive.setLDrive(0.296);
//				drive.setRDrive(0.3);
//				System.out.println(rEncoder.getDistance());
//				time.schedule(new TimerTask(){
//				@Override	
//				public void run() {
//					timer = true;
//					
//				}
//				}, 2300);
//			
//			}
			drive.stop();
			
			
			break;
			
		default:
			drive.setLDrive(0);
			drive.setRDrive(0);
		}
	}
		

	
	@Override
	public void teleopPeriodic() {
		
		encoderPid.stop();
		drive.stop();
		
		

		double throttle = -Control.drive.get(CTRL_AXIS.LY);
		double turn = Control.drive.get(CTRL_AXIS.RX);
		double shifter = Control.drive.isButtonHeld(CTRL_BTN.R) ? 0.5 : 1.0;
		
		if (Control.drive.isButtonHeld(CTRL_BTN.L)){
			drive.setRDrive(-Control.drive.get(CTRL_AXIS.RY)*shifter);
			drive.setLDrive(-Control.drive.get(CTRL_AXIS.LY)*shifter);
		}else{
//			drive.setRDrive(throttle * shifter * (1 + Math.min(0, turn)));
//			drive.setLDrive(throttle * shifter * (1 - Math.max(0, turn)));
			drive.setRDrive(throttle * shifter * (1 - Math.max(0, turn)));
			drive.setLDrive(throttle * shifter * (1 + Math.min(0, turn)));
//			
		}
		
		if (Control.operator.isButtonPushed(CTRL_BTN.B)){
			resetEncoders();
			
		}
		
		counter++;
		if(counter >= 200){
			System.out.println("Left distance: " + lEncoder.getDistance() + " Right distance: " + rEncoder.getDistance());
			counter = 0;
			
		}
		
		if (Control.operator.get(CTRL_AXIS.RY) <= 0){
			
			climber.set(-Control.operator.get(CTRL_AXIS.RY));
		}
		
		
		if (Control.operator.get(CTRL_AXIS.RTrigger) > .5){
			elevator.set(0.5);
		}else if (Control.operator.get(CTRL_AXIS.LTrigger) > .5){
			elevator.set(-0.5);
		}else{
			elevator.set(0);
			
		}
        if(Control.operator.isButtonHeld(CTRL_BTN.A)){
        	shooter.set(1);
        }else{
        	shooter.set(0);
        	
        }
//        	shooter.start_pid();
//        	while(!Control.operator.isButtonPushed(CTRL_BTN.A)){
//        		counter ++;
//        		if((counter % 10000) == 0){
//        			System.out.println("IN PID LOOP: " + "Throttle: " + Control.operator.get(CTRL_AXIS.LY) + " Speed: " + shooter.read() + " Set Point: " + shooter.getSetPoint() + " Error: " + shooter.controller.getError());
//        		}
//        		shooter.setRPM(2675);
//        		shooter.controller.setPIDS(.00006, 0.000008, -0.000000001);
//        		}
//        	}
//        	shooter.stop();
//		System.out.println("R Encoder: " + rEncoder.getDistance() + " L Encoder: " + lEncoder.getDistance());
        }
	
	public void VisionTrack(double target){
		PixyPacket pkt = i2c.getPixy();
		System.out.println(pkt.y);
		if(pkt.y != -1){
			pixyPid.set(target);
			pixyPid.start();
			System.out.println("Test1");
			while(pkt.y < target - .04  || pkt.y > target + .04){
				System.out.println("Test2");
				if(pkt.area == -1)
					break;
				System.out.println("YPos: " + pkt.y + "PID Value: " + pixyPid.getMotorValue());
				pkt = i2c.getPixy();
			}
			pixyPid.stop();
			drive.stop();
		}
//			
//			if(pkt.y < target - .05 || pkt.y > target + .05){//Only start PID if off centered
//				//pixyPid.set(target);
//				//pixyPid.start();
//				while(pkt.y < target -.06 || pkt.y > target + .06){
//					if(pkt.y == -1 || !Control.drive.isButtonHeld(CTRL_BTN.Y)){//Restart loop if ball lost during turn
//						//pixyPid.stop();
//						drive.stop();
//						return;
//						//break;
//					}
//					if(pkt.y < target - .06){
//						drive.setLDrive(0.2);
//						drive.setRDrive(-0.2);
//					}else if(pkt.y > target + .06){
//						drive.setLDrive(-0.2);
//						drive.setRDrive(0.2);
//					}else{
//						break;
//					}
//					pkt = i2c.getPixy();
//					System.out.println("YPos: " + pkt.y);
//
//				}
//				drive.stop();
				//pixyPid.stop();			
//			if(pkt.area <= minArea && pkt.area > 0){
//				drive.setLDrive(0.3);
//				drive.setRDrive(0.3);
//				System.out.println("Area: " + pkt.area);
//			}else if(pkt.area >= maxArea && maxArea > 0){
//				drive.setLDrive(-0.3);
//				drive.setRDrive(-0.3);
//				System.out.println("Area: " + pkt.area);

//			}else{
//				drive.stop();
//				System.out.println("Area: " + pkt.area);
//			}
			
//		}else{//Don't move if see nothing
//			drive.stop();
//		}
		
		
	}
	
	public void resetEncoders(){
		lEncoder.reset();
		rEncoder.reset();
	}
	
	public void driveStraight(double speed, double distance){
		rEncoder.reset();
		lEncoder.reset();
		encoderPid.set(0);
		encoderPid.start();
		drive.setRDrive(speed);
		while (rEncoder.getDistance() <= distance){
			encoderPid.set(rEncoder.getDistance());
		}
		encoderPid.stop();
		drive.stop();
	}

		
		
	
	@Override
	public void testPeriodic() {

	}
}
