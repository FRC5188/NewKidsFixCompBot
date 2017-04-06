
package org.usfirst.frc.team5188.robot;

import edu.wpi.first.wpilibj.Timer;

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
	final String gearCenter = "gearCenter";
	final String baseline = "baseline";
	final String gearTime = "gearTime";
	int counter = 0;
	String auto;
	double error = 0;
	static boolean ifCenter = false; 
	
	Encoder rEncoder = new Encoder(0,1);
	Encoder lEncoder = new Encoder(2,3);
	
	M_I2C i2c = new M_I2C();
	AHRS gyro = new AHRS(SerialPort.Port.kMXP);
	PixySensor pidPixy = new PixySensor(i2c);
	
	DriverStation driverStation = DriverStation.getInstance();
	
	Drive drive = new Drive(0, 1);
	VictorSP climber = new VictorSP(2);
	VictorSP elevator = new VictorSP(3);
	
	Auto autoFunc = new Auto(drive, gyro, rEncoder);
	
	
	GyroSensorActuator pidGyro = new GyroSensorActuator(drive, gyro);
	SendableChooser<String> chooser = new SendableChooser<>();
	
	Pats_PID_Controller pixyPid = new Pats_PID_Controller(0.5, .006 , .0006 , 10, pidPixy, pidGyro);
	
	Timer time = new Timer();
	boolean base = false, timer = false;
	
	
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("Gear Center", gearCenter);
		chooser.addObject("Base Line", baseline);		
		chooser.addObject("Gear Time", gearTime);

		
		SmartDashboard.putData("Auto Choices", chooser);
		rEncoder.setDistancePerPulse(0.08590292414);
		lEncoder.setDistancePerPulse(0.08590292414);
//		rEncoder.setReverseDirection(true); THIS MAY NEED TO BE CHANGED
		lEncoder.reset();
		rEncoder.reset();
		gyro.reset();

		
	}
	@Override
	public void autonomousInit() {
//		lEncoder.reset(); BOTH OF THESE MAY NEED TO HAPPEN
//		rEncoder.reset();


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
					
					Timer.delay(2.850);
					
					timer = true;
				
				}
				drive.stop();
			
//				NOT SURE WHY THERE IS TWO
				
//				while (!timer && driverStation.isAutonomous()){
//					drive.setLDrive(0.2);
//					drive.setRDrive(0.2);
//					
//					time.schedule(new TimerTask(){
//					@Override	
//					public void run() {
//						timer = true;
//					}
//					}, 2850);
//				
//				}
//				drive.stop();
				
			break;
		
		case baseline:

			drive.setLDrive(0.496);
			drive.setRDrive(0.5);
			
			Timer.delay(2.250);
			
			drive.stop();
			
			
			break;
			
		case gearCenter:
			resetEncoders();
			
			Timer.delay(.5);
			
			autoFunc.DriveStraightFor(36); // THIS IS A RANDOM DISTANCE

			drive.stop();
			
			
			break;
			
		default:
			drive.setLDrive(0);
			drive.setRDrive(0);
		}
	}
		

	
	@Override
	public void teleopPeriodic() {

		
		

		double throttle = -Control.drive.get(CTRL_AXIS.LY);
		double turn = Control.drive.get(CTRL_AXIS.RX);
		double shifter = Control.drive.isButtonHeld(CTRL_BTN.R) ? 0.5 : 1.0;
		
		if (Control.drive.isButtonHeld(CTRL_BTN.L)){
			drive.setRDrive(-Control.drive.get(CTRL_AXIS.RY)*shifter);
			drive.setLDrive(-Control.drive.get(CTRL_AXIS.LY)*shifter);
		}else{
			
//			I DON'T KNOW WHICH ONE IS RIGHT
			
			drive.setRDrive(throttle * shifter * (1 + Math.min(0, turn)));
			drive.setLDrive(throttle * shifter * (1 - Math.max(0, turn)));
//			drive.setRDrive(throttle * shifter * (1 - Math.max(0, turn)));
//			drive.setLDrive(throttle * shifter * (1 + Math.min(0, turn)));
//			
		}
		
//		if (Control.operator.isButtonPushed(CTRL_BTN.B)){
//			resetEncoders();
//			
//		}
		
//		counter++;
//		if(counter >= 200){
//			System.out.println("Left distance: " + lEncoder.getDistance() + " Right distance: " + rEncoder.getDistance());
//			counter = 0;
//			
//		}
		
		if (Control.operator.get(CTRL_AXIS.RY) <= 0){
			
			climber.set(-Control.operator.get(CTRL_AXIS.RY));
		}
		
	}
		
		
		
	
	private boolean VisionTrack(double target){
		PixyPacket pkt = i2c.getPixy();
		System.out.println(pkt.y);
		if(pkt.x != -1){
			if(pkt.x < .39 || pkt.x > .44){//Only start PID if off centered
				while(pkt.x < .39 || pkt.x > .44){
					
					if(pkt.x < .39){
						drive.setLDrive(-0.2);
						drive.setRDrive(0.2);
						
					}
					if(pkt.x > .44){
						drive.setLDrive(0.2);
						drive.setRDrive(-0.2);
					}
					if (Control.drive.isButtonHeld(CTRL_BTN.B))
						break;
					if(pkt.y == -1)//Restart teleop if ball lost during turn
						break;
					pkt = i2c.getPixy();
					System.out.println("XPos: " + pkt.x);
				}
				pixyPid.stop();
				pkt = i2c.getPixy();

			}
			pixyPid.stop();
			if(pkt.area >= 0.1 && pkt.area <= 0.16){
				drive.stop();
				ifCenter = true;
			}
			if(pkt.area <= 0.12 && pkt.area > 0){
				pixyPid.stop();
				drive.setLDrive(0.3);
				drive.setRDrive(0.3);
				System.out.println("Area: " + pkt.area);
				pkt = i2c.getPixy();

			}else if(pkt.area >= 0.14){
				pixyPid.stop();
				drive.setLDrive(-0.3);
				drive.setRDrive(-0.3);
				System.out.println("Area: " + pkt.area);


			}else{
				drive.stop();
				System.out.println("Area: " + pkt.area);

			}
			
		}else{//Don't move if see nothing
			drive.stop();
		}
		return ifCenter;
	}
	
	public void resetEncoders(){
		lEncoder.reset();
		rEncoder.reset();
	}		
	
	@Override
	public void testPeriodic() {

	}
}
