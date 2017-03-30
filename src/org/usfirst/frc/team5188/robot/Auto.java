package org.usfirst.frc.team5188.robot;
import java.util.Timer;
import java.util.TimerTask;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import modules.Pats_PID_Controller;

public class Auto {
	Timer time = new Timer();
	Drive drive = new Drive(0, 1);
	AHRS gyro = new AHRS(SerialPort.Port.kMXP);
	GyroSensorActuator pidGyro = new GyroSensorActuator(drive, gyro);
	M_I2C i2c = new M_I2C();

	PixySensor pidPixy = new PixySensor(i2c);

	Pats_PID_Controller pid = new Pats_PID_Controller(0.006, 0.006 , .053, 10, pidPixy, pidGyro);

	boolean base = false, timer = false;
	
	public Sendable VisionTrack(){
		PixyPacket pkt = i2c.getPixy();
		if(pkt.x != -1){
			if(pkt.x < .48 || pkt.x > .52){//Only start PID if off centered
				pid.set(0.5);
				pid.start();
				while(pkt.x < .48 || pkt.x > .52){
					if (!Control.drive.isButtonHeld(CTRL_BTN.Y))
						break;
					if(pkt.x == -1)//Restart teleop if ball lost during turn
						break;
					
					pkt = i2c.getPixy();
					System.out.println("XPos: " + pkt.x);

				}
				pid.stop();
			}
			if(pkt.area <= 0.05 && pkt.area > 0){
				drive.setLDrive(0.3);
				drive.setRDrive(0.3);
				System.out.println("Area: " + pkt.area);
			}else if(pkt.area >= 0.9){
				drive.setLDrive(-0.3);
				drive.setRDrive(-0.3);
				System.out.println("Area: " + pkt.area);

			}else{
				drive.stop();
				System.out.println("Area: " + pkt.area);

			}
			
		}else{//Dont move if see nothing
			drive.stop();
		}
		return null;
	}
	
	
	public Sendable baseline(){	
		if(!base){
			drive.setLDrive(0.8);
			drive.setRDrive(0.8);
			time.schedule(new TimerTask(){
				@Override
				public void run() {
					drive.setLDrive(0);
					drive.setRDrive(0);
					timer = true;
				}  }, 1000);
			base = true;
		}
		if(timer){
			VisionTrack();
		}
		return null;
	}

}
