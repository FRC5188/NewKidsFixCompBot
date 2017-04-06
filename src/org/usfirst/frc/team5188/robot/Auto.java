package org.usfirst.frc.team5188.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import modules.Pats_PID_Controller;

public class Auto  {
	Drive drive;
	AHRS gyro;
	Encoder encoder;

	public Auto(Drive drive, AHRS gyro, Encoder encoder){
		this.drive = drive;
		this.gyro = gyro; 
		this.encoder = encoder;
	}
	
	GyroSensorActuator pidGyro = new GyroSensorActuator(drive, gyro);
//	M_I2C i2c = new M_I2C();
	
//	PixySensor pidPixy = new PixySensor(i2c);
//	Pats_PID_Controller pid = new Pats_PID_Controller(0.006, 0.006 , .053, 10, pidPixy, pidGyro);
	Pats_PID_Controller gyroPid = new Pats_PID_Controller(0.5, 0.9 , .85, 10, pidGyro, pidGyro);

	boolean base = false, timer = false;



	/// < summary >
	/// This method will drive straight using the gyro for x seconds.
	/// This requires the GetMotorOutput method.
	/// < /summary >@Override

	
		// Reset the NavX
		void DriveStraightFor ( double distance )
		{
			// Reset the NavX
			gyro.reset();
			encoder.reset();
			while ( encoder.getDistance() <= distance)
			{
				if (encoder.getDistance() >= distance - 4){
					break;
				}
				drive.setLDrive(GetMotorOutput(gyro.getAngle(), 0) *.3);
				drive.setRDrive(GetMotorOutput(gyro.getAngle(), 1) *.285);
				System.out.println("Right Power: " + GetMotorOutput(gyro.getAngle(), 1) *.3 + " Left Power " + GetMotorOutput(gyro.getAngle(), 0) *.3 + " Encoder Distance " + encoder.getDistance());

				Timer.delay(.05);
				
			}
			System.out.println("we Stoped");
			drive.stop();
		}
	

	/// < summary >
	/// This method will return the required motor ouput.
	/// < /summary >
	/// < param name = "degrees" > This is the degrees returned by the NavX < /param >
	/// < param name = "sel" > This is the side selection. < /param >
	double GetMotorOutput(double degrees, int sel)
	{
		// Max ouputs
		// Right side max power
		int rX = 1;
		
		// Left side max power
		int lX = 1;

		// Internal Data
		// Left power
		double lP = 0;

		// Right power
		double rP = 0;

		// The degrees where f(x) = 0; stoping the motor completely. Anything greater
		// will go in reverse. ( This is basically the sensitvity. The closer to 0, the 
		// more it will aggressivly turn )
		double mid = 15.0;

		// Plug in the function for the new right output.
		if (degrees > 360)
		{
			degrees -= 360;
		}
		if (degrees < -360)
		{
			degrees += 360;
		}

		// Set the motor power
	        rP = rX - ( ( 1.0 / mid ) * -degrees * rX );
	        lP = lX - ( ( 1.0 / mid ) * degrees * lX );

	        if (rP > 1)
	        {
	            rP = 1;
	        }
	        else if (rP < -1)
	        {
	            rP = -1;
	        }

	        if ( lP > 1 )
	        {
	            lP = 1;
	        }
	        else if ( lP < -1 )
	        {
	            lP = -1;
	        }

		// Set the returned value.
	        double ret = (sel == 1) ? rP : lP;
//	        System.out.println("New motor power:  " + ret);
//	        System.out.println("Gyro pos = " + gyro.getAngle());
	        return ret;
	}

	/// < summary >
	/// This method will rotate the robot to a certain degree.
	/// < / summary >
	/// < param name = "degrees" > The target degrees < /param >
	/// < param name = "tolerance" > The dergee of tolerance on each side ( high and low ) < /param >
	/// < param name = "power" > The speed of the motor ( remember, +-1 is max ) < /param > 
	void RotateToDegreesIteration ( double degrees, double tolerance, double power)
	{
		
		// Set the tolerance
		double high = degrees + tolerance;
		double low = degrees - tolerance;

		// If the robot is too far to the right, turn left. 
		// Else, if the robot is too far to the left, turn right.
		// If not, give the last motor power it received.
		while (gyro.getAngle() < low || gyro.getAngle() > high){
			if (gyro.getAngle() < low)
			{
				drive.setLDrive(power);
				drive.setRDrive(-power);
			}
			if (gyro.getAngle() > high)
			{
				drive.setLDrive(-power);
				drive.setRDrive(power);
			}
			System.out.println("Gyro Angle: " + gyro.getAngle());
		}
		
		drive.stop();
	}

	public void Rotate(double degrees){
		//	gyro.reset();
		double finalAngle;
		finalAngle = gyro.getAngle() + degrees;
		double initAngle = gyro.getAngle();
		gyroPid.resetAccumulator();
		gyroPid.set(finalAngle);
		gyroPid.start();
		while (Math.abs(initAngle - gyro.getAngle()) < 88){
			while (gyro.getAngle() < finalAngle){
			
//			System.out.println(" Set Point: " + dangle + " Error: " + (dangle - gyro.getAngle()) + " Current Angle: " + gyro.getAngle());

			}
			
		gyroPid.stop();
		}
	}	
}


