package modules;

public interface PID_Controller {
	public void setPIDS(double p, double i, double d);
	public double[] getPIDS();
	public void set(double setPoint);
	public void resetAccumulator();
	public void start();
	public void stop();
	public void invert(boolean inverstionState);
	PID_Sensor getSensor();
	PID_Actuator getActuator();
	public void setSensor(PID_Sensor sensor);
	public void setActuator(PID_Actuator actuator);
	public boolean isRunning();
	public double getSet();	//returns setpoint
	public double getError();
	public void setLoopTime(double loopTime);
}