package modules;

public enum CTRL_AXIS {
	LX (0),
	LY(1),
	LTrigger(2),
	RTrigger(3),
	RX(4),
	RY(5),
	AXIS_TOTAL(6);
	
	public int value;
	
	CTRL_AXIS(int value){
		this.value = value;
	}
}
