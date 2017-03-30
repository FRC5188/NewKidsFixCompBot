package modules;

public enum CTRL_BTN {
	A(1),
	B(2),
	X(3),
	Y(4),
	L(5),
	R(6),
	BACK(7),
	START(8),
	L_STICK(9),
	R_STICK(10),
	TOTAL_BUTTONS(10);
	
	public int value;
	
	CTRL_BTN(int value){
		this.value = value;
	}
}
