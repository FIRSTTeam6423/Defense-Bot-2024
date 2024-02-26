package frc.robot.commons;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public final class IronUtil {
    public static double deadzone(double input, double zone) {
        if(Math.abs(input) >= zone){
			return input;
		} else {
			return 0;
		}
    }
	public static boolean inRange(double input, double goal, double range) {
		return input > goal - range && input < goal + range;
	} 
	public static double powKeepSign(double input, double pow) {
		int sign = (int) Math.signum(input);
		return sign * Math.pow(input, pow);
	}
}
