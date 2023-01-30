package frc.robot.util;

public class ExponentialEase {
    public static double ease(double input) {
        if(input > 0) {
            return Math.pow(2, input)/2;
        } else if (input < 0) {
            return Math.pow(2, input)/-2;
        } else if (input == 0) {
            return 0;
        } else {
            throw new NumberFormatException("Input is not a number");
        }
    }
    
}
