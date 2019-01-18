package frc.robot.subsystems;

import frc.robot.subsystems.TalonPIDSource;
import edu.wpi.first.wpilibj.PIDController;

public class PIDImpl implements PID {

    double rotate;

    PIDController pidcontrol;
    
    double Kp;
    double Ki;
    double Kd; 

     @Override
    public void gyroPidWrite(double output) {
        rotate = output;
    }
}