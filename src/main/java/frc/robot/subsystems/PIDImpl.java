package frc.robot.subsystems;

import frc.robot.subsystems.TalonPIDSource;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;

public class PIDImpl implements PID {

    PIDController pidcontrol;
    TalonPIDSource talonPID;
    PIDSource pidsource;
    PIDOutput pidoutput;
    
    double Kp;
    double Ki;
    double Kd; 

    @Override
    public void PIDInit() {
        this.Kp = 0.001;
        this.Ki = 0.001;
        this.Kd = 0.001;

        this.talonPID = new TalonPIDSource()
        
        this.pidcontrol = new PIDController(Kp, Ki, Kd, pidsource, pidoutput);
    }


}
