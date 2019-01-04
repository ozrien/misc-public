package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {

    Joystick _joy = new Joystick(0);
    WPI_TalonSRX _left = new WPI_TalonSRX(1);
    WPI_TalonSRX _rght = new WPI_TalonSRX(0);


    public void teleopInit()
    {
        //default settings
        TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
        // use PID[0] for position, use a weak gain
        allConfigs.slot0.kP = 0.1;
        // use PID[1] to produce a term kF * Target,  
        //Now Target is a feed forward [-100,+100]
        allConfigs.slot1.kF = 1023.0/100.0;
        
        // use left as PID[0] + PID[1] master
        _left.configAllSettings(allConfigs); // NEW in 2019
        // phase it
        _left.setSensorPhase(true); //<<<<<<<<<< MODIFY THIS

        // right is an aux follower, default settings is fine 
        _rght.configFactoryDefault(); // NEW in 2019
    }
 
    public void teleopPeriodic() {
        // get left and right y stick
        double y1 = _joy.getRawAxis(1) * -1;
        double y2 = _joy.getRawAxis(3) * -1;

        //select slots
        _left.selectProfileSlot(0, 0); // primary PID[0] use slot 0
        _left.selectProfileSlot(1, 1); // aux PID[1] use slot 1

        // y1 will be our target plus/minus 4096 sensor units, or one rotatoin (CTRE MAG)
        // y2 will be our feedfowrad [-100,+100] => percentOutput thanks to our slot1.kF.
        _left.set(ControlMode.Position, 4096*y1, DemandType.AuxPID, y2*100);

        _rght.follow(_left, FollowerType.AuxOutput1);
    }
}
