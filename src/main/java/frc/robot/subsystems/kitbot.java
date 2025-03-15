package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

public class kitbot extends SubsystemBase  { 

 // private static final int kCanID_neo1 = 14; //este tiene el encoder/
 // private static final MotorType kMotorType = MotorType.kBrushed;
 // private SparkMax  m_motor_neo;

public kitbot() {  
//m_motor_neo =  new SparkMax(kCanID_neo1, kMotorType); 
//      SparkMaxConfig config = new SparkMaxConfig();
//      config
//    .inverted(false)
//    .idleMode(IdleMode.kBrake);
// m_motor_neo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


public Command Deja_Coral() {
    return runOnce(
        () -> {

         // m_motor_neo.set(.5);
          Timer.delay(.1);
  //       m_motor_neo.set(0);
        });

  } 
}