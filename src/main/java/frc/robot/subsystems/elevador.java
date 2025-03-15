//elevador
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Valores_Elevador;
import frc.robot.Constants.Elevador_Motores;
import frc.robot.Constants.Elevador_PID_Values;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkClosedLoopController;

public class elevador extends SubsystemBase  { 

 
 //private AnalogPotentiometer sensor_ultra = new AnalogPotentiometer(3,765);
 // private AnalogPotentiometer sensor_ultra2 = new AnalogPotentiometer(2,765);
  
  private double Brazo_Objetivo;
  private double Elevador_Objetivo;
  private double Giro_Objetivo;

  // INPUTS sensores
  private double posicion_elevador;
  private double posicion_brazo;
  private double posicion_giro;
  
  private SparkMax   m_motor_elevador =  new SparkMax(Elevador_Motores.kCanID_elevador, MotorType.kBrushless); 
  private SparkMax   m_motor_elevador2 =  new SparkMax(Elevador_Motores.kCanID_elevador2, MotorType.kBrushless); 
  private SparkMax m_motor_brazo =  new SparkMax(Elevador_Motores.kCanID_brazo, MotorType.kBrushless); 
  private SparkMax  m_motor_giro =  new SparkMax(Elevador_Motores.kCanID_giro, MotorType.kBrushless); 
  private SparkMax m_motor_taker =  new SparkMax(Elevador_Motores.kCanID_taker, MotorType.kBrushless); 

  private RelativeEncoder Elevador_Encoder = m_motor_elevador.getEncoder();
  private RelativeEncoder BrazoEncoder = m_motor_brazo.getEncoder();
  private RelativeEncoder GiroEncoder = m_motor_giro.getEncoder();
  private DigitalInput tomado = new DigitalInput(0);
  //private DigitalInput tomado2 = new DigitalInput(1);

  //private RelativeEncoder TakerEncoder = m_motor_taker.getEncoder();

      SparkClosedLoopController ElevadorController = m_motor_elevador.getClosedLoopController();
      SparkClosedLoopController BrazoController = m_motor_brazo.getClosedLoopController();
      SparkClosedLoopController GiroController = m_motor_giro.getClosedLoopController();
      //SparkClosedLoopController TakerController = m_motor_taker.getClosedLoopController();
  
  
public elevador() {  

      SparkMaxConfig config = new SparkMaxConfig();
      SparkMaxConfig config2 = new SparkMaxConfig();
      SparkMaxConfig config_brazo = new SparkMaxConfig();
      SparkMaxConfig config_giro = new SparkMaxConfig();
      SparkMaxConfig config_taker = new SparkMaxConfig();
      

      config
       .inverted(false)
       .idleMode(IdleMode.kBrake)
       .closedLoop
       .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
       .pidf(Elevador_PID_Values.elevador_p,Elevador_PID_Values.elevador_i,Elevador_PID_Values.elevador_d,Elevador_PID_Values.elevador_ff)
       .outputRange(-1, 1)
       .maxMotion
        .maxVelocity(Elevador_PID_Values.elevador_maxVelocity)
        .maxAcceleration(Elevador_PID_Values.elevador_maxAcceleration)
       .allowedClosedLoopError(Elevador_PID_Values.elevador_LoopError);

      config_brazo
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .pidf(Elevador_PID_Values.brazo_p, Elevador_PID_Values.brazo_i,Elevador_PID_Values.brazo_d, Elevador_PID_Values.brazo_ff)  //probar...
        .outputRange(-1, 1)
        .maxMotion
         // Set MAXMotion parameters for position control
        .maxVelocity( Elevador_PID_Values.brazo_maxVelocity)
        .maxAcceleration(Elevador_PID_Values.brazo_maxAcceleration)
        .allowedClosedLoopError(Elevador_PID_Values.brazo_LoopError);
   
      config_giro
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
        //.p(0.1)  //probar...
        .pidf(Elevador_PID_Values.giro_p,Elevador_PID_Values.giro_i,Elevador_PID_Values.giro_d,Elevador_PID_Values.giro_ff)
        .outputRange(-1, 1)
        .maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(Elevador_PID_Values.giro_maxVelocity)
        .maxAcceleration(Elevador_PID_Values.giro_maxAcceleration)
        .allowedClosedLoopError(Elevador_PID_Values.giro_LoopError);

      config_taker
        .inverted(false)
        .idleMode(IdleMode.kBrake);
      
      
     config2 
     .idleMode(IdleMode.kBrake);
     config2.follow(Elevador_Motores.kCanID_elevador,true);

     m_motor_elevador2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_motor_elevador.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_motor_brazo.configure(config_brazo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_motor_giro.configure(config_giro, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_motor_taker.configure(config_taker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     Elevador_Encoder.setPosition(0);
     BrazoEncoder.setPosition(0);
     GiroEncoder.setPosition(0);
    // TakerEncoder.setPosition(0);


      switch (Robot.nivel_dejar) {
        case 0:   SmartDashboard.putString("Nivel","L4 deja");
                  break;
        case 1:   SmartDashboard.putString("Nivel","L3 deja");
                  break;
        case 2:   SmartDashboard.putString("Nivel","L2 deja");
                  break;
        case 3:   SmartDashboard.putString("Nivel","L1 deja");
                  break;
        case 4:   SmartDashboard.putString("Nivel","deja Alga");
                  break;                }

       switch (Robot.nivel_tomar) {
                    case 0:   SmartDashboard.putString("Tomar_nivel","Coral Human");
                              break;
                    case 1:   SmartDashboard.putString("Tomar_nivel","Alga L2-L3");
                              break;
                    case 2:   SmartDashboard.putString("Tomar_nivel","Alga L3-L4");
                              break;
                    case 3:   SmartDashboard.putString("Tomar_nivel","Coral Inicial");
                              break;
                    case 4:   SmartDashboard.putString("Tomar_nivel","Coral Piso");
                              break;   
                    case 5:   SmartDashboard.putString("Tomar_nivel","Alga Piso");
                              break;                   }
  }

  @Override
  public void periodic(){
    posicion_brazo = BrazoEncoder.getPosition();
    posicion_elevador = Elevador_Encoder.getPosition();
    posicion_giro = GiroEncoder.getPosition();
    SmartDashboard.putNumber("Posicion Brazo", posicion_brazo);
    SmartDashboard.putNumber("Posicion Elevador", posicion_elevador);
    SmartDashboard.putNumber("Posicion Giro", posicion_giro);
  }

  private void Posicionar() {
    BrazoController.setReference(Brazo_Objetivo, ControlType.kMAXMotionPositionControl);
    ElevadorController.setReference(Elevador_Objetivo, ControlType.kMAXMotionPositionControl);
    GiroController.setReference(Giro_Objetivo, ControlType.kMAXMotionPositionControl);
  }

  private void Posicionar_joy(double dato) {
    if(Math.abs(dato) < 0.01){
      return;
    }
    double setpoint = Math.max(Elevador_PID_Values.elevador_minSetPoint, posicion_elevador + (dato*3));
    setpoint = Math.min(Elevador_PID_Values.elevador_maxSetPoint, setpoint);
    ElevadorController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
  }

  private void Baja_L4() {
  //  ElevadorController.setReference(Elevador_Objetivo-10, ControlType.kMAXMotionPositionControl);
   // ahora debe bajar el Brazo al Dejar
}

public Command Intento() {

    return runOnce(
      () -> { 
        ElevadorController.setReference(10, ControlType.kMAXMotionPositionControl);
      });
    
  }
  public Command Pocisiona_Dejar_L4() {
    return runOnce(
        () -> { 
          Baja_L4();
        });

  }

  public void setSetpointCommand_dejar(int setpoint) {
    Elevador_Objetivo = Valores_Elevador.Dejar[setpoint][0];
    Brazo_Objetivo = Valores_Elevador.Dejar[setpoint][1];
    Giro_Objetivo =  Valores_Elevador.Dejar[setpoint][2];
  }

  public void setSetpointCommand_tomar(int setpoint) {
    Elevador_Objetivo = Valores_Elevador.Tomar[setpoint][0];
    Brazo_Objetivo = Valores_Elevador.Tomar[setpoint][1];
    Giro_Objetivo =  Valores_Elevador.Tomar[setpoint][2];
  }

public Command Espera_Actualiza() {
  return runOnce(
        () -> { 

          //Timer.delay(1);
        });
}


public Command PruebaGiro() {
  return runOnce(
        () -> { 

          GiroController.setReference(.75, ControlType.kMAXMotionPositionControl);
          SmartDashboard.putNumber("Encoder Giro", GiroEncoder.getPosition());

        });
}

public Command PruebaGiro_regresa() {
  return runOnce(
        () -> { 

          GiroController.setReference(0, ControlType.kMAXMotionPositionControl);
          SmartDashboard.putNumber("Encoder Giro", GiroEncoder.getPosition());

        });
}

  public Command taker_toma() {
    return new FunctionalCommand(
        () -> {m_motor_taker.set(-0.8);},
        () -> {},
        interrupted -> { Timer.delay(0.2); m_motor_taker.set(0);},
        () -> tomado.get() //|| tomado2.get()
        );
      }
  
  public Command taker_stop() {

    return runOnce(
        () -> {
          //kerController.setReference(20, ControlType.kMAXMotionPositionControl);
            m_motor_taker.set(0); 
          }
        );
  }
          
  public Command taker_inicial() {
    return runOnce(
      () -> { 
          m_motor_taker.set(-0.5); 
          Timer.delay(.3);
          m_motor_taker.set(0);
        }
      );
  }

  public Command taker_deja() {
    switch (Robot.nivel_dejar) {
      case 0:   return new SequentialCommandGroup(Pocisiona_Dejar_L4(),
        runOnce(() -> {
           m_motor_taker.set(.5);
           Timer.delay(.2);
           m_motor_taker.set(0);
          }));
      case 1: return runOnce(() -> {
        m_motor_taker.set(.5);
        Timer.delay(.3);
        m_motor_taker.set(0);
       });
       case 2: return runOnce(() -> {
        m_motor_taker.set(.5);
        Timer.delay(.3);
        m_motor_taker.set(0);
       });
       case 3: return runOnce(() -> {
        m_motor_taker.set(.5);
        Timer.delay(.3);
        m_motor_taker.set(0);
       });
       case 4: return new ParallelCommandGroup(
        runOnce(() -> {
          GiroController.setReference(Giro_Objetivo-3/4, ControlType.kMAXMotionPositionControl);
         }),
       runOnce(() -> {
          m_motor_taker.set(.5);
          Timer.delay(.3);
          m_motor_taker.set(0);
         }));
    
  //    case 1:   SmartDashboard.putString("Nivel","L3 deja");
  //              break;
  //    case 2:   SmartDashboard.putString("Nivel","L2 deja");
  //              break;
  //    case 3:   SmartDashboard.putString("Nivel","L1 deka");
  //              break;
  //    case 4:   SmartDashboard.putString("Nivel","deja Alga");
  //              break;        
              
                default: 
                return runOnce(
                  () -> {
          
                //    TakerController.setReference(0, ControlType.kMAXMotionPositionControl);
                     m_motor_taker.set(.5);
                     Timer.delay(.3);
                     m_motor_taker.set(0);
          
                  });   } 
             
   

  }
  
  public Command Pocisiona_Dejar() {
    return run(
        () -> { 
          setSetpointCommand_dejar(Robot.nivel_dejar); 
          Posicionar();
          SmartDashboard.putNumber("Encoder Elevador", Elevador_Encoder.getPosition());
          SmartDashboard.putNumber("Encoder Giro", GiroEncoder.getPosition());

        });

  }
  
  public Command Pocisiona_Tomar() {
    return run(
        () -> { 
          setSetpointCommand_tomar(Robot.nivel_tomar); 
          Posicionar();
          SmartDashboard.putNumber("Encoder Elevador", Elevador_Encoder.getPosition());
          SmartDashboard.putNumber("Encoder Giro", GiroEncoder.getPosition());
          
        });

  }

  public Command CambiaNivelTomar() {

    return runOnce(
      () -> {
         
          if (Robot.nivel_tomar <6) {
            Robot.nivel_tomar= Robot.nivel_tomar + 1;
          } else { Robot.nivel_tomar = 0 ; }

          //SmartDashboard.putNumber("ciclos",Robot.ciclos);

          switch (Robot.nivel_tomar) {
            case 0:   SmartDashboard.putString("Tomar_nivel","Coral Human");
                      Robot.nivel_dejar = 0;
                      break;
            case 1:   SmartDashboard.putString("Tomar_nivel","Alga L2-L3");
                      Robot.nivel_dejar = 4;
                      break;
            case 2:   SmartDashboard.putString("Tomar_nivel","Alga L3-L4");
                      Robot.nivel_dejar = 4;
                      break;
            case 3:   SmartDashboard.putString("Tomar_nivel","Coral Inicial");
                      Robot.nivel_dejar = 0;
                      break;
            case 4:   SmartDashboard.putString("Tomar_nivel","Coral Piso");
                      Robot.nivel_dejar = 1;
                      break;   
            case 5:   SmartDashboard.putString("Tomar_nivel","Alga Piso");
                      Robot.nivel_dejar = 4;
                      break;                   }

          switch (Robot.nivel_dejar) {
            case 0:   SmartDashboard.putString("Nivel","L4 deja");
                      break;
            case 1:   SmartDashboard.putString("Nivel","L3 deja");
                      break;
            case 2:   SmartDashboard.putString("Nivel","L2 deja");
                      break;
            case 3:   SmartDashboard.putString("Nivel","L1 deja");
                      break;
            case 4:   SmartDashboard.putString("Nivel","deja Alga");
                      break;                } 

                      //SmartDashboard.putBoolean("Sensor Garra",tomado.get());
         });
 }

 public Command CambiaNivel() {
    return runOnce(
      () -> {
        
          if (Robot. nivel_dejar <5) {
            Robot. nivel_dejar= Robot. nivel_dejar+ 1;
          } else { Robot. nivel_dejar = 0 ; }

          //SmartDashboard.putNumber("ciclos",Robot.ciclos);
          switch (Robot. nivel_dejar) {
            case 0:   SmartDashboard.putString("Nivel","L4 deja");
                      break;
            case 1:   SmartDashboard.putString("Nivel","L3 deja");
                      break;
            case 2:   SmartDashboard.putString("Nivel","L2 deja");
                      break;
            case 3:   SmartDashboard.putString("Nivel","L1 daja");
                      break;
            case 4:   SmartDashboard.putString("Nivel","deja Alga");
                      break;                }
        });
  }
  public Command elevadorManual(DoubleSupplier raw_double){
    return run(() -> {Posicionar_joy(raw_double.getAsDouble());});
  }
}
