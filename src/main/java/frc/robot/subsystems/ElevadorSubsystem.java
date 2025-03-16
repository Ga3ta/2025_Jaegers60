// elevador
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevador;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ElevadorSubsystem extends SubsystemBase {

  // private AnalogPotentiometer sensor_ultra = new AnalogPotentiometer(3,765);
  // private AnalogPotentiometer sensor_ultra2 = new AnalogPotentiometer(2,765);

  public enum PosicionElevador {
    Home,
    Manual,
    Source,
    L1,
    L2,
    L3,
    L4,
    Alga1,
    Alga2,
  }

  public enum PosicionBrazo {
    Home,
    Manual,
    Tomar,
    DejarL1,
    DejarL23,
    DejarL4,
  }

  public enum PosicionGiro {
    Home,
    Manual,
    Horizontal,
    Vertical,
  }

  private double[] elevadorMatrizPosicion = {
    0, 0, 50, 100, 120, 130, 140, 125, 135,
  };

  private String[] elevadorPosicionesNombres = {
    "Home", "Manual", "Source", "L1", "L2", "L3", "L4", "Alga1", "Alga2",
  };

  private double[] brazoMatrizPosicion = {0, 0, 0, 90, 50, 30};

  private String[] brazoPosicionesNombres = {
    "Home", "Manual", "Tomar", "DejarL1", "DejarL2-L3", "DejarL4"
  };

  private double[] giroMatrizPosicion = {
    0, 0, 0, 90,
  };

  private String[] giroPosicionesNombres = {
    "Home", "Manual", "Horizontal", "Vertical",
  };

  // Posiciones de los mecanismos
  private PosicionElevador m_elevador_current_position;
  private PosicionBrazo m_brazo_current_position;
  private PosicionGiro m_giro_current_position;

  // Setpoints de los mecanismos
  private double Brazo_Objetivo = 0;
  private double Elevador_Objetivo = 0;
  private double Giro_Objetivo = 0;

  // INPUTS sensores
  private double posicion_elevador = 0;
  private double posicion_brazo = 0;
  private double posicion_giro = 0;
  private boolean coral_tomado = false;

  // Motores
  private SparkMax m_motor_elevador = new SparkMax(Elevador.kCanID_elevador, MotorType.kBrushless);
  private SparkMax m_motor_elevador2 =
      new SparkMax(Elevador.kCanID_elevador2, MotorType.kBrushless);
  private SparkMax m_motor_brazo = new SparkMax(Elevador.kCanID_brazo, MotorType.kBrushless);
  private SparkMax m_motor_giro = new SparkMax(Elevador.kCanID_giro, MotorType.kBrushless);
  private SparkMax m_motor_taker = new SparkMax(Elevador.kCanID_taker, MotorType.kBrushless);

  // Encoders
  private RelativeEncoder Elevador_Encoder = m_motor_elevador.getEncoder();
  private RelativeEncoder BrazoEncoder = m_motor_brazo.getEncoder();
  private RelativeEncoder GiroEncoder = m_motor_giro.getEncoder();
  private DigitalInput coral_sensor = new DigitalInput(0);
  // private DigitalInput tomado2 = new DigitalInput(1);

  // private RelativeEncoder TakerEncoder = m_motor_taker.getEncoder();

  // Controladores
  private SparkClosedLoopController ElevadorController = m_motor_elevador.getClosedLoopController();
  private SparkClosedLoopController BrazoController = m_motor_brazo.getClosedLoopController();
  private SparkClosedLoopController GiroController = m_motor_giro.getClosedLoopController();

  private ProfiledPIDController m_controller_elevador = null;
  private ProfiledPIDController m_controller_brazo = null;
  private ProfiledPIDController m_controller_giro = null;
  private TrapezoidProfile.Constraints constraintsElevador = null;
  private TrapezoidProfile.Constraints constraintsBrazo = null;
  private TrapezoidProfile.Constraints constraintsGiro = null;

  // SparkClosedLoopController TakerController = m_motor_taker.getClosedLoopController();

  public ElevadorSubsystem() {
    m_brazo_current_position = PosicionBrazo.Home;
    m_elevador_current_position = PosicionElevador.Home;
    m_giro_current_position = PosicionGiro.Home;

    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig config2 = new SparkMaxConfig();
    SparkMaxConfig config_brazo = new SparkMaxConfig();
    SparkMaxConfig config_giro = new SparkMaxConfig();
    SparkMaxConfig config_taker = new SparkMaxConfig();

    constraintsElevador =
        new TrapezoidProfile.Constraints(
            Elevador.elevador_maxVelocity, Elevador.elevador_maxAcceleration);
    constraintsBrazo =
        new TrapezoidProfile.Constraints(
            Elevador.brazo_maxVelocity, Elevador.brazo_maxAcceleration);
    constraintsGiro =
        new TrapezoidProfile.Constraints(Elevador.giro_maxVelocity, Elevador.giro_maxAcceleration);

    m_controller_elevador =
        new ProfiledPIDController(
            Elevador.elevador_p, Elevador.elevador_i, Elevador.elevador_d, constraintsElevador);
    m_controller_brazo =
        new ProfiledPIDController(
            Elevador.brazo_p, Elevador.brazo_i, Elevador.brazo_d, constraintsBrazo);
    m_controller_giro =
        new ProfiledPIDController(
            Elevador.giro_p, Elevador.giro_i, Elevador.giro_d, constraintsGiro);

    config.inverted(false).idleMode(IdleMode.kBrake);

    config_brazo.inverted(false).idleMode(IdleMode.kBrake);

    config_giro.inverted(false).idleMode(IdleMode.kBrake);

    config_taker.inverted(false).idleMode(IdleMode.kBrake);

    config2.idleMode(IdleMode.kBrake).follow(Elevador.kCanID_elevador, true);

    m_motor_elevador2.configure(
        config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motor_elevador.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motor_brazo.configure(
        config_brazo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motor_giro.configure(
        config_giro, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_motor_taker.configure(
        config_taker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ResetEncoders();
  }

  @Override
  public void periodic() {
    posicion_brazo = BrazoEncoder.getPosition();
    posicion_elevador = Elevador_Encoder.getPosition();
    posicion_giro = GiroEncoder.getPosition();
    coral_tomado = coral_sensor.get();
    SmartDashboard.putNumber("Encoder Brazo", posicion_brazo);
    SmartDashboard.putNumber("Encoder Elevador", posicion_elevador);
    SmartDashboard.putNumber("Encoder Giro", posicion_giro);
    SmartDashboard.putString(
        "Posicion Elevador", elevadorPosicionesNombres[m_elevador_current_position.ordinal()]);
    SmartDashboard.putString(
        "Posicion Brazo", brazoPosicionesNombres[m_brazo_current_position.ordinal()]);
    SmartDashboard.putString(
        "Posicion Giro", giroPosicionesNombres[m_giro_current_position.ordinal()]);
  }

  private void ResetEncoders() {
    Elevador_Encoder.setPosition(0);
    BrazoEncoder.setPosition(0);
    GiroEncoder.setPosition(0);
  }

  private void stopAll() {
    m_motor_elevador.set(0);
    m_motor_elevador2.set(0);
    m_motor_brazo.set(0);
    m_motor_giro.set(0);
    m_motor_taker.set(0);
  }

  private void intakeMotorAction(boolean in, boolean out) {
    double output = 0;
    if (out) {
      output = 0.5;
    } else if (in) {
      output = -0.5;
      if (coral_tomado) {
        output = 0;
      }
    }
    m_motor_taker.set(output);
  }

  private void controlLoop() {
    double elevador_output = 0, brazo_output = 0, giro_output = 0;
    double elevador_setpoint = Elevador_Objetivo,
        brazo_setpoint = Brazo_Objetivo,
        giro_setpoint = Giro_Objetivo;

    m_controller_elevador.setGoal(elevador_setpoint);
    m_controller_brazo.setGoal(brazo_setpoint);
    m_controller_giro.setGoal(giro_setpoint);

    elevador_output = m_controller_elevador.calculate(posicion_elevador);
    brazo_output = m_controller_brazo.calculate(posicion_brazo);
    giro_output = m_controller_giro.calculate(posicion_giro);

    elevador_output = Math.min(1.0, Math.max(-1.0, elevador_output));
    brazo_output = Math.min(1.0, Math.max(-1.0, brazo_output));
    giro_output = Math.min(1.0, Math.max(-1.0, giro_output));

    m_motor_elevador.set(elevador_output);
    m_motor_brazo.set(brazo_output);
    m_motor_giro.set(giro_output);
  }

  private void setElevadorObjetivo(double nuevo_objetivo) {
    if (nuevo_objetivo > Elevador.elevador_maxSetPoint) {
      nuevo_objetivo = Elevador.elevador_maxSetPoint;
    } else if (nuevo_objetivo < Elevador.elevador_minSetPoint) {
      nuevo_objetivo = Elevador.elevador_minSetPoint;
    }

    Elevador_Objetivo = nuevo_objetivo;
  }

  private void setBrazoObjetivo(double nuevo_objetivo) {
    if (nuevo_objetivo > Elevador.brazo_maxSetPoint) {
      nuevo_objetivo = Elevador.brazo_maxSetPoint;
    } else if (nuevo_objetivo < Elevador.brazo_minSetPoint) {
      nuevo_objetivo = Elevador.brazo_minSetPoint;
    }

    Brazo_Objetivo = nuevo_objetivo;
  }

  private void setGiroObjetivo(double nuevo_objetivo) {
    if (nuevo_objetivo > Elevador.giro_maxSetPoint) {
      nuevo_objetivo = Elevador.giro_maxSetPoint;
    } else if (nuevo_objetivo < Elevador.giro_minSetPoint) {
      nuevo_objetivo = Elevador.giro_minSetPoint;
    }

    Giro_Objetivo = nuevo_objetivo;
  }

  private void manualElevador(double input) {
    double scaledValue = (input + (input < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    input = (Math.abs(input) > Math.abs(0.1)) ? scaledValue : 0;

    if (Math.abs(input) < 0.001) {
      return;
    }
    m_elevador_current_position = PosicionElevador.Manual;
    setElevadorObjetivo(posicion_elevador + (input * 10));
  }

  private void manualBrazo(double input) {
    double scaledValue = (input + (input < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    input = (Math.abs(input) > Math.abs(0.1)) ? scaledValue : 0;

    if (Math.abs(input) < 0.001) {
      return;
    }
    m_brazo_current_position = PosicionBrazo.Manual;
    setBrazoObjetivo(posicion_brazo + (input) * 10);
  }

  private void manualGiro(double input) {
    double scaledValue = (input + (input < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    input = (Math.abs(input) > Math.abs(0.1)) ? scaledValue : 0;

    if (Math.abs(input) < 0.001) {
      return;
    }
    m_giro_current_position = PosicionGiro.Manual;
    setGiroObjetivo(posicion_giro + (input) * 10);
  }

  private boolean elevadorEnPosicion(PosicionElevador posicion) {
    return Math.abs(posicion_elevador - elevadorMatrizPosicion[posicion.ordinal()]) < 0.1;
  }

  private boolean brazoEnPosicion(PosicionBrazo posicion) {
    return Math.abs(posicion_brazo - brazoMatrizPosicion[posicion.ordinal()]) < 0.1;
  }

  private boolean giroEnPosicion(PosicionGiro posicion) {
    return Math.abs(posicion_giro - giroMatrizPosicion[posicion.ordinal()]) < 0.1;
  }

  private boolean sistemaEnPosicion(
      PosicionElevador p_elevador, PosicionBrazo p_brazo, PosicionGiro p_giro) {
    return elevadorEnPosicion(p_elevador) && brazoEnPosicion(p_brazo) && giroEnPosicion(p_giro);
  }

  public Command mandarElevadorAPosicion(PosicionElevador posicion) {
    return runOnce(
        () -> {
          m_elevador_current_position = posicion;
          setElevadorObjetivo(elevadorMatrizPosicion[posicion.ordinal()]);
        });
  }

  public Command mandarBrazoAPosicion(PosicionBrazo posicion) {
    return runOnce(
        () -> {
          m_brazo_current_position = posicion;
          setBrazoObjetivo(brazoMatrizPosicion[posicion.ordinal()]);
        });
  }

  public Command mandarGiroAPosicion(PosicionGiro posicion) {
    return runOnce(
        () -> {
          m_giro_current_position = posicion;
          setGiroObjetivo(giroMatrizPosicion[posicion.ordinal()]);
        });
  }

  public Command mandarSistemaAPosiciones(
      PosicionElevador p_elevador, PosicionBrazo p_brazo, PosicionGiro p_giro) {
    return new SequentialCommandGroup(
        mandarElevadorAPosicion(p_elevador),
        mandarBrazoAPosicion(p_brazo),
        mandarGiroAPosicion(p_giro),
        controlLoopCommand(() -> 0, () -> 0, () -> 0, () -> false, () -> false)
            .until(() -> sistemaEnPosicion(p_elevador, p_brazo, p_giro))
            .withTimeout(3));
  }

  public Command tomarDeSource() {
    return mandarSistemaAPosiciones(
            PosicionElevador.Source, PosicionBrazo.Tomar, PosicionGiro.Horizontal)
        .andThen(
            controlLoopCommand(() -> 0, () -> 0, () -> 0, () -> true, () -> false)
                .until(() -> coral_tomado))
        .andThen(
            mandarSistemaAPosiciones(
                m_elevador_current_position, m_brazo_current_position, PosicionGiro.Vertical));
  }

  public Command dejarCoralEnNivel(int posicion) {
    return new SequentialCommandGroup();
  }

  public Command controlLoopCommand(
      DoubleSupplier raw_elevador,
      DoubleSupplier raw_brazo,
      DoubleSupplier raw_giro,
      BooleanSupplier tomar,
      BooleanSupplier dejar) {
    return run(() -> {
          manualElevador(raw_elevador.getAsDouble());
          manualBrazo(raw_brazo.getAsDouble());
          manualGiro(raw_giro.getAsDouble());
          intakeMotorAction(tomar.getAsBoolean(), dejar.getAsBoolean());
          controlLoop();
        })
        .finallyDo((interrupted) -> stopAll());
  }
}
