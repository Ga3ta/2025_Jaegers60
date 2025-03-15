// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;

  }

  public static class Elevador_Motores {
    public static final int kCanID_elevador = 15; //este tiene el encoder/
    public static final int kCanID_elevador2 = 16;
    public static final int kCanID_brazo = 19;
    public static final int kCanID_giro = 20;
    public static final int kCanID_taker = 21;
  }
  
  public static class Elevador_PID_Values {
    public static final double elevador_p = 0.1; 
    public static final double elevador_i = 0; 
    public static final double elevador_d = 0; 
    public static final double elevador_ff = 0.0001; 
    public static final double elevador_maxVelocity = 3000;
    public static final double elevador_maxAcceleration = 5000;
    public static final double elevador_LoopError = 0.5;
    public static final double elevador_minSetPoint = 0;
    public static final double elevador_maxSetPoint = 480;

    public static final double brazo_p = 0.2; 
    public static final double brazo_i = 0; 
    public static final double brazo_d = 0.001; 
    public static final double brazo_ff = 0.0001; 
    public static final double brazo_maxVelocity = 3000;
    public static final double brazo_maxAcceleration = 5000;
    public static final double brazo_LoopError = 0.2;
    public static final double brazo_minSetPoint = -7;
    public static final double brazo_maxSetPoint = 7;


    public static final double giro_p = 0.1; 
    public static final double giro_i = 0; 
    public static final double giro_d = 0; 
    public static final double giro_ff = 0.0001; 
    public static final double giro_maxVelocity = 3000;
    public static final double giro_maxAcceleration = 5000;
    public static final double giro_LoopError = 0.5;
    public static final double giro_minSetPoint = 0;
    public static final double giro_maxSetPoint = 3;
    
  }
  

  public static class Valores_Elevador {
                   
    //public static final double[][] Tomar = { 
    //  {0 , 33, 50}, 
    //  {0, 3, 0}, 
    //  {0, 4, 0},
    //  {0, 0, 0},  
    //  {0, 0, 150},
    //  {0, 0, 0} 
  //};
   // public static final double[][] Dejar = { 
    //  {150, 19, 0}, 
    //  {100, 2, 0}, 
    //  {0, 2, 0},
    //  {0, 0, 0},  
    //  {50, 19, 0}};
 // }

 // Valores Elevador / Brazo / Giro
  public static final double[][] Tomar = { 
    {100, 0, 0},   //Coral Human
    {130, 0, 0},    //Alga L2-L3
    {150, 0, 0},    //Alga L3-L4 
    {160, 0, 0},    //Coral Inicial
    {160, 0, 0},    //Coral Piso
    {160, 0, 0}    //Alga Piso  // 167 fue el maximo mecanico. Nueva trasnsusion x3  se deha en 480
};
  public static final double[][] Dejar = { 
    {90, 0, 0},   //L4 Dejar
    {75, 0, 0},   //L3 Dejar
    {60, 0, 0},   //L2 Dejar
    {50, 0, 0},  //L1 Dejar 
    {0, 0, 0}};   //Deja Alga
}
}
