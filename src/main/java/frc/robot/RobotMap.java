// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.lang.reflect.Field;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

/*
  JSON RobotMap config format:
  [
    {
      "VarName": "NAME_OF_VARIABLE",
      "Value": NEW_VALUE_OF_CORRESPONDING_TYPE
    }
  ]
*/

class RobotMapConfigValue {
  String varName = null;
  Object value = null;

  public void setVarName(String varName) {
      this.varName = varName;
  }

  public void setValue(Object value) {
      this.value = value;
  }
}

/** Add your docs here. */
public class RobotMap {

  public static void init()
  {
    try {
      String home = java.lang.System.getenv("HOME");
      if (home == null || home.isEmpty()) {
        home = "/home/lvuser";
      }
      else {
        System.out.println("$HOME is " + home);
      }
      File f = new File(home + "/1100_Config");
      if (f.exists())
      {
        ObjectMapper mapper = new ObjectMapper();
        List<RobotMapConfigValue> changedValues = mapper.readValue(f, new TypeReference<List<RobotMapConfigValue>>(){});

        for (int i=0; i < changedValues.size(); i++) {
          RobotMapConfigValue value = changedValues.get(i);
          try {
            Field field = RobotMap.class.getField(value.varName);
            field.set(null, value.value);
            System.out.println("Configuring " + value.varName + " to " + value.value);
          }
          catch (java.lang.NoSuchFieldException x) {
            System.out.println("No RobotMap field named " + value.varName);
          }
        }
      }  
      else
      {
        System.out.println("No configuration present, using defaults");
      }          
    } catch (Exception e) {
      System.err.println("exception from RobotMap.init():" + e.toString());
      // can't read the config. Carry on.
    }
  }
  // [R]obot geometry
  public static double R_TRACK_WIDTH_INCHES = 28.5;
  public static double R_WHEEL_BASE_INCHES  = 28.5;
  public static double R_BASE_RADIUS_INCHES = 14.25 * Math.sqrt(2);
  
  //[D]rive
  public static int D_FRONT_LEFT_DRIVE = RoboRioMap.CAN_2;
  public static int D_BACK_LEFT_DRIVE = RoboRioMap.CAN_4;
  public static int D_FRONT_RIGHT_DRIVE = RoboRioMap.CAN_8;
  public static int D_BACK_RIGHT_DRIVE = RoboRioMap.CAN_6;

  public static int D_FRONT_LEFT_TURNING = RoboRioMap.CAN_3;
  public static int D_BACK_LEFT_TURNING = RoboRioMap.CAN_5;
  public static int D_FRONT_RIGHT_TURNING = RoboRioMap.CAN_9;
  public static int D_BACK_RIGHT_TURNING = RoboRioMap.CAN_7;

  // [E]levator
  public static boolean E_ENABLED = false;
  public static int E_LEFTMOTOR = RoboRioMap.CAN_11;
  public static int E_RIGHTMOTOR = RoboRioMap.CAN_12;

  //[F]unnel
  public static boolean F_ENABLED = true;
  public static int F_LEFTMOTOR = RoboRioMap.CAN_13;
  public static int F_RIGHTMOTOR = RoboRioMap.CAN_14;

  // [L]ights
  public static int L_LEDS = RoboRioMap.PWM_0;
  
  //[U]ser Input
	public static int U_DRIVER_XBOX_CONTROLLER = 0;
	public static int U_OPERATOR_XBOX_CONTROLLER = 1;

  //[V]ision
  public static boolean V_ENABLED = false;

  //[W]oS
  public static boolean W_ENABLED = true;
  public static int W_MOTOR = RoboRioMap.CAN_10;
  public static int W_SHOULDERMOTOR = RoboRioMap.CAN_17;

  //[A]lgae Intake
  public static boolean A_ENABLED = false;
  public static int A_ANGLEMOTOR = RoboRioMap.CAN_15;
  public static int A_ROLLERMOTOR = RoboRioMap.CAN_16;
  public static int A_INTAKESENSOR = RoboRioMap.DIO_0;

  //[C]limber
  public static boolean C_ENABLED = false;
  public static int C_WINCHMOTOR = RoboRioMap.CAN_18;
}
