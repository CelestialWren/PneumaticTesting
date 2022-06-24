// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.Pneumatics.DisableCompressor;
import frc.robot.commands.Pneumatics.EnableCompressor;
import frc.robot.subsystems.ShuffleboardInfo;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private Compressor compress;
  private DoubleSolenoid solenoid;
  private static Pneumatics instance;
  private PneumaticShuffleboard shuffleboard;

  private Pneumatics() {
     compress = new Compressor(PneumaticsModuleType.CTREPCM);
     solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
    shuffleboard = new PneumaticShuffleboard();
  }

  public static Pneumatics getInstance(){
    if(instance == null)
      instance = new Pneumatics();
    return  instance;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void enableCompressor(boolean state){
    if(state){
      compress.enableDigital();
    }
    else
      compress.disable();
      
  }

  public void changePosition(DoubleSolenoid.Value value){
    solenoid.set(value);
  }

  public void close(){
    compress.close();
    solenoid.close();
  }

  public  class PneumaticShuffleboard{
      ShuffleboardTab tab = ShuffleboardInfo.getInstance().getPneumaticControl();
      public PneumaticShuffleboard(){
ShuffleboardLayout controlLayout = tab.getLayout("Control", BuiltInLayouts.kList);
        controlLayout.add("Extend", new ExtendIntake());
        controlLayout.add("Retract", new RetractIntake());
        controlLayout.add("Float", new FloatIntake());
        ShuffleboardLayout compressorLayout = tab.getLayout("Compressor", BuiltInLayouts.kList);
        compressorLayout.add("Enable", new EnableCompressor());
        compressorLayout.add("Disable", new DisableCompressor());
      }
  }
  }
