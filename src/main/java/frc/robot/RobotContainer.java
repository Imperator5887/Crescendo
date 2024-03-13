  /**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4OIConstants;
import frc.robot.Constants.limelightConstants.aprilTag;
import frc.robot.Constants.limelightConstants.trapAprilTag;
import frc.robot.commands.Mecanismos.ArmVelocityCommand;
import frc.robot.commands.Mecanismos.ClimbCommand;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PhotonLLCommand;
import frc.robot.commands.Mecanismos.PivoteoByInterpolation;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.ShooterButtonCmd;
import frc.robot.commands.Mecanismos.setPivotVelocity;
import frc.robot.commands.hybrid.autos;
import frc.robot.commands.hybrid.subroutines;
import frc.robot.commands.swerve.autoAlign;
import frc.robot.commands.swerve.swerveDriveComando;
import frc.robot.commands.swerve.swervePrecisionCommand;
import frc.robot.subsystems.Mecanismos.ClimberSubsystem;
import frc.robot.subsystems.Mecanismos.IntakeSubsystem;
import frc.robot.subsystems.Mecanismos.Pivoteo;
import frc.robot.subsystems.Mecanismos.ShooterSubsystem;
import frc.robot.subsystems.swerve.swerveSusbsystem;
import frc.robot.subsystems.vision.PhotonLL;

public class RobotContainer {

    private swerveSusbsystem swerve;
    private PhotonLL photoncamera;
    private Pivoteo arm;
    private ClimberSubsystem climber;
    private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;


    public static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    public static Joystick placerJoystick = new Joystick(OIConstants.kPlacerControllerPort);
    
    public RobotContainer(){

        
        
        swerve = swerveSusbsystem.getInstance();
        photoncamera = PhotonLL.getInstance();
        arm = Pivoteo.getInstance();
        climber = ClimberSubsystem.getInstance();
        m_intakeSubsystem = IntakeSubsystem.getInstance();
        m_shooterSubsystem = ShooterSubsystem.getInstance();


        // "save" a command in order to call it within an event marker.

        /* 
        swerveSubsystem.setDefaultCommand(new swerveDriveComando(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                true
                ));

        */
        swerve.setDefaultCommand(new swerveDriveComando(
                    swerve,
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true
                    ));


              //pivoteo deffault command:
       // arm.setDefaultCommand(new ArmVelocityCommand(0));
        // Intake:
        m_intakeSubsystem.setDefaultCommand(
            new IntakeButtonCmd(0)//
        );

        // shooter:
        m_shooterSubsystem.setDefaultCommand(
            new ShooterButtonCmd(0)//
        );

        /* comented to set deffault as "under the chain"
        climber.setDefaultCommand(
            new ClimbCommand(false)
        );
        */

        photoncamera.setDefaultCommand(new PhotonLLCommand());

                
              
        configureButtonBindings();

         
         
    }

   

    
    private void configureButtonBindings() {

       
         
       // ================================================== DRIVER JOYSTICKS  ========================================//

        // LOWER VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topLeft).whileFalse(
        new swervePrecisionCommand(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // INCREASE VELOCITY:
       new JoystickButton(driverJoytick, Constants.PS4OIConstants.topRight).whileFalse(
        new swerveDriveComando(swerve, 
        () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverYAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverXAxis),
                    () -> -driverJoytick.getRawAxis(PS4OIConstants.kDriverRotAxis),
                    () -> driverJoytick.getRawButton(PS4OIConstants.kDriverFieldOrientedButtonIdx),
                    true)
       );

        // RESEAT HEADING:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.triangle).whileFalse(
            new InstantCommand(
                () -> swerve.resetHeading()
            )
        );


      /* */  arm.setDefaultCommand(
            new ArmVelocityCommand(0)
        );
     
        // ALIGN TO APRILTAG:
        new JoystickButton(driverJoytick, Constants.PS4OIConstants.square).whileTrue(new autoAlign(trapAprilTag.constraints));
    

        // SET POSITION BY INTERPOLATION:
        //new JoystickButton(driverJoytick, Constants.PS4OIConstants.circle).whileTrue(new PivoteoByInterpolation());

       // ================================================== PLACER JOYSTICKS  =============================================//

       // +++++++++++++++++++ ÁNGULOS PIVOTEO ++++++++++++++++++++ //
        
        // --> LOW ARM <--
        new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(subroutines.lowArm());
    
        // --> ARM "DEFAULT" POSITION <--
        new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand(0.17));

       // +++++++++++++++ NOTE ROUTINES +++++++++++++++++ //

        // --> RECARGAR SHOOTER <--
        new JoystickButton(placerJoystick, PS4OIConstants.topLeft).whileTrue(new ShooterButtonCmd(0.75)); //
        
        // --> PUSH NOTE <--
        new JoystickButton(placerJoystick, PS4OIConstants.topRight).whileTrue(
            new IntakeButtonCmd(-0.5)
        );

        
        // ********** INTAKE ********** //

         // --> INTAKE WITH INFRARED <--
        new JoystickButton(placerJoystick, PS4OIConstants.cross).whileTrue(
            new IntakeButtonCmd(-0.4, true)
        );
        
        // --> BACKUP INTAKE <--
        new JoystickButton(placerJoystick, PS4OIConstants.triangle).whileTrue(new IntakeButtonCmd(-0.3));

        // ********** OUT TAKE ********** //
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.circle).whileTrue(new IntakeButtonCmd(0.6));
       
        /* ---> LOW ARM AND INTAKE <---
        new JoystickButton(placerJoystick, Constants.PS4OIConstants.cross).whileTrue(
            new ParallelCommandGroup(
                new IntakeButtonCmd(-0.5),
                new ArmVelocityCommand(0.075)
            )
        );
        */
        // +++++++++++++++ AMP SCORING +++++++++++++++ //

        // ---> STEP 1: TODO HASTA ATRÁS <-- 
        new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new PivoteoCommand(0.6));
        
        // --> STEP 3: MANTENER <--
        new JoystickButton(placerJoystick, PS4OIConstants.shareBtn).whileTrue(new PivoteoCommand(0.49)); // changed from 0.53 to 0.49

        // --> OUT TAKE <--
        new JoystickButton(placerJoystick, PS4OIConstants.triggerRight).whileTrue(new IntakeButtonCmd(0.5));
    }

    public Command getAutonomousCommand() {
       
        //return subroutines.lowArmAndShoot();
        //return null;
        //return AutoBuilder.buildAuto("1and2OnCenter");
        //return autos.threeNoteCenterV2();
        //return AutoBuilder.buildAuto("1and2OnCenterHermosillo");
        return autos.OneTwoThreeCenterShooting();
        //return AutoBuilder.buildAuto("angleTest");

    }


}

 // ======================================== X-BOX PLACER JOYSTICKS =================================================//
        // MECHANISMS
        //new JoystickButton(placerJoystick,9).whileTrue(new PivoteoCommand(0.075)); // 44°

       //new JoystickButton(placerJoystick,10).whileTrue(new PivoteoCommand(0.40)); // 

       // =================================== BUMPY 1.0 =================================================//

        //PID PIVOTING
       
       /*new JoystickButton(placerJoystick, PS4OIConstants.pad).whileTrue(new PivoteoCommand(0.23));

          //DESCOMENTAR PARA MATCH
         //86 PULGADAS A 60 GRADOS /// SI SIRVE /// TIRA DESDE LA NOTA DE ENMEDIO
         new JoystickButton(placerJoystick, Constants.PS4OIConstants.PSButton).whileTrue(new PivoteoCommand(0.19)); 

         // ÁNGULO TRAP:
         new JoystickButton(placerJoystick,Constants.PS4OIConstants.topLeft).whileTrue(new PivoteoCommand(0.154)); // 40° a 32.5in del trap encoder: 0.25 //0.122
 
         // UNDER THE CHAIN:
        new JoystickButton(placerJoystick,Constants.PS4OIConstants.triggerRight).whileTrue(new PivoteoCommand(0.43)); 
         
        // ÁNGULO CLIMBER:
         new JoystickButton(placerJoystick,Constants.PS4OIConstants.triggerRight).whileTrue(new PivoteoCommand(0.51)); // 90 grados (empujón)
        */
        
        //new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new PivoteoCommand(0.21));


        // --> APUNTAR <--
        // new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new ArmVelocityCommand(0.15));

        //new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand(0.2));
        // --> FIJAR <--
        //new JoystickButton(placerJoystick, PS4OIConstants.topLeft).whileTrue(new ArmVelocityCommand(0.075));

        // --> BAJAR <--
        //new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(new ArmVelocityCommand(-0.1));

        // --> ESCALAR <--
        //new JoystickButton(placerJoystick, PS4OIConstants.joystickDer).whileTrue(new ArmVelocityCommand(0.3));


        // ÁNGULO SHOOT:
       //new JoystickButton(placerJoystick,Constants.PS4OIConstants.joystickIzq).whileTrue(new PivoteoCommand( 0.075));// 30° // 0.19 //0.075
        // BAJAR BRAZO
        /* 

        // --> TEST 1 <-- SHOOT DE LEJOS
        //new JoystickButton(placerJoystick, PS4OIConstants.PSButton).whileTrue(new PivoteoCommand(0.2));
        //new JoystickButton(placerJoystick, PS4OIConstants.joystickIzq).whileTrue(subroutines.apuntarLejos());

        // SHOOT WITH DELAY
        //new JoystickButton(placerJoystick, Constants.PS4OIConstants.topRight).whileTrue(subroutines.shootWithDelay());


        new JoystickButton(placerJoystick, Constants.PS4OIConstants.square).whileTrue(new SequentialCommandGroup(
            new PivoteoCommand(0.26),
            new PivoteoCommand(0.061)
        )); // 44°+

        
        new POVButton(placerJoystick, 90).whileTrue(
            new InstantCommand(
                () -> arm.setAbsoluteEncoderZero(0)
            )
        );

    
        /*new JoystickButton(placerJoystick, PS4OIConstants.triangle).whileTrue(
            new InstantCommand(
            () -> arm.resetEncoder()
            )
        );*/

        // climber:+

        //new POVButton(placerJoystick, 90).whileTrue(new ClimbCommand(true));


        // =============================  SHOOTING POSITIONS TBC =========================================================== //
       // --> SUBWOOFER <----
       //new JoystickButton(placerJoystick, 10).whileTrue(new PivoteoCommand(0.061)); // 35°

       // --> ROBOT STARTING ZONE <---
       //new JoystickButton(placerJoystick, 1).whileTrue(new PivoteoCommand(0.0820)); // x
        // ============================================ Tiro al AMP (Boton de PS) ==========================================//
        
        
        //================================== APRIL TAG: ===================================================================//
       //new JoystickButton(driverJoytick, 5).whileTrue(new autoAlign());
       //new JoystickButton(driverJoytick, 2).whileTrue(new autoAlign(limelightConstants.noteOffsets.offsets));

       // ==================================================== PRUEBAS ==================================================== //

        // PRUEBA MOTOR:
        //new JoystickButton(placerJoystick, PS4OIConstants.square).whileTrue(new ShooterButtonCmd(-0.6));

       
        // AMP
        //new JoystickButton(placerJoystick, PS4OIConstants.triggerLeftBtn).whileTrue(new IntakeButtonCmd(0.5));
        //new JoystickButton(placerJoystick, PS4OIConstants.shareBtn).whileTrue(new IntakeButtonCmd(0.6));
        //new JoystickButton(placerJoystick, PS4OIConstants.optionsBtn).whileTrue(new IntakeButtonCmd(0.7));

        //comentado sig linea 09/03/24 8Mam
        //new JoystickButton(driverJoytick, PS4OIConstants.cross).whileTrue(new PivoteoCommand(0.25));
        //new JoystickButton(placerJoystick, PS4OIConstants.cross).whileTrue(new IntakeButtonCmd(-0.5));
        
        
        

        