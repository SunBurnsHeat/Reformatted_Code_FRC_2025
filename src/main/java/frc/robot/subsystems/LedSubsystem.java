package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

// public class LedSubsystem extends SubsystemBase {
//     private final AddressableLED ledBar;
//     private LEDPattern allianceLED;

//     private enum LedState { BLANK, RAINBOW, SCROLL, BREATHING, ALLIANCE_SOLID, GREEN, YELLOW }
//     private LedState currentState = LedState.BLANK;

//     private boolean hasCoral = false; 
//     private boolean ongoingCoral = false; 

//     public LedSubsystem() {
//         // Initialize alliance color
//         setAllianceColor();

//         // Initialize LED hardware
//         ledBar = new AddressableLED(LEDConstants.kLEDBarPWM);
//         ledBar.setLength(LEDConstants.ledLength);

//         // Start with blank pattern
//         setBlank();
//         ledBar.start();

//         // Register this subsystem with the command scheduler
//         this.register(); // Ensures periodic() is called automatically
//     }

//     @Override
//     public void periodic() {
//         // Update LED state based on robot mode and conditions
//         if (DriverStation.isDisabled()) {
//             setScroll();
//         } else if (DriverStation.isAutonomous()) {
//             setRainbow();
//         } else if (DriverStation.isTeleop()) {
//             // double timeRemaining = DriverStation.getMatchTime();
//             // if (timeRemaining > 0 && timeRemaining < 20) {
//             //     setBreathing();
//             // } else if (hasCoral) {
//             //     setGreen();
//             // } else if (ongoingCoral) { 
//             //     setYellow();
//             // } else {
//                 setAllianceSolid();
//             // }
//         }
//     }

//     private void setAllianceColor() {
//         DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
//         allianceLED = (alliance == DriverStation.Alliance.Red)
//             ? LEDPattern.solid(Color.kRed)
//             : LEDPattern.solid(Color.kBlue);
//     }

//     // Helper method to create a new buffer
//     private AddressableLEDBuffer createBuffer() {
//         return new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//     }

//     // Public methods with individual buffers
//     public void setBlank() {
//         if (currentState != LedState.BLANK) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = LEDPattern.solid(Color.kBlack);
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.BLANK;
//         }
//     }

//     public void setRainbow() {
//         if (currentState != LedState.RAINBOW) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.RAINBOW;
//         }
//     }

//     public void setScroll() {
//         if (currentState != LedState.SCROLL) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCoral, Color.kWheat)
//                 .scrollAtRelativeSpeed(Percent.per(Second).of(25));
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.SCROLL;
//         }
//     }

//     public void setBreathing() {
//         if (currentState != LedState.BREATHING) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = allianceLED.breathe(Second.of(2.5));
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.BREATHING;
//         }
//     }

//     public void setAllianceSolid() {
//         if (currentState != LedState.ALLIANCE_SOLID) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = allianceLED;
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.ALLIANCE_SOLID;
//         }
//     }

//     public void setGreen() {
//         if (currentState != LedState.GREEN) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = LEDPattern.solid(Color.kGreen);
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.GREEN;
//         }
//     }

//     public void setYellow() {
//         if (currentState != LedState.YELLOW) {
//             AddressableLEDBuffer buffer = createBuffer();
//             LEDPattern pattern = LEDPattern.solid(Color.kYellow);
//             pattern.applyTo(buffer);
//             ledBar.setData(buffer);
//             currentState = LedState.YELLOW;
//         }
//     }

//     // Removed setElevatorProgress() since it depended on ElevatorSubsystem

//     public void stop() {
//         ledBar.stop();
//     }

//     public void start() {
//         ledBar.start();
//     }

//     private static final LedSubsystem instance = new LedSubsystem();

//     public static LedSubsystem getInstance() {
//         return instance;
//     }
// }
public class LedSubsystem extends SubsystemBase {

    // private static ElevatorSubsystem elevator = new ElevatorSubsystem();
            
            private static AddressableLED ledBar; // instance of the led bar class
        
            // Buffer or the holder of the color's data and its variable
            private static AddressableLEDBuffer led_red_alliance;
            private static AddressableLEDBuffer led_blue_alliance;
            private static AddressableLEDBuffer led_red_blue;
            private static AddressableLEDBuffer led_blank;
            private static AddressableLEDBuffer led_green;
            private static AddressableLEDBuffer led_yellow;
            // private static AddressableLEDBuffer led_dynamic_msg;
            private static AddressableLEDBuffer rainbow_buffer;
            private static AddressableLEDBuffer elevator_progress_buffer;
            private static AddressableLEDBuffer scroll_buffer;
            private static AddressableLEDBuffer breath_buffer;
            private static AddressableLEDBuffer alliance_buffer;
            private static AddressableLEDBuffer autonomous_buffer;
            private static AddressableLEDBuffer blink_alliance_buffer;

            private static LEDPattern scrollBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kOrange, Color.kLime);
            private static LEDPattern allianceLED;
        
        
            // private static int dynamicCount = 0; // counter for dynamic messages
            // private static int rainbowFirstPixelHue = 0; // tracks the hue for the first pixel in the rainbow effect
            // public static boolean dynamicEffect = false; // a tracking varible for checking dynamic effect
        
            // a block that runs once when class is initially loaded
            static {
                setAllianceColor();

                ledBar = new AddressableLED(LEDConstants.kLEDBarPWM); // initializes the led bar object with the given power port
                ledBar.setLength(LEDConstants.ledLength);
        
                // set buffers to their specified length
        
                led_red_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                led_blue_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                led_red_blue = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                led_blank = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                rainbow_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                elevator_progress_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                scroll_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                breath_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                led_green = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                led_yellow = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                autonomous_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                blink_alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
                // led_dynamic_msg = new AddressableLEDBuffer(LEDConstants.ledBufferLength); // specific blinking message
        
                // initial message for led buffers
                for (int i = 0 ; i < LEDConstants.ledLength; i++){
                    // pre-set all message buffers during initilization
                    led_red_alliance.setLED(i, Color.kRed);
                    led_blue_alliance.setLED(i, Color.kBlue);
                    led_green.setLED(i, Color.kGreen);
                    led_yellow.setLED(i, Color.kYellow);
        
                    // using bitwise-AND operators .... (alternating red-blue led)
                    if (((i & 3) == 0) // if the last two binary value of i & 3 is the same as 0's
                     || ((i & 3) == 2) // if if the last two binary value of i & 3 is the same as 2's
                     ){
                        led_red_blue.setLED(i, Color.kRed);
                    }  else {
                        led_red_blue.setLED(i, Color.kBlue);
                    }
        
                    led_blank.setLED(i, Color.kBlack);
                    // led_dynamic_msg.setLED(i, Color.kBlack); // no pre-set for dynamic msg
                }
        
                ledBar.setData(led_blank); // displays the black-led at first
                ledBar.start(); // activates the led strip
            }
        
            public static void stopLedBar(){
                ledBar.stop(); // turns off
            }
        
            public static void startLedBar(){
                ledBar.start(); // turns on
            }
        
            public static void setBlankMsg(){
                // dynamicEffect = false;
                ledBar.setData(led_blank); // doesn't light up
            }
        
            public static void setRedBlueMsg(){
                // dynamicEffect = false; 
                ledBar.setData(led_red_blue); // lights up red_blue
            }
        
            public static void setGreenMsg(){
                // dynamicEffect = false;
                ledBar.setData(led_green); // lights up green
            }

            public static void setYellowMsg(){
                // dynamicEffect = false;
                ledBar.setData(led_yellow); // lights up green
            }
        
            public static void setAllianceColor(){ // sets allaince color in led
                DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
                if(friendlyAlliance == DriverStation.Alliance.Red){
                    allianceLED = LEDPattern.solid(Color.kRed);
                }
                else if(friendlyAlliance == DriverStation.Alliance.Blue){
                    allianceLED = LEDPattern.solid(Color.kBlue);
                }
                else {
                    allianceLED = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
                }
            }

            public static void setAllianceBlink() {
                // Use allianceLED and apply a fast blink (0.2s cycle = 5 Hz)
                LEDPattern blinkPattern = allianceLED.blink(Second.of(0.2)).atBrightness(Percent.of(100));
                
                // Apply to the blink buffer and update LED strip
                blinkPattern.applyTo(blink_alliance_buffer);
                ledBar.setData(blink_alliance_buffer);
            }

            public static void setAllianceSolid(){
                allianceLED.applyTo(alliance_buffer);
                ledBar.setData(alliance_buffer);
                ledBar.setData(alliance_buffer);
            }
            public static void setAutonomousPattern() {
                // Base layer: Dim gold background
                LEDPattern baseLayer = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(20));

                // Chase layer: Bright purple pulses moving across the strip
                LEDPattern chaseLayer = LEDPattern.solid(Color.kBlue)
                    .breathe(Second.of(1)) // Pulse every 1 second
                    .scrollAtRelativeSpeed(Percent.per(Second).of(50)) // Move at 50% speed
                    .atBrightness(Percent.of(80)); // Bright pulses

                // Combine layers: Overlay chase on base
                LEDPattern autonomousPattern = chaseLayer.overlayOn(baseLayer);

                // Apply to buffer and update LED strip
                autonomousPattern.applyTo(autonomous_buffer);
                ledBar.setData(autonomous_buffer);
            }
        
            public static void scrollMsg(){
                LEDPattern pattern = scrollBase.scrollAtRelativeSpeed(Percent.per(Second).of(5));
        
                pattern.applyTo(scroll_buffer);
        
                ledBar.setData(scroll_buffer);
            }
        
            public static void setBreathingMsg(){
                LEDPattern breathing = allianceLED.breathe(Second.of(
                .5)).atBrightness(Percent.of(30));
        
                breathing.applyTo(breath_buffer);
        
                ledBar.setData(breath_buffer);
            }
        
            public static void setRainbow(){
                LEDPattern rainbow = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
                rainbow.applyTo(rainbow_buffer);
                ledBar.setData(rainbow_buffer);
            }

            public static void setPoliceLights() {
                // Static variables to maintain state
                double lastChangeTime = 0;
                boolean isFirstPhase = true;
                final double BLINK_INTERVAL = 0.15; // 150ms per phase (~6.67 Hz cycle)
                final int HALF_LENGTH = LEDConstants.ledLength / 2;

                // Get current time
                double currentTime = Timer.getFPGATimestamp();

                // Switch phases
                if (currentTime - lastChangeTime >= BLINK_INTERVAL) {
                    isFirstPhase = !isFirstPhase;
                    lastChangeTime = currentTime;
                }

                // Set alternating patterns
                for (int i = 0; i < LEDConstants.ledLength; i++) {
                    if (isFirstPhase) {
                        // First half red, second half blue
                        if (i < HALF_LENGTH) {
                            blink_alliance_buffer.setLED(i, Color.kRed);
                        } else {
                            blink_alliance_buffer.setLED(i, Color.kBlue);
                        }
                    } else {
                        // First half blue, second half red
                        if (i < HALF_LENGTH) {
                            blink_alliance_buffer.setLED(i, Color.kBlue);
                        } else {
                            blink_alliance_buffer.setLED(i, Color.kRed);
                        }
                    }
                }

                // Update LED strip
                ledBar.setData(blink_alliance_buffer);
            }
        
            // public static void elevatorMsg(){
            //     LEDPattern elevatorGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTurquoise, Color.kNavy);
            //     LEDPattern elevatoPattern = LEDPattern.progressMaskLayer(() -> elevator.getPosition()/ElevatorConstants.kElevatorMaxHeightRaw).overlayOn(elevatorGradient);
            //     elevatoPattern.applyTo(elevator_progress_buffer);
            //     ledBar.setData(elevator_progress_buffer);
            // }

    // public static void setRainbow(){ // sets rainbow led
    //     // For every pixel
    //     for (var i = 0; i < led_dynamic_msg.getLength(); i++) {
    //       // Calculate the hue - hue is easier for rainbows because the color
    //       // shape is a circle so only one value needs to precess
    //       final var hue = (rainbowFirstPixelHue + (i * 180 / led_dynamic_msg.getLength())) % 180;
    //       // Set the value
    //       led_dynamic_msg.setHSV(i, hue, 255, 128);
    //     }
    //     // Increase by to make the rainbow "move"
    //     rainbowFirstPixelHue += 3;
    //     // Check bounds
    //     rainbowFirstPixelHue %= 180;
    // }

    // public static void setDynamicMsg(){
    //     if ( dynamicCount < 1000 ){
    //         dynamicCount++;
    //     }
    //     setRainbow();
    //     ledBar.setData(led_dynamic_msg);
    // }
}
