/*
 * ============================================================================
 * DYNAMIC SPEED BUMP - FINAL SOLUTION
 * ============================================================================
 * 
 * COMPONENTS (EXACTLY AS SPECIFIED):
 * - 1x ESP8266 (NodeMCU)
 * - 1x Photoresistor (ONLY ONE! ESP8266 has 1 ADC pin)
 * - 1x 10kΩ Resistor
 * - 2x Servo motors
 * 
 * HOW IT WORKS:
 * - ONE photoresistor mounted OVERHEAD looking down at ONE point on road
 * - Measures time car blocks the sensor
 * - Uses STANDARD CAR LENGTH (4.2 meters) to calculate speed
 * - ASSUMES ONLY CARS (no trucks, no motorcycles)
 * - Trucks are IGNORED (they go slow anyway)
 * 
 * SPEED FORMULA:
 * Speed (km/h) = (Car Length in meters / Time in seconds) × 3.6
 * 
 * EXAMPLE:
 * Standard car length = 4.2 meters
 * Time under sensor = 0.5 seconds
 * Speed = (4.2 / 0.5) × 3.6 = 30.24 km/h
 * 
 * ============================================================================
 */

#include <Servo.h>

// ============================================================================
// PIN DEFINITIONS (ESP8266 - ONLY ONE ADC!)
// ============================================================================

#define PHOTORESISTOR_PIN A0    // ONLY analog pin on ESP8266!
#define SERVO_LEFT_PIN D1       // Left servo (normal orientation)
#define SERVO_RIGHT_PIN D2      // Right servo (reversed orientation)

// ============================================================================
// FIXED PARAMETERS (DO NOT CHANGE - These are physics constants)
// ============================================================================

const float STANDARD_CAR_LENGTH_METERS = 4.2;  // Average sedan/car length
                                                // Range: 3.8m (small) to 4.5m (large)
                                                // 4.2m is perfect middle ground

// ============================================================================
// ADJUSTABLE SETTINGS (Change these based on your road)
// ============================================================================

const float SPEED_LIMIT_KMH = 30.0;             // Speed limit for cars

// Convert speed limit to minimum time under sensor
// Formula: Time (seconds) = Car Length (m) / Speed (m/s)
const float SPEED_LIMIT_MS = SPEED_LIMIT_KMH / 3.6;           // Convert to m/s
const float MIN_TIME_UNDER_SENSOR_SEC = STANDARD_CAR_LENGTH_METERS / SPEED_LIMIT_MS;
const unsigned long MIN_TIME_UNDER_SENSOR_MS = MIN_TIME_UNDER_SENSOR_SEC * 1000;

// Example calculation:
// If SPEED_LIMIT = 30 km/h:
//   Speed limit in m/s = 30 / 3.6 = 8.33 m/s
//   Car length = 4.2 meters
//   Minimum safe time = 4.2 / 8.33 = 0.504 seconds = 504 ms
//   
//   IF time < 504 ms → OVERSPEED (raise bump)
//   IF time ≥ 504 ms → SAFE (flat bump)

// Sensor Settings
const int LDR_THRESHOLD = 400;                   // Below this = car detected
                                                 // Run calibration to set correctly

// Timing Settings
const unsigned long DEBOUNCE_MS = 50;            // Prevent false triggers
const unsigned long BUMP_RAISED_DURATION_MS = 3000;  // Bump stays raised for 3 seconds
const unsigned long MAX_CAR_TIME_MS = 2000;      // Max time for a car (2 seconds)
                                                 // If >2 seconds, assume truck/error

// Servo Settings
const int SERVO_NORMAL_RAISED = 90;              // Left servo: 90° = raised
const int SERVO_NORMAL_FLAT = 0;                // Left servo: 0° = flat
const int SERVO_REVERSED_RAISED = 0;            // Right servo: 0° = raised (reversed)
const int SERVO_REVERSED_FLAT = 90;             // Right servo: 90° = flat (reversed)

// Debug Settings
const bool ENABLE_DEBUG = true;                  // Set to false to disable serial output

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Servo servoLeft;
Servo servoRight;

// Vehicle detection
bool carCurrentlyDetected = false;
unsigned long carStartTime = 0;
unsigned long carEndTime = 0;
unsigned long timeUnderSensor = 0;

// Speed calculation
float calculatedSpeedKMH = 0;
bool isOverspeed = false;

// Statistics
int totalCarsCount = 0;
int overspeedCount = 0;
int trucksIgnored = 0;

// Bump state
bool bumpRaised = false;
unsigned long bumpRaisedTime = 0;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  printHeader();
  
  // Initialize sensor pin
  pinMode(PHOTORESISTOR_PIN, INPUT);
  
  // Initialize servos
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  
  // Start with bump flat
  servoLeft.write(SERVO_NORMAL_FLAT);
  servoRight.write(SERVO_REVERSED_FLAT);
  delay(500);
  
  // Run calibration
  calibrateSensor();
  
  printSettings();
  printReady();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read the ONLY analog sensor
  int sensorValue = analogRead(PHOTORESISTOR_PIN);
  bool carDetected = (sensorValue < LDR_THRESHOLD);
  
  // Debug: Print sensor values (uncomment if needed)
  // static unsigned long lastPrint = 0;
  // if (ENABLE_DEBUG && millis() - lastPrint > 1000) {
  //   Serial.print("Sensor: ");
  //   Serial.println(sensorValue);
  //   lastPrint = millis();
  // }
  
  // DETECT CAR ARRIVAL (No car → Car appears)
  if (!carCurrentlyDetected && carDetected) {
    handleCarArrival();
  }
  
  // DETECT CAR DEPARTURE (Car → No car)
  if (carCurrentlyDetected && !carDetected) {
    handleCarDeparture();
  }
  
  // Update current state
  carCurrentlyDetected = carDetected;
  
  // MANAGE BUMP RESET TIMER
  if (bumpRaised && (millis() - bumpRaisedTime > BUMP_RAISED_DURATION_MS)) {
    lowerBump();
    bumpRaised = false;
    
    if (ENABLE_DEBUG) {
      Serial.println("[SYSTEM] ✅ Ready for next car");
      Serial.println("═══════════════════════════════════════════════════════════\n");
    }
  }
  
  delay(10);  // Small delay for stability
}

// ============================================================================
// CORE DETECTION FUNCTIONS
// ============================================================================

void handleCarArrival() {
  // Debounce to prevent multiple triggers
  static unsigned long lastTrigger = 0;
  if (millis() - lastTrigger < DEBOUNCE_MS) return;
  lastTrigger = millis();
  
  carStartTime = millis();
  
  if (ENABLE_DEBUG) {
    Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
    Serial.println("║                    🚗 CAR DETECTED                        ║");
    Serial.println("╚═══════════════════════════════════════════════════════════╝");
    Serial.println("[STATUS] Car passing over sensor");
    Serial.println("[TIMING] ⏱️  Measuring time under sensor...");
  }
}

void handleCarDeparture() {
  // Debounce
  static unsigned long lastTrigger = 0;
  if (millis() - lastTrigger < DEBOUNCE_MS) return;
  lastTrigger = millis();
  
  carEndTime = millis();
  timeUnderSensor = carEndTime - carStartTime;
  
  // CHECK IF THIS IS A TRUCK (too long under sensor)
  if (timeUnderSensor > MAX_CAR_TIME_MS) {
    // This is likely a truck or bus - IGNORE IT
    trucksIgnored++;
    
    if (ENABLE_DEBUG) {
      Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      Serial.println("⚠️  VEHICLE IGNORED (Likely a truck/bus)");
      Serial.print("   Time under sensor: ");
      Serial.print(timeUnderSensor);
      Serial.println(" ms");
      Serial.println("   Trucks go slow naturally - No action needed");
      Serial.print("   Trucks ignored so far: ");
      Serial.println(trucksIgnored);
      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }
    return;  // Exit without processing as a car
  }
  
  // PROCESS AS A CAR
  totalCarsCount++;
  
  // Calculate speed using STANDARD CAR LENGTH
  float timeSeconds = timeUnderSensor / 1000.0;
  float speedMS = STANDARD_CAR_LENGTH_METERS / timeSeconds;
  calculatedSpeedKMH = speedMS * 3.6;
  
  // Check if overspeed
  isOverspeed = (calculatedSpeedKMH > SPEED_LIMIT_KMH);
  
  if (isOverspeed) {
    overspeedCount++;
  }
  
  // Display results
  printResults();
  
  // DECISION: Raise bump or keep flat
  if (isOverspeed) {
    Serial.println("[DECISION] ⚠️  CAR OVERSPEED! Raising speed bump...");
    raiseBump();
    bumpRaised = true;
    bumpRaisedTime = millis();
  } else {
    Serial.println("[DECISION] ✅ Car at safe speed - Bump stays FLAT");
  }
  
  // Print statistics
  printStatistics();
  Serial.println("═══════════════════════════════════════════════════════════\n");
}

// ============================================================================
// SPEED BUMP CONTROL FUNCTIONS
// ============================================================================

void raiseBump() {
  servoLeft.write(SERVO_NORMAL_RAISED);
  servoRight.write(SERVO_REVERSED_RAISED);
  
  if (ENABLE_DEBUG) {
    Serial.println("[ACTION] 🔼 SPEED BUMP RAISED!");
    Serial.print("[TIMER] ⏱️  Bump will stay raised for ");
    Serial.print(BUMP_RAISED_DURATION_MS / 1000);
    Serial.println(" seconds");
  }
}

void lowerBump() {
  servoLeft.write(SERVO_NORMAL_FLAT);
  servoRight.write(SERVO_REVERSED_FLAT);
  
  if (ENABLE_DEBUG) {
    Serial.println("[ACTION] 🔽 Speed bump lowered to FLAT");
  }
}

// ============================================================================
// DISPLAY AND PRINTING FUNCTIONS
// ============================================================================

void printResults() {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("📊 CAR MEASUREMENT RESULTS:");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("   ⏱️  Time under sensor: ");
  Serial.print(timeUnderSensor);
  Serial.println(" ms");
  Serial.print("   📏 Car length (assumed): ");
  Serial.print(STANDARD_CAR_LENGTH_METERS);
  Serial.println(" meters");
  Serial.print("   🚗 Calculated speed: ");
  Serial.print(calculatedSpeedKMH, 1);
  Serial.println(" km/h");
  Serial.print("   📊 Speed limit: ");
  Serial.print(SPEED_LIMIT_KMH);
  Serial.println(" km/h");
  Serial.print("   🚗 Car #");
  Serial.println(totalCarsCount);
  
  if (isOverspeed) {
    Serial.println("   ⚠️  VERDICT: OVERSPEED");
  } else {
    Serial.println("   ✅ VERDICT: Safe speed");
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void printStatistics() {
  Serial.println("\n📈 RUNNING STATISTICS:");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("   🚗 Total cars: ");
  Serial.println(totalCarsCount);
  Serial.print("   ⚠️  Overspeed cars: ");
  Serial.print(overspeedCount);
  
  if (totalCarsCount > 0) {
    float percent = (overspeedCount * 100.0) / totalCarsCount;
    Serial.print(" (");
    Serial.print(percent, 1);
    Serial.println("%)");
  }
  
  if (trucksIgnored > 0) {
    Serial.print("   🚛 Trucks ignored: ");
    Serial.println(trucksIgnored);
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void printHeader() {
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║     DYNAMIC SPEED BUMP - FINAL SOLUTION                   ║");
  Serial.println("║     ONE Photoresistor | ONLY Cars | Trucks Ignored        ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
}

void printSettings() {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("⚙️  SYSTEM CONFIGURATION:");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("   📏 Standard car length: ");
  Serial.print(STANDARD_CAR_LENGTH_METERS);
  Serial.println(" meters");
  Serial.print("   ⚡ Speed limit: ");
  Serial.print(SPEED_LIMIT_KMH);
  Serial.println(" km/h");
  Serial.print("   ⏱️  Safe time threshold: ");
  Serial.print(MIN_TIME_UNDER_SENSOR_MS);
  Serial.println(" ms");
  Serial.println("");
  Serial.println("   🚗 HOW IT WORKS:");
  Serial.println("      - Time < threshold → OVERSPEED → Raise bump");
  Serial.println("      - Time ≥ threshold → SAFE → Flat bump");
  Serial.println("      - Time > 2 seconds → TRUCK → Ignore");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void printReady() {
  Serial.println("\n✅ SYSTEM READY!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("📐 PHYSICAL SETUP:");
  Serial.println("   1. Mount photoresistor 3-5 meters ABOVE the road");
  Serial.println("   2. Point it straight DOWN at one spot");
  Serial.println("   3. Connect to ESP8266 pin A0 (ONLY analog pin)");
  Serial.println("");
  Serial.println("🚗 Waiting for cars...");
  Serial.println("   (Trucks will be automatically ignored)");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void calibrateSensor() {
  Serial.println("\n--- 🔧 CALIBRATION ---");
  Serial.println("Ensure NO car or truck is near the sensor");
  delay(2000);
  
  int sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(PHOTORESISTOR_PIN);
    delay(50);
  }
  
  int ambient = sum / 20;
  int suggestedThreshold = ambient - 100;
  
  Serial.print("   Ambient light reading: ");
  Serial.println(ambient);
  Serial.print("   Suggested LDR_THRESHOLD: ");
  Serial.println(suggestedThreshold);
  Serial.print("   Current threshold: ");
  Serial.println(LDR_THRESHOLD);
  
  if (suggestedThreshold != LDR_THRESHOLD) {
    Serial.println("   ⚠️  UPDATE THIS in code for best results:");
    Serial.print("      const int LDR_THRESHOLD = ");
    Serial.println(suggestedThreshold);
    Serial.println("   Then re-upload the code");
  } else {
    Serial.println("   ✅ Threshold is optimal!");
  }
  Serial.println("----------------------------------------\n");
}

// ============================================================================
// END OF CODE
// ============================================================================