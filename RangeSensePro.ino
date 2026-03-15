/*
 * RangeSensePro - Professional Ultrasonic Distance Sensing Library
 * Features: OOP, Hybrid Filtering (Median + EMA), Noise Detection (SD), Non-blocking
 * Author: Emre Ucar
 * Contact: [Your Email/Github if you want]
 * License: MIT
 */

#include <Arduino.h>

// Define the size of the sliding window for filtering. Must be an odd number.
#define FILTER_WINDOW_SIZE 5 

class UltrasonicSensor {
  private:
    uint8_t triggerPin;
    uint8_t echoPin;
    
    // Circular buffer to store raw readings
    uint16_t readingHistory[FILTER_WINDOW_SIZE];
    uint8_t bufferIndex = 0;
    
    // Filtered value stored as float for precision during calculations
    float filteredDistanceCm = 0.0;
    
    // EMA (Exponential Moving Average) alpha coefficient (0.0 to 1.0).
    // Lower means smoother but slower response.
    float emaAlpha = 0.2; 
    
    // Timing variables for non-blocking execution
    unsigned long lastUpdateTime = 0;
    const unsigned long updateIntervalMs = 60; // HC-SR04 needs at least 60ms between readings

    // Maximum expected distance to reject obvious outliers before filtering
    const uint16_t maxValidDistanceCm = 400; 

    // Helper function: Simple Bubble Sort for Median calculation
    // Sorts an array in ascending order. Time complexity: O(N^2)
    void performBubbleSort(uint16_t arr[], uint8_t n) {
      for (uint8_t i = 0; i < n - 1; i++) {
        for (uint8_t j = 0; j < n - i - 1; j++) {
          if (arr[j] > arr[j + 1]) {
            uint16_t temp = arr[j];
            arr[j] = arr[j + 1];
            arr[j + 1] = temp;
          }
        }
      }
    }

  public:
    // Constructor: Initializes pins and prepares the history buffer
    UltrasonicSensor(uint8_t trig, uint8_t echo) : triggerPin(trig), echoPin(echo) {
      // Set pin modes
      pinMode(triggerPin, OUTPUT);
      pinMode(echoPin, INPUT);
      
      // Initialize buffer with a reasonable starting value
      for(uint8_t i=0; i<FILTER_WINDOW_SIZE; i++) {
        readingHistory[i] = 20; // Assume 20cm initial distance
      }
      filteredDistanceCm = 20.0;
    }

    // Function to calculate the Standard Deviation (SD) of the history buffer.
    // Used for real-time sensor stability assessment.
    float getStandardDeviation() {
      float sum = 0.0, mean = 0.0, varianceSum = 0.0;
      
      // 1. Calculate the arithmetic mean
      for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; ++i) {
        sum += readingHistory[i];
      }
      mean = sum / (float)FILTER_WINDOW_SIZE;

      // 2. Calculate sum of squared differences from the mean
      for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; ++i) {
        varianceSum += pow(readingHistory[i] - mean, 2);
      }
      
      // 3. Return the square root of the average of variance
      return sqrt(varianceSum / (float)FILTER_WINDOW_SIZE);
    }

    // Core function to update the sensor reading. Should be called frequently in loop().
    // Implements a non-blocking hybrid filter (Median + EMA).
    // Returns: Current filtered distance in centimeters.
    uint16_t updateAndGetDistance() {
      // Check if it's time to take a new reading (non-blocking)
      if (millis() - lastUpdateTime >= updateIntervalMs) {
        lastUpdateTime = millis();

        // --- PHASE 1: PRE-FILTERING (RAW DATA ACQUISITION) ---
        // Trigger the sensor by sending a 10us HIGH pulse
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);

        // Read the echo pulse duration in microseconds
        unsigned long pulseDurationUs = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

        // Convert pulse duration to distance (Speed of sound is ~343 m/s)
        // Distance (cm) = (Duration / 2) / 29.1
        uint16_t rawDistanceCm = (pulseDurationUs / 2) / 29.1;

        // Basic outlier rejection: if reading is 0 (timeout) or too high, 
        // use previous valid filtered value to prevent massive spikes.
        if (rawDistanceCm == 0 || rawDistanceCm > maxValidDistanceCm) {
          rawDistanceCm = (uint16_t)filteredDistanceCm; 
        }

        // Add raw data to the circular buffer
        readingHistory[bufferIndex] = rawDistanceCm;
        bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE; // Wrap around index

        // --- PHASE 2: MEDIAN FILTERING (REJECTS IMPULSE NOISE) ---
        // Create a temporary copy of the history buffer for sorting
        uint16_t sortedArray[FILTER_WINDOW_SIZE];
        for(uint8_t i=0; i<FILTER_WINDOW_SIZE; i++) {
          sortedArray[i] = readingHistory[i];
        }
        
        // Sort the array to find the median value
        performBubbleSort(sortedArray, FILTER_WINDOW_SIZE);
        
        // The median is the middle element of the sorted array
        uint16_t medianValue = sortedArray[FILTER_WINDOW_SIZE / 2];

        // --- PHASE 3: EMA FILTERING (SMOOTHING) ---
        // Apply Exponential Moving Average on the median value
        // New_Filtered = (Alpha * Median) + ((1 - Alpha) * Previous_Filtered)
        filteredDistanceCm = (emaAlpha * medianValue) + ((1.0 - emaAlpha) * filteredDistanceCm);
      }
      
      // Return the final filtered value cast to an integer
      return (uint16_t)filteredDistanceCm;
    }
};

// --- USAGE EXAMPLE ---

// Define hardware pins
const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 10;

// Instantiate the sensor object
UltrasonicSensor distanceSensor(TRIG_PIN, ECHO_PIN);

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Call update frecuently. It handles its own non-blocking timing.
  uint16_t currentDistance = distanceSensor.updateAndGetDistance();
  
  // Get the stability metric (lower is better)
  float stabilityMetric = distanceSensor.getStandardDeviation();

  // Print data to Serial Monitor every second (non-blocking)
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm | Stability (SD): ");
    Serial.println(stabilityMetric);

    // Basic diagnostics example based on Stability
    if(stabilityMetric > 15.0 && currentDistance < 100) {
       Serial.println(">>> ALERT: Unstable Reading! Check sensor connections or target surface.");
    }
    
    lastPrintTime = millis();
  }
  
  // Your other loop code can run freely here...
}