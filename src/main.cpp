#include <Arduino.h>

// Configuration
const int REED_PIN = 5;                      // GPIO5 pour le Reed Switch
const int PULSES_PER_REV = 4;                // Nombre d'aimants sur le disque
const unsigned long RPM_CALC_INTERVAL = 1500; // Calcul RPM toutes les 1000ms
const unsigned long DEBOUNCE_TIME = 10;      // Anti-rebond 10ms

// Variables volatiles (accessibles depuis l'interruption)
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;

// Variables de calcul RPM
unsigned long lastRPMCalc = 0;
int rpm = 0;

// Interruption Reed Switch avec anti-rebond
void IRAM_ATTR compteur_pulses() {
  unsigned long currentTime = millis();
  
  // Anti-rebond : ignorer les impulsions trop rapprochées (rebonds mécaniques)
  if (currentTime - lastPulseTime > DEBOUNCE_TIME) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== TEST REED SWITCH RPM ===");
  Serial.println("GPIO: " + String(REED_PIN));
  Serial.println("Aimants par tour: " + String(PULSES_PER_REV));
  Serial.println("Debounce: " + String(DEBOUNCE_TIME) + "ms");
  Serial.println("==============================\n");
  
  // Configuration du pin avec pull-up interne
  pinMode(REED_PIN, INPUT_PULLUP);
  
  // Attachement de l'interruption sur front descendant (aimant s'approche)
  attachInterrupt(digitalPinToInterrupt(REED_PIN), compteur_pulses, FALLING);
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calcul du RPM toutes les RPM_CALC_INTERVAL ms
  if (currentTime - lastRPMCalc >= RPM_CALC_INTERVAL) {
    unsigned long deltaTime = currentTime - lastRPMCalc;
    
    // Lecture atomique du compteur (désactiver interruptions temporairement)
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;  // Reset du compteur
    interrupts();
    
    // Calcul RPM : (pulses × 60000) / (aimants_par_tour × temps_ms)
    if (pulses > 0) {
      rpm = (pulses * 60000) / (PULSES_PER_REV * deltaTime);
    } else {
      // Timeout : si pas de pulse depuis 500ms, afficher 0
      if (currentTime - lastPulseTime > 500) {
        rpm = 0;
      }
    }
    
    // Affichage dans le moniteur série
    Serial.print("Pulses: ");
    Serial.print(pulses);
    Serial.print("\tDeltaTime: ");
    Serial.print(deltaTime);
    Serial.print("ms\tRPM: ");
    Serial.print(rpm);
    Serial.print("\t| État pin: ");
    Serial.println(digitalRead(REED_PIN) == HIGH ? "OUVERT (aimant loin)" : "FERMÉ (aimant proche)");
    
    lastRPMCalc = currentTime;
  }
  
  // Petit délai pour ne pas surcharger le Serial
  delay(10);
}