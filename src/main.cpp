/*
 * Radar Doppler HB100 avec ESP32 (ADC interne)
 * Mesure de vitesse de ballon
 * 
 * Câblage:
 * HB100 IF Out → Ampli (AD620 + LM358) → ESP32 GPIO 36 (ADC1_CH0)
 * Signal amplifié: 0-3.3V, centré à 1.65V (VCC/2)
 */

#include <Arduino.h>
#include "arduinoFFT.h"

// Désactiver WiFi pour réduire consommation lors de tests sur alimentation ESP32 5V
#include <WiFi.h>

// Configuration ADC ESP32
#define ADC_PIN 36               // GPIO 36 (ADC1_CH0, VP) - Meilleur pour ADC
#define ADC_RESOLUTION 12        // 12 bits (0-4095)
#define ADC_VREF 3.3            // Tension de référence

// Configuration FFT
#define SAMPLES 1024             // Nombre d'échantillons (puissance de 2)
#define SAMPLING_FREQUENCY 10000 // 10 kHz - Bien au-dessus de Nyquist pour 3 kHz

// HB100 opère à 10.525 GHz
// Formule Doppler: fd = 2 * v * f0 / c
// où v = vitesse (m/s), f0 = 10.525 GHz, c = 3e8 m/s
// Coefficient: 70.166 Hz par m/s (ou 19.49 Hz par km/h)
#define DOPPLER_COEFF 70.166     // Hz par m/s

// Variables FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Timing
unsigned long samplingPeriodUs;
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Radar Doppler HB100 ===");
  Serial.println("Initialisation...");
  
  // Désactiver WiFi pour économiser ~100mA (utile si alimentation via ESP32 5V pin)
  WiFi.mode(WIFI_OFF);
  btStop();  // Désactiver Bluetooth aussi
  Serial.println("WiFi et Bluetooth désactivés (économie d'énergie)");
  
  // Configuration ADC ESP32
  pinMode(ADC_PIN, INPUT);
  analogReadResolution(ADC_RESOLUTION);  // 12 bits
  analogSetAttenuation(ADC_11db);        // 0-3.3V (atténuation 11dB)
  
  Serial.println("ADC ESP32 configuré");
  Serial.print("Pin ADC: GPIO ");
  Serial.println(ADC_PIN);
  Serial.print("Résolution: ");
  Serial.print(ADC_RESOLUTION);
  Serial.println(" bits (0-4095)");
  Serial.print("Plage: 0-");
  Serial.print(ADC_VREF);
  Serial.println("V");
  
  // Calculer la période d'échantillonnage
  samplingPeriodUs = round(1000000.0 / SAMPLING_FREQUENCY);
  
  Serial.println("\nConfiguration:");
  Serial.print("Echantillons: ");
  Serial.println(SAMPLES);
  Serial.print("Fréquence échantillonnage: ");
  Serial.print(SAMPLING_FREQUENCY);
  Serial.println(" Hz");
  Serial.print("Période échantillonnage: ");
  Serial.print(samplingPeriodUs);
  Serial.println(" µs");
  Serial.println("\nPrêt! Envoyez un ballon...\n");
  
  delay(2000);
}

void loop() {
  // Acquisition des échantillons
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long startTime = micros();
    
    // Lire le signal depuis ADC ESP32
    int adcValue = analogRead(ADC_PIN);  // 0-4095 (12 bits)
    
    // Centrer le signal autour de 0 (retirer offset DC)
    // Signal centré à VCC/2 → soustraire 2047 (milieu de 0-4095)
    vReal[i] = adcValue - 2048.0;
    vImag[i] = 0;  // Partie imaginaire à zéro
    
    // Attendre pour respecter la fréquence d'échantillonnage
    while (micros() - startTime < samplingPeriodUs) {
      // Attente active
    }
  }
  
  // Appliquer une fenêtre de Hamming pour réduire les fuites spectrales
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  
  // Calculer la FFT
  FFT.compute(FFTDirection::Forward);
  
  // Calculer les magnitudes
  FFT.complexToMagnitude();
  
  // Trouver la fréquence dominante
  double peakFrequency = FFT.majorPeak();
  
  // Calculer la vitesse
  // v = fd / coefficient
  double vitesse_ms = peakFrequency / DOPPLER_COEFF;      // m/s
  double vitesse_kmh = vitesse_ms * 3.6;                   // km/h
  
  // Trouver la magnitude du pic pour évaluer la confiance
  double maxMagnitude = 0;
  int maxIndex = 0;
  for (int i = 2; i < (SAMPLES / 2); i++) {  // Ignorer DC et très basses fréquences
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
      maxIndex = i;
    }
  }
  
  // Seuil de détection (ajuster selon vos besoins)
  const double DETECTION_THRESHOLD = 1000.0;  // À ajuster expérimentalement
  const double MIN_SPEED = 5.0;               // Vitesse minimale en km/h
  
  // Afficher uniquement si signal significatif
  if (maxMagnitude > DETECTION_THRESHOLD && vitesse_kmh > MIN_SPEED) {
    Serial.println("========================================");
    Serial.print("Fréquence Doppler: ");
    Serial.print(peakFrequency, 2);
    Serial.println(" Hz");
    
    Serial.print("VITESSE: ");
    Serial.print(vitesse_kmh, 1);
    Serial.print(" km/h (");
    Serial.print(vitesse_ms, 2);
    Serial.println(" m/s)");
    
    Serial.print("Signal: ");
    Serial.println(maxMagnitude, 0);
    Serial.println("========================================\n");
  } else {
    // Mode silencieux - afficher un point pour montrer que ça tourne
    Serial.print(".");
    delay(100);
  }
  
  delay(100);  // Petit délai entre les mesures
}
