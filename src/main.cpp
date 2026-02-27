/*
 * Radar Doppler CMD324 avec ESP32 (ADC interne)
 * Mesure de vitesse de ballon
 * 
 * Câblage:
 * CMD324 IF Out → Ampli (AD620 + LM358) → ESP32 GPIO 36 (ADC1_CH0)
 * Signal amplifié: 0-3.3V, centré à 1.65V (VCC/2)
 */

#include <Arduino.h>
#include "arduinoFFT.h"
#include <U8g2lib.h>
#include <SPI.h>

// Désactiver WiFi pour réduire consommation lors de tests sur alimentation ESP32 5V
#include <WiFi.h>

// Configuration ADC ESP32
#define ADC_PIN 36               // GPIO 36 (ADC1_CH0, VP) - Meilleur pour ADC
#define ADC_RESOLUTION 12        // 12 bits (0-4095)
#define ADC_VREF 3.3            // Tension de référence

// Configuration FFT
#define SAMPLES 1024             // Nombre d'échantillons (puissance de 2)
#define SAMPLING_FREQUENCY 10000 // 10 kHz - Bien au-dessus de Nyquist pour 3 kHz

// Mode Debug - Affiche toutes les valeurs (même faibles)
#define DEBUG_MODE true          // Mettre false après calibration

// CMD324 opère à 24.125 GHz (bande K)
// Formule Doppler: fd = 2 * v * f0 / c
// où v = vitesse (m/s), f0 = 24.125 GHz, c = 3e8 m/s
// Coefficient: 160.83 Hz par m/s (ou 44.68 Hz par km/h)
#define DOPPLER_COEFF 160.83     // Hz par m/s

// Variables FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Timing
unsigned long samplingPeriodUs;
unsigned long lastSampleTime = 0;

// Anti-bruit : nombre de détections consécutives requises avant validation
// NOTE : 1 = réactif (objets rapides), 2 = plus sûr (moins de faux positifs)
int consecutiveDetections = 0;
const int REQUIRED_CONFIRMATIONS = 1;

// Fréquences parasites connues à ignorer (ex: bruit secteur 50 Hz et harmoniques)
// 1770 Hz = bruit identifié, on exclut une plage autour
#define NOISE_FREQ_MIN 1700.0   // Hz - début zone bruit identifiée
#define NOISE_FREQ_MAX 1850.0   // Hz - fin zone bruit identifiée

// --- Seuils de détection (calibrer ici) ---
// Signal bruit < 1200, main ~1360+, ballon estimé > 3000
#define DETECTION_THRESHOLD 1000.0   // Monter si faux positifs persistent
#define MIN_SPEED           2.0       // Vitesse minimale en km/h

// --- Correction d'angle ---
// Capteur placé à 30° de la trajectoire du ballon
// v_réelle = v_mesurée / cos(angle)
// cos(0°)=1.000  cos(15°)=0.966  cos(30°)=0.866  cos(45°)=0.707
#define ANGLE_DEGREES       0.0
#define ANGLE_CORRECTION    (1.0 / cos(ANGLE_DEGREES * PI / 180.0))  // = 1.155 à 30°

// --- Ecran LCD ST7565R GMG12864-06D (128x64, SPI) ---
// Pins SPI hardware ESP32 (VSPI) : SCK=18, MOSI=23 (câblés directement)
#define LCD_CS   5   // Chip Select
#define LCD_DC   2   // Data/Command (A0 / RS)
#define LCD_RST  4   // Reset
// Si l'affichage est vide/inversé, remplacer par : U8G2_ST7565_ERC12864_F_4W_HW_SPI
U8G2_ST7565_NHD_C12864_F_4W_HW_SPI u8g2(U8G2_R0, LCD_CS, LCD_DC, LCD_RST);

// -------------------------------------------------------
// Affiche la vitesse en grand sur l'écran LCD
// Mise en page : nombre centré (police 32px) + "km/h" en petit
// -------------------------------------------------------
void displaySpeed(double kmh) {
  char numStr[8];
  if (kmh < 100.0) {
    dtostrf(kmh, 4, 1, numStr);  // ex: " 12.3"
  } else {
    dtostrf(kmh, 3, 0, numStr);  // ex: "120"
  }
  // Supprimer les espaces en début de chaîne
  char* num = numStr;
  while (*num == ' ') num++;

  u8g2.clearBuffer();

  // Vitesse en grand (police 32px, chiffres uniquement)
  u8g2.setFont(u8g2_font_logisoso32_tn);
  int numWidth = u8g2.getStrWidth(num);
  // Centrer le nombre, légèrement décalé à gauche pour laisser place à "km/h"
  int numX = (128 - numWidth) / 2 - 12;
  if (numX < 0) numX = 0;
  u8g2.drawStr(numX, 48, num);

  // Unité "km/h" en petit, alignée à droite du nombre
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.drawStr(numX + numWidth + 4, 64, "km/h");

  u8g2.sendBuffer();
}

// Ecran d'attente
void displayIdle() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.drawStr(16, 26, "Radar CDM324");
  u8g2.drawStr(24, 48, "En attente...");
  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Radar Doppler CMD324 (24.125 GHz) ===");
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

  // Initialisation écran LCD
  u8g2.begin();
  u8g2.setContrast(0x28);  // Ajuster si trop clair/sombre (plage 0x00-0xFF)
  displayIdle();
  Serial.println("Ecran LCD initialise");

  delay(2000);
}

void loop() {
  // Acquisition des échantillons
  int minADC = 4095, maxADC = 0;  // Pour détecter saturation
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long startTime = micros();
    
    // Lire le signal depuis ADC ESP32
    int adcValue = analogRead(ADC_PIN);  // 0-4095 (12 bits)
    
    // Suivre min/max pour détecter saturation
    if (adcValue < minADC) minADC = adcValue;
    if (adcValue > maxADC) maxADC = adcValue;
    
    // Centrer le signal autour de 0 (retirer offset DC)
    // Valeur mesurée au repos : ~1930 (légèrement sous 2048 à cause des tolérances R5/R6)
    vReal[i] = adcValue - 1930.0;
    vImag[i] = 0;  // Partie imaginaire à zéro
    
    // Attendre pour respecter la fréquence d'échantillonnage
    while (micros() - startTime < samplingPeriodUs) {
      // Attente active
    }
  }

  // PRÉ-FILTRE : vérifier la plage ADC avant de lancer la FFT
  // Au repos : plage ~40-60 pts | Avec mouvement : plage ~80+ pts
  int adcRange = maxADC - minADC;
  const int MIN_ADC_RANGE = 80;  // Seuil : en dessous = pas de mouvement détecté

  if (DEBUG_MODE) {
    Serial.print("ADC: [");
    Serial.print(minADC);
    Serial.print("-");
    Serial.print(maxADC);
    Serial.print("] range=");
    Serial.print(adcRange);
  }

  if (adcRange < MIN_ADC_RANGE) {
    if (DEBUG_MODE) Serial.println(" | Pas de mouvement");
    else Serial.print(".");
    // Pas de delay ici : cycle rapide pour ne pas rater un objet rapide
    return;  // Sortir immédiatement, pas de FFT
  }

  if (DEBUG_MODE) Serial.print(" | FFT...");

  // Appliquer une fenêtre de Hamming pour réduire les fuites spectrales
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  
  // Calculer la FFT
  FFT.compute(FFTDirection::Forward);
  
  // Calculer les magnitudes
  FFT.complexToMagnitude();
  
  // Trouver le pic en ignorant DC et très basses fréquences (index 0 et 1)
  double maxMagnitude = 0;
  int maxIndex = 2;
  for (int i = 2; i < (SAMPLES / 2); i++) {
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
      maxIndex = i;
    }
  }

  // Calculer la fréquence du pic depuis son index (cohérent avec maxMagnitude)
  double peakFrequency = maxIndex * ((double)SAMPLING_FREQUENCY / SAMPLES);

  // Calculer la vitesse
  double vitesse_ms  = (peakFrequency / DOPPLER_COEFF) * ANGLE_CORRECTION;  // m/s corrigé
  double vitesse_kmh = vitesse_ms * 3.6;                                      // km/h corrigé

  // Vérifier si la fréquence est dans une zone de bruit connue
  bool isNoise = (peakFrequency >= NOISE_FREQ_MIN && peakFrequency <= NOISE_FREQ_MAX);

  // Mode DEBUG : afficher toutes les valeurs pour diagnostic
  if (DEBUG_MODE) {
    Serial.print(" Signal: ");
    Serial.print(maxMagnitude, 0);
    Serial.print(" | Fréq: ");
    Serial.print(peakFrequency, 1);
    Serial.print(" Hz");
    if (isNoise) Serial.print(" [BRUIT]");
    Serial.print(" | Vitesse: ");
    Serial.print(vitesse_kmh, 1);
    Serial.print(" km/h | Conf: ");
    Serial.println(consecutiveDetections);

    // Alerte saturation
    if (minADC < 100 || maxADC > 3995) {
      Serial.println("SATURATION ADC detectee !");
    }
  }

  // Valider la détection : signal fort + vitesse plausible + fréquence non parasite
  if (maxMagnitude > DETECTION_THRESHOLD && vitesse_kmh > MIN_SPEED && !isNoise) {
    consecutiveDetections++;
  } else {
    consecutiveDetections = 0;  // Réinitialiser si conditions non remplies
  }

  // Afficher uniquement après N confirmations consécutives
  if (consecutiveDetections >= REQUIRED_CONFIRMATIONS) {
    Serial.println("========================================");
    Serial.print("Frequence Doppler: ");
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

    // Afficher sur l'écran LCD
    displaySpeed(vitesse_kmh);

  } else if (consecutiveDetections == 0 && !DEBUG_MODE) {
    // Mode silencieux - afficher un point pour montrer que ça tourne
    Serial.print(".");
  }
  // Pas de delay : cycle le plus rapide possible pour objets rapides
}
