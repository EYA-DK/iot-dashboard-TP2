// ============================================================
//  Système de barrière de stationnement intelligent – ESP32
//  Auteur  : Eya DKHIL
//  Date    : 2026-06-19
//  Description : Contrôle d'une barrière via servo, mesure de
//                distance (HC-SR04), température/humidité (DHT22),
//                luminosité, affichage LCD I2C et envoi des données
//                vers ThingSpeak via WiFi.
// ============================================================

// ------------------------------------------------------------
//  BIBLIOTHÈQUES NÉCESSAIRES
// ------------------------------------------------------------
#include <ESP32Servo.h>          // Contrôle du servo moteur sur ESP32
#include <DHT.h>                 // Capteur de température et humidité DHT22
#include <Wire.h>                // Communication I2C (LCD)
#include <LiquidCrystal_I2C.h>  // Afficheur LCD 16x2 via I2C
#include <WiFi.h>                // Connexion au réseau WiFi
#include <HTTPClient.h>          // Requêtes HTTP vers ThingSpeak

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – BROCHES (PIN MAPPING)
// ------------------------------------------------------------
#define PIN_SERVO       13   // Signal du servo moteur
#define PIN_POTENTIOMETRE 35 // Entrée ADC du potentiomètre (0–4095)
#define PIN_DHT22       15   // Données du capteur DHT22
#define PIN_TRIG        5    // Déclencheur HC-SR04 (TRIG)
#define PIN_ECHO        18   // Récepteur HC-SR04 (ECHO)
#define PIN_LED         4    // Anode de la LED (avec résistance)
#define PIN_BUZZER      2    // Signal du buzzer
#define PIN_LUMIERE     34   // Sortie analogique du capteur de lumière (0–4095)

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – LCD I2C
// ------------------------------------------------------------
#define LCD_ADRESSE     0x27  // Adresse I2C de l'afficheur LCD
#define LCD_COLONNES    16    // Nombre de colonnes du LCD
#define LCD_LIGNES      2     // Nombre de lignes du LCD

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – DHT22
// ------------------------------------------------------------
#define TYPE_DHT        DHT22  // Type de capteur DHT utilisé

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – ALARME
// ------------------------------------------------------------
#define DISTANCE_ALARME_CM  20    // Seuil de distance pour déclencher l'alarme (cm)
#define FREQ_BUZZER_HZ      1000  // Fréquence du buzzer en Hz (1 kHz)
// #define CANAL_BUZZER        0     // Supprimé : concept de canal inexistant en Core v3.x

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – TEMPORISATION
// ------------------------------------------------------------
#define INTERVALLE_DHT_MS        2000   // Lecture DHT toutes les 2 secondes
#define INTERVALLE_LCD_MS        3000   // Cycle d'écran LCD toutes les 3 secondes
#define INTERVALLE_THINGSPEAK_MS 15000  // Envoi ThingSpeak toutes les 15 secondes

// ------------------------------------------------------------
//  CONSTANTES DE CONFIGURATION – WIFI ET THINGSPEAK
// ------------------------------------------------------------
#define WIFI_SSID              "Wokwi-GUEST"      
#define WIFI_PASSWORD          ""   
#define THINGSPEAK_WRITE_KEY   "IKWU68H2TSP0T4G6"
#define THINGSPEAK_READ_KEY   "QKNR8IW49X5WO6YX"
#define THINGSPEAK_CHANNEL_ID  "3412525"
#define THINGSPEAK_URL  "https://api.thingspeak.com/update"
// ------------------------------------------------------------
//  DÉCLARATIONS DES OBJETS GLOBAUX
// ------------------------------------------------------------

// Servo moteur
Servo servoBarriere;

// Capteur DHT22
DHT dht(PIN_DHT22, TYPE_DHT);

// Afficheur LCD I2C (16 colonnes, 2 lignes)
LiquidCrystal_I2C lcd(LCD_ADRESSE, LCD_COLONNES, LCD_LIGNES);

// ------------------------------------------------------------
//  VARIABLES GLOBALES – DONNÉES CAPTEURS
// ------------------------------------------------------------
float     temperature       = 0.0;  // Température en °C
float     humidite          = 0.0;  // Humidité en %
float     distanceCm        = 0.0;  // Distance mesurée en cm
int       luminositePct     = 0;    // Luminosité en pourcentage (0–100%)
int       angleServo        = 0;    // Angle actuel du servo (0–180°)

// ------------------------------------------------------------
//  VARIABLES GLOBALES – GESTION DU TEMPS
// ------------------------------------------------------------
unsigned long derniereLectureDHT        = 0;  // Horodatage dernière lecture DHT
unsigned long dernierCycleLCD           = 0;  // Horodatage dernier changement d'écran LCD
unsigned long dernierEnvoiThingSpeak    = 0;  // Horodatage dernier envoi ThingSpeak
int           ecranLCDActuel            = 0;  // Indice de l'écran LCD affiché (0 ou 1)

// ============================================================
//  FONCTIONS AUXILIAIRES
// ============================================================

/**
 * measureDistance()
 * -----------------
 * Mesure la distance via le capteur ultrasonique HC-SR04.
 * Envoie une impulsion TRIG de 10 µs, mesure la durée du
 * signal ECHO et calcule la distance en centimètres.
 *
 * Retourne : distance en cm (float), ou -1.0 en cas de timeout.
 */
float measureDistance() {
  // Assurer que TRIG est à l'état bas avant la mesure
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  // Envoyer une impulsion haute de 10 µs sur TRIG
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Mesurer la durée du signal ECHO (timeout = 30 ms → ~510 cm max)
  long dureeEcho = pulseIn(PIN_ECHO, HIGH, 30000UL);

  // Si timeout, retourner -1
  if (dureeEcho == 0) {
    return -1.0;
  }

  // Calcul de la distance : vitesse du son = 343 m/s = 0.0343 cm/µs
  // Distance = (durée × 0.0343) / 2  (aller-retour)
  float distance = (dureeEcho * 0.0343) / 2.0;
  return distance;
}

/**
 * readLightPercent()
 * ------------------
 * Lit la valeur ADC 12 bits du capteur de lumière (GPIO34)
 * et la convertit en pourcentage (0–100%).
 *
 * Retourne : luminosité en pourcentage (int).
 */
int readLightPercent() {
  int valeurADC = analogRead(PIN_LUMIERE);
  // Conversion de la plage ADC 12 bits (0–4095) vers 0–100%
  int pourcentage = map(valeurADC, 0, 4095, 0, 100);
  return pourcentage;
}

/**
 * uploadToThingSpeak()
 * --------------------
 * Envoie les données des capteurs vers le canal ThingSpeak
 * via une requête HTTP GET.
 *   - Field1 : température (°C)
 *   - Field2 : distance (cm)
 *   - Field3 : luminosité (%)
 *   - Field4 : angle du servo (°)
 */
void uploadToThingSpeak() {
  // Vérifier que le WiFi est connecté avant d'envoyer
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ThingSpeak] WiFi non connecté – envoi annulé.");
    return;
  }

  HTTPClient http;

  // Construction de l'URL avec les paramètres des champs
  String url = String(THINGSPEAK_URL)
    + "?api_key=" + String(THINGSPEAK_WRITE_KEY)
    + "&field1=" + String(temperature, 1)
    + "&field2=" + String(distanceCm, 1)
    + "&field3=" + String(luminositePct)
    + "&field4=" + String(angleServo);

  Serial.print("[ThingSpeak] Envoi vers : ");
  Serial.println(url);

  // Initialiser la connexion HTTP
  http.begin(url);

  // Envoyer la requête GET
  int codeHTTP = http.GET();

  if (codeHTTP > 0) {
    Serial.print("[ThingSpeak] Réponse HTTP : ");
    Serial.println(codeHTTP);
  } else {
    Serial.print("[ThingSpeak] Erreur HTTP : ");
    Serial.println(http.errorToString(codeHTTP));
  }

  // Libérer les ressources HTTP
  http.end();
}

// ============================================================
//  SETUP – INITIALISATION
// ============================================================
void setup() {
  // ----------------------------------------------------------
  //  Initialisation du port série (débogage)
  // ----------------------------------------------------------
  Serial.begin(115200);
  Serial.println("=== Barrière de stationnement ESP32 – Démarrage ===");

  // ----------------------------------------------------------
  //  Configuration des broches d'entrée/sortie
  // ----------------------------------------------------------
  pinMode(PIN_TRIG,    OUTPUT);  // HC-SR04 TRIG en sortie
  pinMode(PIN_ECHO,    INPUT);   // HC-SR04 ECHO en entrée
  pinMode(PIN_LED,     OUTPUT);  // LED en sortie
  pinMode(PIN_BUZZER,  OUTPUT);  // Buzzer en sortie

  // Initialiser les sorties à l'état bas
  digitalWrite(PIN_TRIG,   LOW);
  digitalWrite(PIN_LED,    LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // ----------------------------------------------------------
  //  Configuration du canal PWM LEDC pour le buzzer (ESP32)
  // ----------------------------------------------------------
  ledcAttach(PIN_BUZZER, FREQ_BUZZER_HZ, 8);   // Core v3.x : attache et configure en une seule appel

  // ----------------------------------------------------------
  //  Initialisation du servo moteur
  // ----------------------------------------------------------
  servoBarriere.attach(PIN_SERVO);  // Attacher le servo à GPIO 13
  servoBarriere.write(90);          // Position initiale : barrière à mi-course (90°)
  Serial.println("[Servo] Initialisé à 90°");

  // ----------------------------------------------------------
  //  Initialisation du capteur DHT22
  // ----------------------------------------------------------
  dht.begin();
  Serial.println("[DHT22] Capteur initialisé");

  // ----------------------------------------------------------
  //  Initialisation de l'afficheur LCD I2C
  // ----------------------------------------------------------
  Wire.begin();          // Démarrer le bus I2C (SDA=GPIO21, SCL=GPIO22 par défaut ESP32)
  lcd.init();            // Initialiser le LCD
  lcd.backlight();       // Activer le rétroéclairage
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Barriere Smart");
  lcd.setCursor(0, 1);
  lcd.print("Initialisation..");
  Serial.println("[LCD] Afficheur initialisé");
  delay(2000);           // Afficher le message d'accueil 2 secondes
  lcd.clear();

  // ----------------------------------------------------------
  //  Connexion au réseau WiFi
  // ----------------------------------------------------------
  Serial.print("[WiFi] Connexion à : ");
  Serial.println(WIFI_SSID);

  lcd.setCursor(0, 0);
  lcd.print("Connexion WiFi..");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tentatives = 0;
  while (WiFi.status() != WL_CONNECTED && tentatives < 20) {
    delay(500);
    Serial.print(".");
    tentatives++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connecté !");
    Serial.print("[WiFi] Adresse IP : ");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi OK!");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] Échec de connexion – mode hors ligne.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi: ECHEC");
    lcd.setCursor(0, 1);
    lcd.print("Mode hors ligne");
  }

  delay(2000);
  lcd.clear();

  // ----------------------------------------------------------
  //  Initialisation des horodatages
  // ----------------------------------------------------------
  derniereLectureDHT     = millis();
  dernierCycleLCD        = millis();
  dernierEnvoiThingSpeak = millis();

  Serial.println("[Setup] Initialisation terminée – démarrage de la boucle principale.");
}

// ============================================================
//  LOOP – BOUCLE PRINCIPALE
// ============================================================
void loop() {
  unsigned long maintenant = millis();  // Horodatage courant

  // ----------------------------------------------------------
  //  1. LECTURE DU POTENTIOMÈTRE → ANGLE DU SERVO
  //     GPIO35 : ADC 12 bits (0–4095) → angle 0–180°
  // ----------------------------------------------------------
  int valeurPot = analogRead(PIN_POTENTIOMETRE);
  angleServo = map(valeurPot, 0, 4095, 0, 180);

  // ----------------------------------------------------------
  //  2. LECTURE DHT22 – Température & Humidité (toutes les 2s)
  // ----------------------------------------------------------
  if (maintenant - derniereLectureDHT >= INTERVALLE_DHT_MS) {
    derniereLectureDHT = maintenant;

    float tempLue = dht.readTemperature();   // Température en °C
    float humiLue = dht.readHumidity();      // Humidité en %

    // Vérifier la validité des lectures
    if (!isnan(tempLue) && !isnan(humiLue)) {
      temperature = tempLue;
      humidite    = humiLue;
      Serial.print("[DHT22] Temp: ");
      Serial.print(temperature, 1);
      Serial.print(" °C | Humidité: ");
      Serial.print(humidite, 1);
      Serial.println(" %");
    } else {
      Serial.println("[DHT22] Erreur de lecture – données ignorées.");
    }
  }

  // ----------------------------------------------------------
  //  3. MESURE DE DISTANCE – HC-SR04
  // ----------------------------------------------------------
  float distLue = measureDistance();
  if (distLue > 0) {
    distanceCm = distLue;
  }
  Serial.print("[HC-SR04] Distance: ");
  Serial.print(distanceCm, 1);
  Serial.println(" cm");

  // ----------------------------------------------------------
  //  4. LECTURE DE LA LUMINOSITÉ – Capteur analogique GPIO34
  // ----------------------------------------------------------
  luminositePct = readLightPercent();
  Serial.print("[Lumière] Luminosité: ");
  Serial.print(luminositePct);
  Serial.println(" %");

  // ----------------------------------------------------------
  //  5. LOGIQUE D'ALARME
  //     Si distance < 20 cm :
  //       - LED allumée
  //       - Buzzer actif (1 kHz via PWM)
  //       - Barrière fermée (servo à 0°)
  //     Sinon :
  //       - LED éteinte
  //       - Buzzer silencieux
  //       - Servo positionné selon le potentiomètre
  // ----------------------------------------------------------
  if (distanceCm > 0 && distanceCm < DISTANCE_ALARME_CM) {
    // --- ALARME ACTIVE ---
    Serial.println("[ALARME] Obstacle détecté ! Barrière fermée.");

    // Allumer la LED
    digitalWrite(PIN_LED, HIGH);

    // Activer le buzzer à 1 kHz
    ledcWriteTone(PIN_BUZZER, FREQ_BUZZER_HZ);

    // Fermer la barrière (servo à 0°)
    servoBarriere.write(0);
    angleServo = 0;  // Mettre à jour la variable pour l'affichage et ThingSpeak

  } else {
    // --- FONCTIONNEMENT NORMAL ---
    // Éteindre la LED
    digitalWrite(PIN_LED, LOW);

    // Désactiver le buzzer
    ledcWriteTone(PIN_BUZZER, 0);

    // Positionner le servo selon le potentiomètre
    servoBarriere.write(angleServo);
  }

  // ----------------------------------------------------------
  //  6. AFFICHAGE LCD – Cycle entre 2 écrans toutes les 3s
  //     Écran 0 : Distance + Température
  //     Écran 1 : Luminosité + Angle servo
  // ----------------------------------------------------------
  if (maintenant - dernierCycleLCD >= INTERVALLE_LCD_MS) {
    dernierCycleLCD = maintenant;
    lcd.clear();

    if (ecranLCDActuel == 0) {
      // --- Écran 1 : Distance et Température ---
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      if (distanceCm > 0) {
        lcd.print((int)distanceCm);
        lcd.print("cm");
      } else {
        lcd.print("---cm");
      }

      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(temperature, 1);
      lcd.print(" C");

      ecranLCDActuel = 1;  // Passer à l'écran suivant

    } else {
      // --- Écran 2 : Luminosité et Angle servo ---
      lcd.setCursor(0, 0);
      lcd.print("Light: ");
      lcd.print(luminositePct);
      lcd.print("%");

      lcd.setCursor(0, 1);
      lcd.print("Servo: ");
      lcd.print(angleServo);
      lcd.print(" deg");

      ecranLCDActuel = 0;  // Revenir à l'écran 0
    }
  }

  // ----------------------------------------------------------
  //  7. ENVOI VERS THINGSPEAK – Toutes les 15 secondes
  //     Field1 = température | Field2 = distance
  //     Field3 = luminosité  | Field4 = angle servo
  // ----------------------------------------------------------
  if (maintenant - dernierEnvoiThingSpeak >= INTERVALLE_THINGSPEAK_MS) {
    dernierEnvoiThingSpeak = maintenant;
    Serial.println("[ThingSpeak] Déclenchement de l'envoi des données...");
    uploadToThingSpeak();
  }

  // ----------------------------------------------------------
  //  Courte pause pour stabiliser les lectures ADC
  // ----------------------------------------------------------
  delay(100);
}
// ============================================================
//  FIN DU PROGRAMME
// ============================================================
