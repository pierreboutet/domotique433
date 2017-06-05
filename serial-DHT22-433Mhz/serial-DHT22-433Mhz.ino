/**
 * Domotique with 433mhz transmitter and humidity sensor
 * 
 * this program takes command input via Serial, it accepts 4 command :
 * - light-ON : light on the first light
 * - light-OFF : light OFF the first light
 * - Humidity : give back the humodity and temperature
 * - send : send the given code via radio (433Mhz) ex : 'send:5264691'
 * - listen : listen radio, and give back the next code heard
 * 
 * after each command, it print 'WAITING' to indicate it is waiting for orders
 * 
 */

/** Broche "DATA" du capteur */
const byte BROCHE_CAPTEUR = 3;

/* Code d'erreur de la fonction readDHT11() et readDHT22() */
const byte DHT_SUCCESS = 0;        // Pas d'erreur
const byte DHT_TIMEOUT_ERROR = 1;  // Temps d'attente dépassé
const byte DHT_CHECKSUM_ERROR = 2; // Données reçues erronées

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

/** Fonction setup() */
void setup() {
 
  /* Initialisation du port série */
  Serial.begin(9600);
  Serial.println(F("Domotique ARDUINO 433Mhz interface 0.1"));
  Serial.println("WAITING");
  
  /* Place la broche du capteur en entrée avec pull-up */
  pinMode(BROCHE_CAPTEUR, INPUT_PULLUP);

  // init 433Mhz
  mySwitch.enableReceive(0);
  mySwitch.enableTransmit(10);
  // Optional set protocol (default is 1, will work for most outlets)
   mySwitch.setProtocol(1);
   mySwitch.setPulseLength(160);
}

/** Fonction loop() */
void loop() {
  int incomingByte = 0;
  String command;
  String param;
  unsigned long code;
  if (Serial.available() > 0) {
      // read the incoming byte:
      //incomingByte = Serial.read();
      
      command = Serial.readString();

      // if order has a ":", we split order and param :
      int index = command.indexOf(':');
      param = "";
      if (index != -1) {
        param = command.substring(index+1);
        command = command.substring(0, index);
      }

      // say what you got:
      Serial.print("order : ");
      Serial.println(command);
      Serial.print("param : ");
      Serial.println(param);
      
      if( command == "Humidity" ) {
        printTemp();
      } else if( command == "listen" ) {
        listenRadio();
      } else if( command == "light-ON" ) {
        mySwitch.send(5264691, 24);
      } else  if( command == "light-OFF" ) {
        mySwitch.send(5264700, 24);
      } else if (command=="Send"){
        code = param.toInt();
        
        Serial.println("Sending : " + param);
        mySwitch.send(code, 24);
      } else {
        Serial.println("unknown order");
      }
      Serial.println("WAITING");
  }
}


void listenRadio() {
  unsigned long decimal = 0;
  unsigned long counter = 0;
  while (counter < 10000000 && decimal == 0) {
    decimal = mySwitch.getReceivedValue();
    counter ++;
  }
  Serial.print("counter:");
  Serial.print(counter);
  Serial.println();
  Serial.print("listen:");
  Serial.print(decimal);
  Serial.println();
}


void printTemp() {
  float temperature, humidity;
 
  /* Lecture de la température et de l'humidité, avec gestion des erreurs */
  // N.B. Remplacer readDHT11 par readDHT22 en fonction du capteur utilisé !
  switch (readDHT22(BROCHE_CAPTEUR, &temperature, &humidity)) {
  case DHT_SUCCESS: 
     
    /* Affichage de la température et du taux d'humidité */
    Serial.print(F("Humidite (%): "));
    Serial.println(humidity, 2);
    Serial.print(F("Temperature (^C): "));
    Serial.println(temperature, 2);
    break;
 
  case DHT_TIMEOUT_ERROR: 
    Serial.println(F("Pas de reponse !")); 
    break;
 
  case DHT_CHECKSUM_ERROR: 
    Serial.println(F("Pb de communication !")); 
    break;
  }
  
  /* Pas plus d'une mesure par seconde */
  // N.B. Avec le DHT22 il est possible de réaliser deux mesures par seconde
  delay(5000);
}

/**
 * Lit la température et le taux d'humidité mesuré par un capteur DHT11.
 *
 * @param pin Broche sur laquelle est câblée le capteur.
 * @param temperature Pointeur vers la variable stockant la température.
 * @param humidity Pointeur vers la variable stockant le taux d'humidité.
 * @return DHT_SUCCESS si aucune erreur, DHT_TIMEOUT_ERROR en cas de timeout, ou DHT_CHECKSUM_ERROR en cas d'erreur de checksum.
 */
byte readDHT11(byte pin, float* temperature, float* humidity) {
  
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 18, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS) 
    return ret;
    
  /* Calcul la vraie valeur de la température et de l'humidité */
  *humidity = data[0];
  *temperature = data[2];

  /* Ok */
  return DHT_SUCCESS;
}

/**
 * Lit la température et le taux d'humidité mesuré par un capteur DHT22.
 *
 * @param pin Broche sur laquelle est câblée le capteur.
 * @param temperature Pointeur vers la variable stockant la température.
 * @param humidity Pointeur vers la variable stockant le taux d'humidité.
 * @return DHT_SUCCESS si aucune erreur, DHT_TIMEOUT_ERROR en cas de timeout, ou DHT_CHECKSUM_ERROR en cas d'erreur de checksum.
 */
byte readDHT22(byte pin, float* temperature, float* humidity) {
  
  /* Lit le capteur */
  byte data[5];
  byte ret = readDHTxx(pin, data, 1, 1000);
  
  /* Détecte et retourne les erreurs de communication */
  if (ret != DHT_SUCCESS) 
    return ret;
    
  /* Calcul la vraie valeur de la température et de l'humidité */
  float fh = data[0];
  fh *= 256;
  fh += data[1];
  fh *= 0.1;
  *humidity = fh;
 
  float ft = data[2] & 0x7f;
  ft *= 256;
  ft += data[3];
  ft *= 0.1;
  if (data[2] & 0x80) {
    ft *= -1;
  }
  *temperature = ft;

  /* Ok */
  return DHT_SUCCESS;
}

/**
 * Fonction bas niveau permettant de lire la température et le taux d'humidité (en valeurs brutes) mesuré par un capteur DHTxx.
 */
byte readDHTxx(byte pin, byte* data, unsigned long start_time, unsigned long timeout) {
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  // start_time est en millisecondes
  // timeout est en microsecondes
 
  /* Conversion du numéro de broche Arduino en ports / masque binaire "bas niveau" */
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddr = portModeRegister(port);   // Registre MODE (INPUT / OUTPUT)
  volatile uint8_t *out = portOutputRegister(port); // Registre OUT (écriture)
  volatile uint8_t *in = portInputRegister(port);   // Registre IN (lecture)
  
  /* Conversion du temps de timeout en nombre de cycles processeur */
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  /* Evite les problèmes de pull-up */
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   // Laisse le temps à la résistance de pullup de mettre la ligne de données à HIGH
 
  /* Réveil du capteur */
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); // Temps d'attente à LOW causant le réveil du capteur
  // N.B. Il est impossible d'utilise delayMicroseconds() ici car un délai
  // de plus de 16 millisecondes ne donne pas un timing assez précis.
  
  /* Portion de code critique - pas d'interruptions possibles */
  noInterrupts();
  
  /* Passage en écoute */
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT
 
  /* Attente de la réponse du capteur */
  timeout = 0;
  while(!(*in & bit)) { /* Attente d'un état LOW */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*in & bit) { /* Attente d'un état HIGH */
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  /* Lecture des données du capteur (40 bits) */
  for (byte i = 0; i < 40; ++i) {
 
    /* Attente d'un état LOW */
    unsigned long cycles_low = 0;
    while(!(*in & bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    /* Attente d'un état HIGH */
    unsigned long cycles_high = 0;
    while(*in & bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    /* Si le temps haut est supérieur au temps bas c'est un "1", sinon c'est un "0" */
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      data[i / 8] |= 1;
    }
  }
  
  /* Fin de la portion de code critique */
  interrupts();
 
  /*
   * Format des données :
   * [1, 0] = humidité en %
   * [3, 2] = température en degrés Celsius
   * [4] = checksum (humidité + température)
   */
   
  /* Vérifie la checksum */
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
    return DHT_CHECKSUM_ERROR; /* Erreur de checksum */
  else
    return DHT_SUCCESS; /* Pas d'erreur */
}
