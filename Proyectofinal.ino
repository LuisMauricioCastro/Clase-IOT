
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x23,16,2);  //  << Address 1
LiquidCrystal_I2C lcd2(0x25, 16, 2); // << Address 2
LiquidCrystal_I2C lcd3(0x26, 16, 2); // << Address 3

// Definimos el pin digital donde se conecta el sensor
#define DHTPIN 14
// Dependiendo del tipo de sensor
#define DHTTYPE DHT11
const int buzzer = 25; 
// Inicializamos el sensor DHT11
DHT dht(DHTPIN, DHTTYPE);

int Gas_analog = 4;    // used for ESP32
void setup() {
  pinMode(33,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(buzzer,OUTPUT);
  Serial.begin(115200);
   dht.begin();
  // Inicializar el LCD
  lcd.init();
  lcd2.init();
  lcd3.init();
  //Encender la luz de fondo.
  lcd.backlight();
  lcd2.backlight();
  lcd3.backlight();
  // Escribimos el Mensaje en el LCD.
  
  lcd2.print("Temperatura");
  lcd3.print("Humedad");
  
}
void loop() {
  // Leemos la humedad relativa
  float h = dht.readHumidity();
  // Leemos la temperatura en grados centígrados (por defecto)
  float t = dht.readTemperature();
  // Leemos la temperatura en grados Fahreheit
  float f = dht.readTemperature(true);
 //inizializamos el senosr de gas de manera analogica
  int gassensorAnalog = analogRead(Gas_analog);
//Establecesmos el cursor en las pantallas
  lcd2.setCursor(0, 1);
  lcd3.setCursor(0, 1);
   // Escribimos el número de temperatura,humedad y gas metano
  lcd2.print(t);
  lcd2.print(" Grados C");
  lcd3.print(h);
  lcd3.print(" Relativa");
  lcd.setCursor(0, 1);
  lcd.print(gassensorAnalog);
  lcd.print("ppm ");
 //En caso de que las ppm superen los 1400 unidades mandara la leyenda que hay gas y hará sonar la alarma
    if (gassensorAnalog > 1600) {
      lcd.setCursor(0, 0);
    lcd.println("Hay Gas   ");
    digitalWrite(33,HIGH);
    digitalWrite(32,LOW);
    digitalWrite(buzzer,HIGH);
  else {
    lcd.setCursor(0, 0);
    lcd.println("No hay Gas");
    digitalWrite(32,HIGH);
    digitalWrite(33,LOW);
    digitalWrite(buzzer,LOW);
  }
  delay(100);
  
}
