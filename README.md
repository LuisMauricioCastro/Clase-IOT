# Clase-IOT
Programas realizados en la clase de Internet of things 

Grupo 8HW1

Alumno: Luis Mauricio Castro Gutiérrez

Profesor:  Pacheco Gonzalez Alberto

Placa de desarrollo: ESP32 DEVKIT V1

### Plantamiento del problema
El proyecto busca lograr determinar si la ubicación en donde está el sistema tiene niveles de metano no peligrosos, también sensado de la temperatura y humedad relativa del ambiente, esto por la gran cantidad de fugas de gas que existen cotidianamente.
### Objetivos
Se busca hacer con un microcontrolador ESP32 el cual por medio de componentes mostrará los datos de los sensores y alertará si el nivel de metano es suficiente como para lograr ser peligroso, en caso de detectarlo activara una alarma visual y sonora para que el usuario sepa que existe una fuga de gas.
### Metodologia
Nuestra metodologia a usar es la metodologia Scrumban esta metodología permite que se agreguen tareas individuales al plan. Esto permite que los planes de proyectos mantengan una estructura simple y clara, esta observa al proyecto como un rompecabezas el cual requiere de todas las piezas para estar completo, pero no obliga a acomodar las piezas en un orden especifico. 
Basados en esta metodologia decidimos dividirnos tareas como el investigar y probar sensores que íbamos a usar, investigar que librerías y que código tendríamos que incluir para que las pantallas con el adaptador funcionaran, integración del driver ULN2003, implementación de alertas sonora y visual, compra de componentes y uso de pines del ESP32.


Primeramente, se buscó que sensores teníamos a la mano y si estos servían para nuestro proyecto, Se determino que teníamos el sensor de temperatura y humedad, pero no el de metano por lo que se tuvo que comprar. Se investigo en cada componente el uso de los pines del ESP32 dado que algunos pines no son usables por cuestiones de que se usan internamente por el ESP32. Después de lo anterior se decidió hacer las pruebas con los sensores de metano, humedad y temperatura, para esto se tuvo que investigar librerías junto con su funcionamiento en código. Una vez realizadas las pruebas se implementaron las alertas sonoras, al ver que no alertaban del todo se decidió implementar el driver ULN2003 para poder usar alarmas sonoras y visuales mas potentes.

### Descripción de materiales

●	ESP-32.

●	Driver ULN2003.

●	Cables Dupont.

●	Bocina automotriz DC 12V.

●	Fuente de poder 12V.

●	Tiras led 12 volts.

●	2 Protoboards.

●	DHT11.

3 Pantallas LCD.

●	3 adaptadores de conexiones LCD a i2c.

●	1 Sensor MQ4.

Sensor DHT11

![image](https://user-images.githubusercontent.com/98352322/168677494-364a8489-9f6b-4ccb-8c94-5bb5f46216d6.png)

Este sensor es muy usado gracias a su alta fiabilidad y su estabilidad, esto gracias a que su señal digital ya viene previamente calibrada. Sumado a esto su instalación y uso es muy fácil pues solo cuenta con 3 conexiones (Voltaje 3V/5V, Tierra y I/O), por software se tiene una librería para la interacción con el sensor. 

Especificaciones técnicas detalladas:

![image](https://user-images.githubusercontent.com/98352322/168677568-e3c6d7de-57b7-46b3-95b8-5b47476271c4.png)

Sensor MQ-4

![image](https://user-images.githubusercontent.com/98352322/168677600-109af7c4-6af9-460b-9138-655a5d1ef029.png)

Este sensor está encargado de detectar el gas metano en el ambiente, tiene 2 salidas de información (analógica y digital), su instalación es rápida y sencilla dado que cuenta con 4 terminales(Voltaje, Tierra, Salida digital, Salida analógica) esto dándonos la capacidad de con un solo pin enviar salida de datos a él microcontrolador.

Especificaciones técnicas detalladas:

![image](https://user-images.githubusercontent.com/98352322/168677680-3c62b662-23eb-4fa2-b8e4-3ce35534b8bb.png)

Pantalla LCD (16,2) 

![image](https://user-images.githubusercontent.com/98352322/168677708-3735367c-ec8e-4111-bbf1-b8eac7ed399c.png)

Pantalla de 16 caracteres y 2 renglones, serán las encargadas de mostrar los datos de los sensores al usuario.

Adaptador display LCD (16,2) a i2c

![image](https://user-images.githubusercontent.com/98352322/168677760-9f0e7db9-858b-48b8-8502-41907d072e0b.png)

Este adaptador permite transformar un display (16,2) común y corriente a un protocolo de comunicación i2c, este protocolo es muy beneficioso pues nos permite controlar múltiples pantallas con solo 2 pines, este adaptador cuenta con un cambio de dirección si se le es soldado ciertas terminales, esto para manejar múltiples pantallas.

Especificaciones técnicas detalladas:

Compatible con tarjetas Arduino  o otros microcontroladores que tengan bus I2C 

Tipo de display: Blanco negativo en fondo azul.

Direcciónes I2C:0x38-0x3F (0x3F por default)

Suministro de energía: 5V

Interfaz: I2C a 4 bits LCD datos y líneas de control.

Ajuste de contraste: Potenciómetro en placa.

Control de luz trasera: Firmware o un cable jumper.

Tamaño: 80x36 mm

Protoboard

![image](https://user-images.githubusercontent.com/98352322/168677889-757bef72-502c-4bc9-8da0-5f526703660c.png)

Protoboard Breadboard 830 Puntos Blanca
Tablilla de conexión (Protoboard), con 1 bloque, 2 tiras, sin bornes y 830 perforaciones, ideales para armar prototipos de circuitos electrónicos.

Tiras led 12 volts

![image](https://user-images.githubusercontent.com/98352322/168677949-b06dc0e7-1458-46e9-a854-3e7071805c72.png)

La función de las tiras led es mostrar de manera visual el comportamiento del sistema.

Fuente de poder 12V/Fuente de computadora modificada para entregar 12V

![image](https://user-images.githubusercontent.com/98352322/168678050-13ba8186-5531-4f0e-a007-a94afa580f8d.png)

Sera la encargada de suministrar energía a los componentes que funcionan con 12V, se pueden usar baterías o soluciones más compactas, en nuestro caso usaremos una fuente de poder modificada para tener a disposición una terminal de 12 voltios y de tierra.

Driver ULN2003

![image](https://user-images.githubusercontent.com/98352322/168678099-ccb93bf9-6482-458d-89f9-aa3952301fc2.png)

Driver usado para mandar una señal de 3.3/5V a las entradas, en caso de que la mande el output hará tierra, como se puede ver en el siguiente diagrama:

![image](https://user-images.githubusercontent.com/98352322/168678136-e03dfe29-55fe-4982-9e81-f45c18edcfc8.png)

Sirena automotriz DC 12V 6 tonos

![image](https://user-images.githubusercontent.com/98352322/168678173-32202eb6-ad6b-4534-a7b9-c15e0fd088ba.png)

Sirena de auto que funciona con 12V y tiene 6 tonos, se usará para medio de alerta sonoro frente a la presencia de metano.

Cables dupont

![image](https://user-images.githubusercontent.com/98352322/168678209-c0b827cc-4ecf-4919-b291-15c7eb475aa1.png)

Se realizará la conexión entre los componentes del sistema para mandar los datos, voltajes y tierras.

ESP32

![image](https://user-images.githubusercontent.com/98352322/168678326-e811f4b5-57cb-46a1-b65d-9c252c50151f.png)

Voltaje de Alimentación (USB): 5V DC

Voltaje de Entradas/Salidas: 3.3V DC

SoC: ESP32

CPU principal: Tensilica Xtensa 32-bit LX6

Frecuencia de Reloj: hasta 240Mhz

Desempeñó: Hasta 600 DMIPS

Procesador secundario: Permite hacer operaciones básica en modo de ultra bajo consumo

Wifi: 802.11 b/g/n/e/i (802.11n @ 2.4 GHz hasta 150 Mbit/s)

Bluetooth:v4.2 BR/EDR and Bluetooth Low Energy (BLE)

Xtensa® Dual-Core 32-bit LX6 microprocessors, up to 600 DMIPS

Memoria: 448 KByte ROM

520 KByte SRAM

16 KByte SRAM in RTC

QSPI Flash/SRAM, 4 MBytes

Pines Digitales GPIO: 24 (Algunos pines solo como entrada)

Conversor Analogico Digital: Dos ADC de 12bits tipo SAR, soporta mediciones en hasta 18 canales, algunos pines soporta un amplificador con ganancia programable

UART: 2

Chip USB-Serial: CP2102

Antena en PCB

Seguridad: Estandares IEEE 802.11 incluyendo WFA, WPA/WPA2 and WAPI

1024-bit OTP, up to 768-bit for customers

Aceleración criptográfica por hardware: AES, HASH (SHA-2), RSA, ECC, RNG

### Descripción del diseño

Primeramente, se implementó paso a paso todas las conexiones en fritzing, si bien algunas conexiones pueden llegar a confundir es lo mas acomodado que puede estar el diagrama, explicaremos parte por parte el diseño

![image](https://user-images.githubusercontent.com/98352322/168678467-5acfd584-5fa2-4482-ac20-6b6f258b7ba4.png)

Pantallas LCD:

![image](https://user-images.githubusercontent.com/98352322/168678490-ffbd4405-25aa-44aa-aa27-64a100030c40.png)

En esta parte podemos ver que todas las pantallas cuentan con el protocolo i1c dado que son de 4 pines y en la descripción nos aclara, por lo que conectaremos voltaje y tierra a todas, además conectaremos a los pines del ESP32 i2c SCL y i2c SDA a las terminales positiva y negativa de la parte superior, por estas lineas se enviara la información a las pantallas por ende tenemos que conectar sus pines SCL y SDA en donde corresponden en la linea.

Sensores:

![image](https://user-images.githubusercontent.com/98352322/168678533-7d5db326-c34a-4715-a81a-7a1933229395.png)

Los sensores estarán conectados a voltaje y tierra, sumados a estos el pin de datos del sensor DHT11 se conectará a el pin 14 del ESP32, el sensor MQ-4 conectara su terminal del sensor analógico a el pin 4.

ULN2003:

![image](https://user-images.githubusercontent.com/98352322/168678576-3dfd83c7-268c-4189-a74a-1e1b817eca7e.png)

El ULN2003 va a recibir tres entradas de los pines 25,33 y 32, estas entradas pasaran por el driver y conectaran a la tierra del led verde(el primero), la tierra del led rojo(el segundo) y de nuestra alarma, esto completando el circuito dado que una fuente de poder de 12 volts esta enviando tierra al ULN2003 y positivo a los componentes si el ULN2003 envía tierra a cualquiera de los componentes el circuito se cerrara y el componente funcionara.


Pd: se puso la interpretación de la tira led de 12v solo con un led y la interpretación de la sirena de 12v con un buzzer, se puso una batería de 12v en sustituto a nuestra fuente de poder de computadora modificada.

### Desarrollo del proyecto

Primeramente, escogimos el tema dado a que por situaciones personales se podría llegar a implementar fuera de la clase.
Una vez decidido el proyecto se empezó a conseguir los materiales, inicialmente se estuvo probando cada sensor por separado para entender su funcionamiento.


Al inicio se implementó únicamente el sensor DHT y se hizo pruebas con el siguiente código:		
```
// Incluimos librería
#include <DHT.h>
// Definimos el pin digital donde se conecta el sensor
#define DHTPIN 14
// Dependiendo del tipo de sensor
#define DHTTYPE DHT11
// Inicializamos el sensor DHT11
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  // Inicializamos comunicación serie
  Serial.begin(9600);
  // Comenzamos el sensor DHT
  dht.begin();
}
void loop() {
    // Esperamos 5 segundos entre medidas
  delay(5000);
  // Leemos la humedad relativa
  float h = dht.readHumidity();
  // Leemos la temperatura en grados centígrados (por defecto)
  float t = dht.readTemperature();
  // Leemos la temperatura en grados Fahreheit
  float f = dht.readTemperature(true);
  // Comprobamos si ha habido algún error en la lectura
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    return;
  }
  // Calcular el índice de calor en Fahreheit
  float hif = dht.computeHeatIndex(f, h);
  // Calcular el índice de calor en grados centígrados
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Índice de calor: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
}
```

Al saber que funcionaba correctamente decidimos ver si el sensor mq4 funcionaba con el siguiente codigo:
```
const int MQ_PIN = A0;

const int MQ_DELAY = 2000;

void setup(){

  Serial.begin(9600);
  
}

void loop() {

  int raw_adc = analogRead(MQ_PIN);
  
  float value_adc = raw_adc * (5.0 / 1023.0);
  
  Serial.print("Raw:");
  
  Serial.print(raw_adc);
  
  Serial.print("    Tension:");
  
  Serial.println(value_adc);
  
 delay(MQ_DELAY);
 
}
```


Nos llevamos una mala sorpresa, pues el sensor que tenía no funcionaba, por lo que se tuvo que comprar otro y ya que funcionaba correctamente se decidió implementar la muestra de datos.

Al buscar como implementar pantallas con protocolo I2C se buscó primeramente las pantallas OLED, se buscó el siguiente modelo:

![image](https://user-images.githubusercontent.com/98352322/168678701-88951810-06a3-496f-8fb8-b5b531de3e66.png)

Pero lamentablemente al comprar se tuvieron que regresar dado que no se les podía cambiar la dirección, por lo que se decidió pasar a usar las pantallas LCD (16,2), para esto se requiere el adaptador del display a el protocolo de comunicación i2c.

Para esto requerimos soldar las pantallas con el adaptador, a su vez tendremos que soldar las direcciones de los adaptadores para con el código que se verá a su continuación saber las direcciones de cada pantalla.

![image](https://user-images.githubusercontent.com/98352322/168678947-5e122852-48da-41c6-8fcc-2794152aedcc.png)

En la anterior imagen podemos ver en donde se cambia lña dirección de cada adaptador.


El código para buscar las direcciones es el siguiente:
```

#include <Wire.h> //include Wire.h library

void setup()
{
  Wire.begin(); // Wire communication begin
  Serial.begin(9600); // The baudrate of Serial monitor is set in 9600
  while (!Serial); // Waiting for Serial Monitor
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}
```

Al descifrar las direcciones integraremos las pantallas y sensores, una vez integrado esto pasaremos a la integración de una alerta sonora y visual, esto primeramente se buscó  usar un buzzer y leds sencillos, pero se notó que no generaban suficiente alerta, por lo que se buscó el usar una sirena más potente, esto buscando algo que lograra convertir una señal de 5 volts a 12 volts se barajaron opciones como el uso de relevadores pero se deicidio mejor el usar un driver ULN2003 esto que cumple la función de un relevador sin ser uno, con esto se decidió aprovechar el driver para que también activara 2 tiras leds de 2 colores diferentes, ambas de 12volts, alimentado por una fuente de computadora, ya que esta nos brinda una alimentación para los componentes de 12 Volts.

![image](https://user-images.githubusercontent.com/98352322/168679315-b6051fb6-0c58-4b57-a2a0-cc85a20bda4f.png)

Por lo que todo implementado en código se ve de la siguiente manera:

```
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
```
Se contaba con un buzzer pero este no se percibía bien, por lo que optamos por sustituirlo por una alarma.

Logramos conseguir una alarma de carro a muy buen precio y se conecto a la alimentación de 12 Volts. Gracias al sonido de la alarma pudimos obtener el resultado deseado ya que ahora podíamos escuchar de manera clara cuando el sistema detectaba una anomalía, esto para poder alertar a las personas de este problema.

![image](https://user-images.githubusercontent.com/98352322/168680242-7047baeb-7ac3-4e16-a648-c812f78a9151.png)

Se puede observar en la imagen como se aplicaron las tiras led de 12 Volts, ya que estas brindaran el estado del ambiente de manera lumínica. Demostrando que el color verde es el estado natural correcto y el rojo es cuando ha detectado una anomalía en el ambiente, uno de los principales casos cuando se activa el color rojo en la tira led es cuando ha detectado monóxido de carbono, ya que esta es la principal sustancia que se avecina en los hogares. 

![image](https://user-images.githubusercontent.com/98352322/168680291-85059865-9a12-4684-9c54-341e1bd67d1e.png)

### Planeación
Gracias a que el proyecto fue desarrollado en tiempo y forma nos gustaría implementar una caja donde esté esté guardado, ya que mejora la estética, facilidad de transportar, y sobre todo se vuelve más amigable su uso.

También una posible mejora seria implementar el empaquetado de nuestro sistema mediante una impresora 3D, ya que de esta manera podemos realizar el diseño, a nuestro gusto, buscando la mejor apariencia y el mejor espacio para el acomodo de nuestro sistema.

Tambien una posible mejora implementar un servidor el cual pueda observar el valor de los sensores o incluso con mayor dedicación y tiempo se podria conectar por medio de wifi y codigo a el asistente virtual Alexa, dado que Alexa te permite programar para el sin ninguna restricción 

### Problemas / Obstáculos. 

Existieron varios problemas que se presentaron en la realización del proyecto. Por ejemplo:

●	Cuando soldamos uno de los componentes este no funcionaba pero, se pudo rectificar y arreglar este fallo soldando de nuevo la placa.

●	El sensor de gas detectaba gas cuando no existían partículas, se pudo resolver calibrando manualmente el sensor a 1500 partículas por metro para su detección.

●	Cuando el sistema mandaba una señal de detección se apagaba nuestra tira led verde pero no encendía la bocina ni la tira led roja. Para corregir este error se cambió la polaridad del ULN ya que se pensaba que este se conectaba de cierta manera y resultó que era inverso.

●	Contábamos con un buzzer para la detección de algún sensor pero la frecuencia de este

●	Sensibilidad al ruido, el proyecto en veces cuando se movia el esp32 empezaba a enviar ruido a las pantallas o el sensor se hacia mas o menos sensible.

### Resultados y Conclusiones

El resultado fue muy gratificante dado que el proyecto satisface el problema planteado, si bien en este problema se partió de una solución que podría parecer simple se termino haciendo un sistema digno de competir en el mercado en cuanto a funcionalidad, si bien se esperaba un resultado en el proyecto el cual estuviera mas limpio las continuas mejoras complicaban cada vez mas el circuito, sin mencionar que se tuvo que implementar una fuente de computadora a falta de una batería de 12v.

Mostrando su funcionamiento podemos ver que cuando no hay presencia de gas se despliega la luz verde y en la pantalla LCD despliega el mensaje “No hay gas”.

![image](https://user-images.githubusercontent.com/98352322/168680779-82110b03-7fb4-46b1-888f-b31b33c4c956.png)

En caso de que el sensor detecte gas como vemos en la siguiente imagen, mostrara una alerta visual (led en rojo) y una alerta sonora que es una alarma automotriz de 12v con 6 tonos.

![image](https://user-images.githubusercontent.com/98352322/168680828-6744dce0-e4c3-4c58-9909-acab6c61604d.png)

Video del proyecto funcionando:

https://drive.google.com/file/d/1DOTJb55SXMTuTHYOrzcRV_U-Resku4TW/view?usp=sharing


### Referencias

https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf

http://www.handsontec.com/dataspecs/module/I2C_1602_LCD.pdf

https://uelectronics.com/wp-content/uploads/2018/01/MQ-4.pdf

https://arduinoque.com/arduino/display-lcd-16x2-datasheet/

https://pdf1.alldatasheet.es/datasheet-pdf/view/25575/STMICROELECTRONICS/ULN2003.html

https://www.inventable.eu/2018/02/09/uln2003-driver-salida-microcontroladores/
