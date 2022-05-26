# Prototipo para Monitoreo de Calidad del Aire
Programas realizados en la clase de Internet of things 
Universidad Autonoma de Chihuahua
Carrera: Ingeniería EN Sistemas Computacionales en Hardware
Materia: Internet de las cosas
Grupo 8HW1

Alumno: 
Luis Mauricio Castro Gutiérrez
Rodrigo Alberto Valdez Covarrubias

Profesor:  Pacheco González Alberto

Placa de desarrollo: ESP32 DEVKIT V1

---

### Planteamiento del Problema
El proyecto busca lograr determinar si la ubicación en donde está el sistema tiene niveles de metano no peligrosos, también el sensado de la temperatura y humedad relativa del ambiente, esto por la gran cantidad de fugas de gas que existen cotidianamente.
### Objetivos
Se busca hacer con un microcontrolador ESP32 el cual por medio de componentes mostrará los datos de los sensores y alertará si el nivel de metano es suficiente como para lograr ser peligroso, en caso de detectarlo activara una alarma visual y sonora para que el usuario sepa que existe una fuga de gas.
### Metodología
Nuestra metodología a usar es la [metodología Scrumban](https://asana.com/es/resources/scrumban)  esta metodología permite que se agreguen tareas individuales al plan. Esto permite que los planes de proyectos mantengan una estructura simple y clara, esta observa al proyecto como un rompecabezas el cual requiere de todas las piezas para estar completo, pero no obliga a acomodar las piezas en un orden especifico. 
Basados en esta metodología decidimos dividirnos tareas como el investigar y probar sensores que íbamos a usar, investigar que librerías y que código tendríamos que incluir para que las pantallas con el adaptador funcionaran, integración del driver [ULN2003](https://pdf1.alldatasheet.es/datasheet-pdf/view/25575/STMICROELECTRONICS/ULN2003.html) , implementación de alertas sonora y visual, compra de componentes y uso de pines del ESP32.

Primeramente, se buscó que sensores teníamos a la mano y si estos servían para nuestro proyecto, Se determino que teníamos el sensor de temperatura y humedad, pero no el de metano por lo que se tuvo que comprar. Se investigo en cada componente el uso de los pines del ESP32 dado que algunos pines no son usables por cuestiones de que se usan internamente por el ESP32. Después de lo anterior se decidió hacer las pruebas con los sensores de metano, humedad y temperatura, para esto se tuvo que investigar librerías junto con su funcionamiento en código. Una vez realizadas las pruebas se implementaron las alertas sonoras, al ver que no alertaban del todo se decidió implementar el driver ULN2003 para poder usar alarmas sonoras y visuales más potentes.


### Descripción de Materiales

●	Microcontrolador ESP-32.

●	Driver ULN2003.

●	Cables Dupont.

●	Bocina automotriz DC 12V.

●	Fuente de poder 12V.

●	Tiras led 12 volts.

●	2 Protoboards.

●	Sensor de humedad y temperatura DHT11.

● 3 Pantallas LCD.

●	3 adaptadores de conexiones LCD a i2c.

●	1 Sensor de metano MQ4

[Sensor de Temperatura y Humedad DHT11](https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)

![image](https://user-images.githubusercontent.com/98352322/168677494-364a8489-9f6b-4ccb-8c94-5bb5f46216d6.png)

Este sensor es muy usado gracias a su alta fiabilidad y su estabilidad, esto gracias a que su señal digital ya viene previamente calibrada. Sumado a esto su instalación y uso es muy fácil pues solo cuenta con 3 conexiones (Voltaje 3V/5V, Tierra y I/O), por software se tiene una librería para la interacción con el sensor. 

Especificaciones técnicas detalladas:

![image](https://user-images.githubusercontent.com/98352322/168677568-e3c6d7de-57b7-46b3-95b8-5b47476271c4.png)

[Sensor de Metano MQ-4](https://uelectronics.com/wp-content/uploads/2018/01/MQ-4.pdf)

![image](https://user-images.githubusercontent.com/98352322/168677600-109af7c4-6af9-460b-9138-655a5d1ef029.png)

Este sensor está encargado de detectar el gas metano en el ambiente, tiene 2 salidas de información (analógica y digital), su instalación es rápida y sencilla dado que cuenta con 4 terminales (Voltaje, Tierra, Salida digital, Salida analógica) esto dándonos la capacidad de con un solo pin enviar salida de datos a él microcontrolador.

Especificaciones técnicas detalladas:

![image](https://user-images.githubusercontent.com/98352322/168677680-3c62b662-23eb-4fa2-b8e4-3ce35534b8bb.png)

[Pantalla LCD (16,2)](http://www.handsontec.com/dataspecs/module/I2C_1602_LCD.pdf) 

![image](https://user-images.githubusercontent.com/98352322/168677708-3735367c-ec8e-4111-bbf1-b8eac7ed399c.png)

Pantalla de 16 caracteres y 2 renglones, serán las encargadas de mostrar los datos de los sensores al usuario.

[Adaptador Display LCD (16,2) a i2c](http://www.handsontec.com/dataspecs/module/I2C_1602_LCD.pdf)

![image](https://user-images.githubusercontent.com/98352322/168677760-9f0e7db9-858b-48b8-8502-41907d072e0b.png)

Este adaptador permite transformar un display (16,2) común y corriente a un protocolo de comunicación i2c, este protocolo es muy beneficioso pues nos permite controlar múltiples pantallas con solo 2 pines, este adaptador cuenta con un cambio de dirección si se le es soldado ciertas terminales, esto para manejar múltiples pantallas.

Especificaciones técnicas detalladas:

Compatible con tarjetas Arduino u otros microcontroladores que tengan bus I2C 

Tipo de display: Blanco negativo en fondo azul.

Direcciones I2C:0x38-0x3F (0x3F por default)

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

[Driver ULN2003](https://pdf1.alldatasheet.es/datasheet-pdf/view/25575/STMICROELECTRONICS/ULN2003.html)

![image](https://user-images.githubusercontent.com/98352322/168678099-ccb93bf9-6482-458d-89f9-aa3952301fc2.png)

Driver usado para mandar una señal de 3.3/5V a las entradas, en caso de que la mande el output hará tierra, como se puede ver en el siguiente diagrama:

![image](https://user-images.githubusercontent.com/98352322/168678136-e03dfe29-55fe-4982-9e81-f45c18edcfc8.png)

Sirena automotriz DC 12V 6 tonos

![image](https://user-images.githubusercontent.com/98352322/168678173-32202eb6-ad6b-4534-a7b9-c15e0fd088ba.png)

Sirena de auto que funciona con 12V y tiene 6 tonos, se usará para medio de alerta sonoro frente a la presencia de metano.

Cables Dupont

![image](https://user-images.githubusercontent.com/98352322/168678209-c0b827cc-4ecf-4919-b291-15c7eb475aa1.png)

Se realizará la conexión entre los componentes del sistema para mandar los datos, voltajes y tierras.

Microcontrolador ESP32

![image](https://user-images.githubusercontent.com/98352322/168678326-e811f4b5-57cb-46a1-b65d-9c252c50151f.png)

Voltaje de Alimentación (USB): 5V DC

Voltaje de Entradas/Salidas: 3.3V DC

SoC: ESP32

CPU principal: Tensilica Xtensa 32-bit LX6

Frecuencia de Reloj: hasta 240Mhz

Desempeñó: Hasta 600 DMIPS

Procesador secundario: Permite hacer operaciones básicas en modo de ultra bajo consumo

Wifi: 802.11 b/g/n/e/i (802.11n @ 2.4 GHz hasta 150 Mbit/s)

Bluetooth:v4.2 BR/EDR and Bluetooth Low Energy (BLE)

Xtensa® Dual-Core 32-bit LX6 microprocessors, up to 600 DMIPS

Memoria: 448 KByte ROM

520 KByte SRAM

16 KByte SRAM in RTC

QSPI Flash/SRAM, 4 MBytes

Pines Digitales GPIO: 24 (Algunos pines solo como entrada)

Conversor Analógico Digital: Dos ADC de 12bits tipo SAR, soporta mediciones en hasta 18 canales, algunos pines soportan un amplificador con ganancia programable

UART: 2

Chip USB-Serial: CP2102

Antena en PCB

Seguridad: Estándares IEEE 802.11 incluyendo WFA, WPA/WPA2 and WAPI

1024-bit OTP, up to 768-bit for customers

Aceleración criptográfica por hardware: AES, HASH (SHA-2), RSA, ECC, RNG

### Descripción del Diseño

Primeramente, se implementó paso a paso todas las conexiones en fritzing, si bien algunas conexiones pueden llegar a confundir es lo más acomodado que puede estar el diagrama, explicaremos parte por parte el diseño

![image](https://user-images.githubusercontent.com/98352322/168678467-5acfd584-5fa2-4482-ac20-6b6f258b7ba4.png)


En esta parte podemos ver que todas las pantallas cuentan con el protocolo i1c dado que son de 4 pines y en la descripción nos aclara, por lo que conectaremos voltaje y tierra a todas, además conectaremos a los pines del ESP32 i2c SCL y i2c SDA a las terminales positiva y negativa de la parte superior, por estas líneas se enviara la información a las pantallas por ende tenemos que conectar sus pines SCL y SDA en donde corresponden en la línea.

Diagrama de Conexiones Pantallas LCD:

![image](https://user-images.githubusercontent.com/98352322/168678490-ffbd4405-25aa-44aa-aa27-64a100030c40.png)

Los sensores estarán conectados a voltaje y tierra, sumados a estos el pin de datos del sensor DHT11 se conectará a el pin 14 del ESP32, el sensor MQ-4 conectará su terminal del sensor analógico a el pin 4.

Diagrama de Conexiones Sensores:

![image](https://user-images.githubusercontent.com/98352322/168678533-7d5db326-c34a-4715-a81a-7a1933229395.png)

El ULN2003 va a recibir tres entradas de los pines 25,33 y 32, estas entradas pasaran por el driver y conectaran a la tierra del led verde (el primero), la tierra del led rojo(el segundo) y de nuestra alarma, esto completando el circuito dado que una fuente de poder de 12 volts está enviando tierra al ULN2003 y positivo a los componentes si el ULN2003 envía tierra a cualquiera de los componentes el circuito se cerrara y el componente funcionara.

Diagrama de Conexiones ULN2003:

![image](https://user-images.githubusercontent.com/98352322/168678576-3dfd83c7-268c-4189-a74a-1e1b817eca7e.png)




Pd: se puso la interpretación de la tira led de 12v solo con un led y la interpretación de la sirena de 12v con un buzzer, se puso una batería de 12v en sustituto a nuestra fuente de poder de computadora modificada.

Importante tener en cuenta el diagrama de conexiones del ESP32 todo el tiempo pues se requiere en cada conexión:

![image](https://user-images.githubusercontent.com/98352322/168682044-42c1322e-dffe-4696-b292-dbe848dfb4b6.png)


### Desarrollo del Proyecto

Primeramente, escogimos el tema dado a que por situaciones personales se podría llegar a implementar fuera de la clase.
Una vez decidido el proyecto se empezó a conseguir los materiales, inicialmente se estuvo probando cada sensor por separado para entender su funcionamiento.


Al inicio se implementó únicamente el sensor DHT y se hizo pruebas con el siguiente código:		
```C++
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

Al saber que funcionaba correctamente decidimos ver si el sensor mq4 funcionaba con el siguiente código:
```C++
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
```C++

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

```C++
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

Logramos conseguir una alarma de carro a muy buen precio y se conectó a la alimentación de 12 Volts. Gracias al sonido de la alarma pudimos obtener el resultado deseado ya que ahora podíamos escuchar de manera clara cuando el sistema detectaba una anomalía, esto para poder alertar a las personas de este problema.

![image](https://user-images.githubusercontent.com/98352322/168680242-7047baeb-7ac3-4e16-a648-c812f78a9151.png)

Se puede observar en la imagen como se aplicaron las tiras led de 12 Volts, ya que estas brindaran el estado del ambiente de manera lumínica. Demostrando que el color verde es el estado natural correcto y el rojo es cuando ha detectado una anomalía en el ambiente, uno de los principales casos cuando se activa el color rojo en la tira led es cuando ha detectado monóxido de carbono, ya que esta es la principal sustancia que se avecina en los hogares. 

![image](https://user-images.githubusercontent.com/98352322/168680291-85059865-9a12-4684-9c54-341e1bd67d1e.png)

### Planeación
Gracias a que el proyecto fue desarrollado en tiempo y forma nos gustaría implementar una caja donde esté guardado, ya que mejora la estética, facilidad de transportar, y sobre todo se vuelve más amigable su uso.

También una posible mejora seria implementar el empaquetado de nuestro sistema mediante una impresora 3D, ya que de esta manera podemos realizar el diseño, a nuestro gusto, buscando la mejor apariencia y el mejor espacio para el acomodo de nuestro sistema.

También una posible mejora implementar un servidor el cual pueda observar el valor de los sensores o incluso con mayor dedicación y tiempo se podría conectar por medio de wifi y código a el asistente virtual Alexa, dado que Alexa te permite programar para el sin ninguna restricción 

### Problemas / Obstáculos. 

Existieron varios problemas que se presentaron en la realización del proyecto. Por ejemplo:

●	Cuando soldamos uno de los componentes este no funcionaba, pero, se pudo rectificar y arreglar este fallo soldando de nuevo la placa.

●	El sensor de gas detectaba gas cuando no existían partículas, se pudo resolver calibrando manualmente el sensor a 1500 partículas por metro para su detección.

●	Cuando el sistema mandaba una señal de detección se apagaba nuestra tira led verde pero no encendía la bocina ni la tira led roja. Para corregir este error se cambió la polaridad del ULN ya que se pensaba que este se conectaba de cierta manera y resultó que era inverso.

●	Contábamos con un buzzer para la detección de algún sensor, pero la frecuencia de este

●	Sensibilidad al ruido, el proyecto en veces cuando se movía el esp32 empezaba a enviar ruido a las pantallas o el sensor se hacía más o menos sensible.

### Resultados y Conclusiones

El resultado fue muy gratificante dado que el proyecto satisface el problema planteado, si bien en este problema se partió de una solución que podría parecer simple se terminó haciendo un sistema digno de competir en el mercado en cuanto a funcionalidad, si bien se esperaba un resultado en el proyecto el cual estuviera mas limpio las continuas mejoras complicaban cada vez mas el circuito, sin mencionar que se tuvo que implementar una fuente de computadora a falta de una batería de 12v.

Mostrando su funcionamiento podemos ver que cuando no hay presencia de gas se despliega la luz verde y en la pantalla LCD despliega el mensaje “No hay gas”.

![image](https://user-images.githubusercontent.com/98352322/168680779-82110b03-7fb4-46b1-888f-b31b33c4c956.png)

Proyecta detectando que no detecta gas

En caso de que el sensor detecte gas como vemos en la siguiente imagen, mostrara una alerta visual (led en rojo) y una alerta sonora que es una alarma automotriz de 12v con 6 tonos.

![image](https://user-images.githubusercontent.com/98352322/168680828-6744dce0-e4c3-4c58-9909-acab6c61604d.png)

Proyecto detectando gas

Video del proyecto funcionando:

https://drive.google.com/file/d/1DOTJb55SXMTuTHYOrzcRV_U-Resku4TW/view?usp=sharing

https://photos.app.goo.gl/XyjaC3mmCq5swmvw7

Videos y fotografias del desarrollo del proyecto:

https://photos.app.goo.gl/fx3UbA5Nq7n7yaQz7

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


# Prototype for Air Quality Monitoring
Program made in the Internet of things class 
Universidad Autonoma de Chihuahua
Career: Computer Systems Engineering in Hardware
Subject: Internet of Things

Students: 

Luis Mauricio Castro Gutiérrez

Rodrigo Alberto Valdez Covarrubias

Professor:  Pacheco Gonzalez Alberto

development board: ESP32 DEVKIT V1

### Problem Statement
The project seeks to determine if the location where the system is has levels of methane not dangerous, also sensing the temperature and relative humidity of the environment, this by the large amount of gas leaks that exist daily.
### Objectives
It is want to do with an ESP32 microcontroller which by means of components will show the data of the sensors and alert if the level of methane is sufficient to achieve be dangerous, if detected, a visual and audible alarm will be activated so that the user knows that there is a gas leak.
### Methodology
Our methodology to use is the Scrumban methodology this methodology allows individual tasks to be added to the plan. This allows the project plans to maintain a simple and clear structure, it looks at the project as a puzzle which requires all the pieces to be complete, but does not force to accommodate the pieces in a specific order. 
Based on this methodology we decided to divide tasks such as researching and testing sensors that we were going to use, investigating which libraries and code we would have to include for the screens with the adapter to work, integration of the ULN2003 driver, implementation of audible and visual alerts, purchase of components and use of ESP32 pins.
First, we looked for sensors we had at hand and if these were useful for our project, it was determined that we had the temperature and humidity sensor, but not the methane sensor so it had to be purchased. The use of ESP32 pins was investigated in each component since some pins are not usable due to issues that are used internally by the ESP32. After the above it was decided to do the tests with the sensors of methane, humidity and temperature, for this it had to investigate libraries along with their operation in code. Once the tests were carried out, the sound alerts were implemented, seeing that they did not alert at all, it was decided to implement the ULN2003 driver in order to use more powerful sound and visual alarms.
### Material Description
●	Microcontroller ESP-32.

●	Driver ULN2003.

●	Dupont Wires.

●	Car Horn DC 12V.

●	PSU 12V.

●	Led strips 12 volts.

●	2 Protoboards.

●	DHT11 Humidity and Temperature sensor.

● 3 LCD Screens.

●	3 adapters for LCD to i2c connections

●	1 Methane Sensor MQ4.

[Humidity and Temperature sensor DHT11](https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)

![image](https://user-images.githubusercontent.com/98352322/168677494-364a8489-9f6b-4ccb-8c94-5bb5f46216d6.png)

This sensor is widely used thanks to its high reliability and stability, thanks to the fact that its digital signal is already calibrated. Added to this its installation and use is very easy as it only has 3 connections (Voltage 3V/5V, Earth and I/O), software has a library for interaction with the sensor.

Detailed technical specifications:

![image](https://user-images.githubusercontent.com/98352322/168677568-e3c6d7de-57b7-46b3-95b8-5b47476271c4.png)

[Methane Sensor MQ-4](https://uelectronics.com/wp-content/uploads/2018/01/MQ-4.pdf)

![image](https://user-images.githubusercontent.com/98352322/168677600-109af7c4-6af9-460b-9138-655a5d1ef029.png)

This sensor is responsible for detecting methane gas in the environment, has 2 information outputs (analog and digital), its installation is fast and simple since it has 4 terminals (Voltage, Earth, Digital output, Analog output) this giving us the ability to with a single pin send data output to it microcontroller.
Detailed technical specifications:

![image](https://user-images.githubusercontent.com/98352322/168677680-3c62b662-23eb-4fa2-b8e4-3ce35534b8bb.png)

 LCD screen (16,2) 

![image](https://user-images.githubusercontent.com/98352322/168677708-3735367c-ec8e-4111-bbf1-b8eac7ed399c.png)

Display 16 characters and 2 lines, will be responsible for displaying sensor data to the user.
Adapter for LCD to i2c connections

![image](https://user-images.githubusercontent.com/98352322/168677760-9f0e7db9-858b-48b8-8502-41907d072e0b.png)

This adapter allows to transform a common display (16,2) and current to a communication protocol i2c, this protocol is very beneficial as it allows us to control multiple screens with only 2 pins, This adapter features a change of direction if you are soldered certain terminals, this to handle multiple displays.
Detailed technical specifications:

Compatible with Arduino cards or other microcontrollers with I2C bus
Display type: Negative white on blue background.

I2C address: 0x38-0x3F (0x3F by default)

Power supply: 5V

Interface: I2C to 4-bit LCD data and control lines.

Contrast adjustment: Potentiometer on plate.

Rear light control: Firmware or a jumper cable.

Size: 80x36 mm

Protoboard

![image](https://user-images.githubusercontent.com/98352322/168677889-757bef72-502c-4bc9-8da0-5f526703660c.png)

Protoboard Breadboard 830 White Dots
Connection plate (Protoboard), with 1 block, 2 strips, without terminals and 830 perforations, ideal for assembling prototypes of electronic circuits.

LED strips 12 volts

![image](https://user-images.githubusercontent.com/98352322/168677949-b06dc0e7-1458-46e9-a854-3e7071805c72.png)

The function of the LED strips is to visually show the behavior of the system.

12V Power Supply/Computer Source Modified to Deliver 12V

![image](https://user-images.githubusercontent.com/98352322/168678050-13ba8186-5531-4f0e-a007-a94afa580f8d.png)

It will be responsible for supplying power to components that work with 12V, can be used batteries or more compact solutions, in our case we will use a modified power source to have available a 12 volt terminal and ground.
Driver ULN2003

![image](https://user-images.githubusercontent.com/98352322/168678099-ccb93bf9-6482-458d-89f9-aa3952301fc2.png)

Driver used to send a 3.3/5V signal to the inputs, in case the output sends it will ground, as can be seen in the following diagram:

![image](https://user-images.githubusercontent.com/98352322/168678136-e03dfe29-55fe-4982-9e81-f45c18edcfc8.png)

Automotive siren DC 12V 6 tones

![image](https://user-images.githubusercontent.com/98352322/168678173-32202eb6-ad6b-4534-a7b9-c15e0fd088ba.png)

Car siren that works with 12V and has 6 tones, will be used for sound warning medium against the presence of methane.
Dupont cables

![image](https://user-images.githubusercontent.com/98352322/168678209-c0b827cc-4ecf-4919-b291-15c7eb475aa1.png)

The connection between the components of the system will be made to send the data, voltages and earths.
ESP32

![image](https://user-images.githubusercontent.com/98352322/168678326-e811f4b5-57cb-46a1-b65d-9c252c50151f.png)

Power Voltage (USB): 5V DC

Input/Output Voltage: 3.3V DC

SoC: ESP32

Main CPU: Tensilica Xtensa 32-bit LX6

Clock Frequency: up to 240Mhz

Served: Up to 600 DMIPS

Secondary processor: Allows basic operations in ultra-low power mode

Wifi: 802.11 b/g/n/e/i (802.11n @ 2.4 GHz up to 150 Mbit/s)

Bluetooth:v4.2 BR/EDR and Bluetooth Low Energy (BLE)

Xtensa Dual-Core 32-bit LX6 microprocessors, up to 600 DMIPS

Memory: 448 KByte ROM

520 KByte SRAM

16 KByte SRAM in RTC

QSPI Flash/SRAM, 4 MBytes

GPIO Digital Pins: 24 (Some pins only as input)
Analog Digital Converter: Two 12bit SAR-type ADCs, support measurements on up to 18 channels, some pins support a programmable gain amplifier
UART: 2

Chip USB-Serial: CP2102

Antenna on PCB
Security: IEEE 802.11 standards including WFA, WPA/WPA2 and WAPI

1024-bit OTP, up to 768-bit for customers

Hardware cryptographic acceleration: AES, HASH (SHA-2), RSA, ECC, RNG
### Design description

First, all connections were implemented step by step in fritzing, although some connections can be confusing is the most accommodating that the diagram can be, we will explain part by part the design.

![image](https://user-images.githubusercontent.com/98352322/168678467-5acfd584-5fa2-4482-ac20-6b6f258b7ba4.png)

Connection Diagram LCD Screen:

![image](https://user-images.githubusercontent.com/98352322/168678490-ffbd4405-25aa-44aa-aa27-64a100030c40.png)

In this part we can see that all screens have the i1c protocol since they are 4 pins and, in the description, clarifies us, so we will connect voltage and earth to all, we will also connect to the pins of the ESP32 i2c SCL and i2c SDA to the positive and negative terminals of the top, for these lines the information will be sent to the screens therefore we have to connect their pins SCL and SDA where they correspond in the line.
Connection Diagram Sensors:

![image](https://user-images.githubusercontent.com/98352322/168678533-7d5db326-c34a-4715-a81a-7a1933229395.png)

The sensors will be connected to voltage and ground, added to these the data pin of the DHT11 sensor will be connected to pin 14 of the ESP32, the MQ-4 sensor will connect its analog sensor terminal to pin 4.
Connection Diagram ULN2003:

![image](https://user-images.githubusercontent.com/98352322/168678576-3dfd83c7-268c-4189-a74a-1e1b817eca7e.png)

The ULN2003 will receive three inputs from pins 25,33 and 32, these inputs will pass through the driver and connect to the green LED earth(the first), the red LED earth(the second) and our alarm, this completing the circuit since a 12 volt power source is sending earth to the ULN2003 and positive to the components if the ULN2003 sends earth to any of the components the circuit will close and the component will work.

Note: Got 12v LED strip interpretation only with an LED and 12v siren interpretation with a buzzer, got a 12v battery replacing our modified computer power source.

Important to keep in mind the ESP32 connection diagram all the time as it is required in each connection:

![image](https://user-images.githubusercontent.com/98352322/168682044-42c1322e-dffe-4696-b292-dbe848dfb4b6.png)


### Project development

First, we chose the subject given that personal situations could be implemented outside the class.
Once the project was decided, materials began to be obtained, initially each sensor was tested separately to understand its operation.

Initially, only the DHT sensor was implemented and the following code was tested:	
```C++
// Include library
#include <DHT.h>
// We define the digital pin where the sensor is connected
#define DHTPIN 14
// Depending on the type of sensor
#define DHTTYPE DHT11
// We initialize the DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  // We initialize serial communication
  Serial.begin(9600);
  // We start the DHT sensor
  dht.begin();
}
void loop() {
    // We waited 5 seconds between measurements
  delay(5000);
  // Leemos la humedad relativa
  float h = dht.readHumidity();
  // We read the temperature in Celsius (default)
  float t = dht.readTemperature();
  // We read the temperature in Fahreheit degrees
  float f = dht.readTemperature(true);
  // We check if there has been any error in reading
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error getting data from DHT11 sensor");
    return;
  }
  // Calculate the heat in Fahreheit
  float hif = dht.computeHeatIndex(f, h);
  // Calculate the heat in degrees centigrade
  float hic = dht.computeHeatIndex(t, h, false);
  Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("heat: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
}
```

Knowing that it worked correctly we decided to see if the mq4 sensor worked with the following code:
```C++
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


We got a bad surprise, because the sensor it had was not working, so we had to buy another one and since it worked correctly it was decided to implement the data sample.

When looking for how to implement screens with I2C protocol, we first looked for OLED screens, we looked for the following model:

![image](https://user-images.githubusercontent.com/98352322/168678701-88951810-06a3-496f-8fb8-b5b531de3e66.png)

But unfortunately when buying they had to return since they could not be changed direction, so it was decided to use LCD screens (16,2), for this requires the display adapter to the communication protocol i2c.

For this we require to weld the screens with the adapter, in turn we will have to weld the addresses of the adapters to with the code that will be seen below knowing the addresses of each screen.

![image](https://user-images.githubusercontent.com/98352322/168678947-5e122852-48da-41c6-8fcc-2794152aedcc.png)

In the above image we can see where the direction of each adapter is changed.

The code to search for the addresses is as follows:
```C++

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

When deciphering the directions we will integrate the screens and sensors, once integrated this will pass to the integration of a sound and visual alert, this was first sought to use a buzzer and simple LEDs, but it was noticed that they did not generate enough alert, so it was sought to use a more powerful siren, this looking for something that would convert a signal from 5 volts to 12 volts were considered options such as the use of relays but was deicidio better to use a driver ULN2003 this that fulfils the function of a relay without being one, with this it was decided to take advantage of the driver to also activate 2 strips LEDs of 2 different colors, both of 12volts, powered by a computer source, since this gives us a power for the components of 12 Volts.

![image](https://user-images.githubusercontent.com/98352322/168679315-b6051fb6-0c58-4b57-a2a0-cc85a20bda4f.png)

So everything implemented in code looks like this:

```C++
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
// Create the direction lcd object 0x3F y 16 columms x 2 rows
LiquidCrystal_I2C lcd(0x23,16,2);  //  << Address 1
LiquidCrystal_I2C lcd2(0x25, 16, 2); // << Address 2
LiquidCrystal_I2C lcd3(0x26, 16, 2); // << Address 3

// We define the digital pin where the sensor is connected
#define DHTPIN 14
// Depending on the type of sensor
#define DHTTYPE DHT11
const int buzzer = 25; 
// We initialize the DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);

int Gas_analog = 4;    // used for ESP32
void setup() {
  pinMode(33,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(buzzer,OUTPUT);
  Serial.begin(115200);
   dht.begin();
  // Initialize the LCD
  lcd.init();
  lcd2.init();
  lcd3.init();
  // Turn on the backlight.
  lcd.backlight();
  lcd2.backlight();
  lcd3.backlight();
  // We write the Message on the LCD.
  
  lcd2.print("Temperature");
  lcd3.print("Humidity");
  
}
void loop() {
  // We read the relative humidity
  float h = dht.readHumidity();
  // We read the temperature in Celsius (default)
  float t = dht.readTemperature();
  // We read the temperature in Fahreheit degrees
  float f = dht.readTemperature(true);
 // We started the gas senor analogically
  int gassensorAnalog = analogRead(Gas_analog);
// Set the cursor on the screens
  lcd2.setCursor(0, 1);
  lcd3.setCursor(0, 1);
   // We write the temperature, humidity and methane gas number
  lcd2.print(t);
  lcd2.print(" Degrees C");
  lcd3.print(h);
  lcd3.print(" Relative");
  lcd.setCursor(0, 1);
  lcd.print(gassensorAnalog);
  lcd.print("ppm ");
 // In case the ppm exceeds 1400 units will send the legend that there is gas and sound the alarm
    if (gassensorAnalog > 1600) {
      lcd.setCursor(0, 0);
    lcd.println("there is gas   ");
    digitalWrite(33,HIGH);
    digitalWrite(32,LOW);
    digitalWrite(buzzer,HIGH);
  else {
    lcd.setCursor(0, 0);
    lcd.println("there is no gas");
    digitalWrite(32,HIGH);
    digitalWrite(33,LOW);
    digitalWrite(buzzer,LOW);
  }
  delay(100);
  
}
```
It had a buzzer but this was not perceived well, so we chose to replace it with an alarm.

We managed to get a car alarm at a very good price and connected to the 12 Volts power. Thanks to the sound of the alarm we were able to get the desired result as we could now hear clearly when the system detected an anomaly, this in order to alert people to this problem.

![image](https://user-images.githubusercontent.com/98352322/168680242-7047baeb-7ac3-4e16-a648-c812f78a9151.png)

You can see in the image how the 12 Volt LED strips were applied, as these will provide the state of the environment in a luminous way. Proving that the green color is the correct natural state and the red is when it has detected an anomaly in the environment, one of the main cases when the red color is activated in the LED strip is when it has detected carbon monoxide, since this is the main substance coming in homes.

![image](https://user-images.githubusercontent.com/98352322/168680291-85059865-9a12-4684-9c54-341e1bd67d1e.png)

### Planning
Thanks to the fact that the project was developed in time and form we would like to implement a box where it is stored, as it improves the aesthetics, ease of transport, and above all becomes more user-friendly.

Also a possible improvement would be to implement the packaging of our system through a 3D printer, since in this way we can realize the design, to our liking, looking for the best appearance and the best space for the accommodation of our system.
Also a possible improvement implement a server which can observe the value of the sensors or even with greater dedication and time you could connect via wifi and code to the virtual assistant Alexa, since Alexa allows you to program for it without any restrictions
### Problems/Obstacles.
There were several problems that arose in the implementation of the project. For example:

When we welded one of the components this did not work, but, it could be rectified and fixed this failure by welding the plate again.

The gas sensor detected gas when there were no particles, it could be solved by manually calibrating the sensor to 1500 particles per meter for detection.

●	When the system sent a detection signal our green LED strip was turned off but neither the horn nor the red LED strip was turned on. To correct this error the polarity of the ULN was changed as it was thought to be connected in a certain way and turned out to be inverse.

We had a buzzer for the detection of some sensor, but the frequency of this

Noise sensitivity, the project sometimes when moving the ESP32 started to send noise to the screens or the sensor became more or less sensitive.
### Results and conclusions

The result was very gratifying given that the project satisfies the problem posed, although in this problem was based on a solution that could seem simple ended up making a system worthy of competing in the market in terms of functionality, Although it was expected a result in the project which was cleaner the continuous improvements complicated the circuit more and more, not to mention that a computer source had to be implemented in the absence of a 12v battery.



![image](https://user-images.githubusercontent.com/98352322/168680779-82110b03-7fb4-46b1-888f-b31b33c4c956.png)

Showing its operation we can see that when there is no presence of gas the green light is displayed and the LCD displays the message "No gas".



![image](https://user-images.githubusercontent.com/98352322/168680828-6744dce0-e4c3-4c58-9909-acab6c61604d.png)

In case the sensor detects gas as we see in the following image, it will show a visual alert (LED in red) and a sound alert that is a 12v automotive alarm with 6 tones.

Video of the project running:

https://drive.google.com/file/d/1DOTJb55SXMTuTHYOrzcRV_U-Resku4TW/view?usp=sharing

https://photos.app.goo.gl/XyjaC3mmCq5swmvw7

Videos and photographs of the development of the project:

https://photos.app.goo.gl/fx3UbA5Nq7n7yaQz7
