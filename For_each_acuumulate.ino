#include <random>    //.1 Motor generador de numeros aleatorios
#include <vector>    //.2 vector
#include <algorithm> //.3 Operacion generate
#include <numeric>   //.4 transform_reduce
#include <iomanip>   //.5 fixed << setprecision()
#include <ctime>     //.6 tiempo de ejecucion
#include <cmath>
#include <iostream>

using namespace std;
unsigned t1, t0;

double total;
int _size = 9999;

void area(double& r) { r = M_PI * r * r; } // paso parametro por referencia




void setup() {
  // put your setup code here, to run once:

    t0 = clock(); //.6 tiempo de inicio
    cout << "\n---for_each" << endl;
    //1.  Motor generador de numeros aleatorios.
    random_device rnd_device;  // Instancia del motor.
    mt19937 mersenne_engine {rnd_device()}; 
    //.1  Distribucion matematica. 
    uniform_real_distribution <double> x {1, 10};  
    //.1  Variable que contiene el numero aleatorio resultante.
    auto gen = [&x, &mersenne_engine](){return x(mersenne_engine);};

    //.2 declaracion del vector 
    vector<double> radio(500);  

    //.3
    generate(radio.begin(), radio.end(), gen); 

    //Prueba 10,000 ciclos
    for (auto a=0 ; a<_size ; a++){
        for_each(radio.begin(), radio.end(), area); // modifica c/elemento
        total = accumulate(radio.begin(), radio.end(), 0.0);
    }
    cout << fixed<< setprecision(3) << "Area total = " << total;
    t1 = clock(); //.6 tiempo final
    //.6
    double time = (double(t1-t0)/CLOCKS_PER_SEC);
    cout << fixed << setprecision(3) 
         <<"\n\nExecution Time: " << time << endl;
}

void loop() {
  // put your main code here, to run repeatedly:

}
