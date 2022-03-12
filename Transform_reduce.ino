#include <random>    //.1 Motor generador de numeros aleatorios
#include <vector>    //.2 vector
#include <algorithm> //.3 Operacion generate
#include <numeric>   //.4 transform_reduce
#include <iomanip>   //.5 fixed << setprecision()
#include <ctime>     //.6 tiempo de ejecucion
#include <cmath>
#include <iostream>

using namespace std;
unsigned t0, t1;

double result;
int _size = 9999;



void setup() {
  // put your setup code here, to run once:
t0= clock(); //.6 tiempo inicial
    cout << "\n---transform_reduce" << endl;
   
    //1.  Motor generador de numeros aleatorios.
    random_device rnd_device;  // Instancia del motor.
    mt19937 mersenne_engine {rnd_device()}; 
    //.1  Distribucion matematica. 
    uniform_real_distribution <double> x {1, 10};  
    //.1  Variable que contiene el numero aleatorio resultante.
    auto gen = [&x, &mersenne_engine](){return x(mersenne_engine);};

    //.2 declaracion del vector 
    vector<int> radio(500);  

    //.3
    generate(radio.begin(), radio.end(), gen);  

    //.4
    for (auto a=0 ; a<_size ; a++ ){
            result = transform_reduce(radio.begin(), radio.end(), 0.0, 
                [](double l, double i) { return l+i ;},           
                [](double r) {return M_PI*r*r; });
    }
    cout << fixed 
         << setprecision(3) 
         << "Area Total: " 
         << result <<" cm" << "\u00B2"<< endl;
    t1= clock(); //.6 tiempo final

    double time = (double(t1-t0)/CLOCKS_PER_SEC);
    cout << endl <<"Execution Time: " << time << endl;
    return 0;
}

void loop() {
  // put your main code here, to run repeatedly:

}
