

#include <iostream>
#include <cstdint>

using namespace std;

class Alumno { // Def clase
    // atributos privados
    // Tarea: agregar nombre (usando cadenas C++, no usar char *)
    uint32_t mat;
    uint8_t edad;
public: // métodos públicos
    Alumno(uint32_t m, uint8_t e): mat(m), edad(e) { } // constructor
    friend ostream& operator<<(ostream& os, const Alumno& alum); // función amiga
};

ostream& operator<<(ostream& os, const Alumno& alum) {
    // tarea: agregar imprimir nombre
    os << "Soy " << unsigned(alum.mat)
       << " y tengo " << unsigned(alum.edad) << " años" << endl;
    return os;
}

void setup() {
  // put your setup code here, to run once:
    Alumno luis = {84060820L, 25}; // Instanciación con notación de inizializador {}
    Alumno noel(84060833L, 24);    // Instanciación con notación de constructor()
    cout << luis << noel;
}

void loop() {
  // put your main code here, to run repeatedly:

}
