// ej12.cpp: Prog. Genérica, APIs (alias), polimorfismo y colecciones
// to-do: librerías, archivos

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>

using namespace std;

// API type aliases
using Name = string;
using Age = uint8_t;
using Work = bool;
using ID = uint32_t;

class Persona {
    private:
        Name nom;
        Age  edad;

    protected: virtual void print(ostream& os) const { // interface/protocol
        os << nom << ", " << unsigned(edad) << " años";
    }

    public:
        Persona(Name n, Age e): nom(n), edad(e) { } // API
        friend ostream& operator<<(ostream& os, const Persona& p) { // API Wrapper
            p.print(os); // polimorfismo
            return os;
        }
};

class Docente : public Persona
{
    protected: Work full; // tiempo-completo

    protected: virtual void print(ostream& os) const { // interface
        Persona::print(os); // reuse
        os << ", docente tiempo " << (full? "completo" : "parcial");
    }
    
    public: Docente(Name n, Age e, Work f=false): Persona(n,e), full(f) { } // API
};

class Alumno : public Persona
{
    private: ID mat;

    protected: virtual void print(ostream& os) const { // interface
        Persona::print(os); // reuse
        os << ", matrícula " << unsigned(mat);
    }

    public: Alumno(Name n, ID m, Age e): Persona(n,e), mat(m) { } // API
};

using GrupoPersonas = vector<Persona *>;

void setup() {
  // put your setup code here, to run once:
    GrupoPersonas univ = { // colección genérica
        new Persona("Chuy, encargado Lab", 44),
        new Docente("Juan", 48),
        new Alumno("Luis", 84060820L, 25),
        new Alumno("Noel", 84060833L, 24)
    };

    for (const auto p: univ) {
        cout << *p << endl; // operación polimórfica
    }
}

void loop() {
  // put your main code here, to run repeatedly:

}
