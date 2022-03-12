// ej11.cpp: Herencia de Clases
// to-do: librerías, archivos

#include <iostream>
#include <cstdint>
#include <string>

using namespace std;

class Persona {
    protected:
        string    nom;
        uint8_t  edad;
        bool      eol;
    public:
        Persona(string n, uint8_t e, bool f=false): nom(n), edad(e), eol(f) { }
        friend ostream& operator<<(ostream& os, const Persona& p) {
            os << p.nom << ", " << unsigned(p.edad) << " años" << (p.eol? "\n" : "");
            return os;
        }
};

class Docente : public Persona {
    protected:
        bool tc; // true if maestro tiempo completo

    public:
        Docente(string n, uint8_t e, bool t=false): Persona(n,e), tc(t) {}
        friend ostream& operator<<(ostream& os, const Docente& d) {
            os << (Persona &)d << ", tiempo " << (d.tc? "completo" : "parcial") << endl;
            return os;
        }
};

class Alumno : public Persona
{
    protected:
        uint32_t  mat;

    public:
        Alumno(string n, uint32_t m, uint8_t e): Persona(n,e), mat(m) { }
        friend ostream& operator<<(ostream& os, const Alumno& alum) {
            os << (Persona &)alum << ", matrícula " << unsigned(alum.mat) << endl;
            return os;
        }
};



void setup() {
  // put your setup code here, to run once:
    Persona manto("Juan", 44, true);
    Docente maestro("Alberto", 48);
    Alumno luis = {"Luis", 84060820L, 25};
    Alumno noel("Noel", 84060833L, 24);
    cout << manto << maestro << luis << noel; // Polimorfismo
}

void loop() {
  // put your main code here, to run repeatedly:

}
