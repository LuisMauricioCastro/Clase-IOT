#include <iostream>
#include <vector>

using namespace std;

using Byte = uint8_t;

using Pins = vector<Byte>;

#define PIN1(p)       Pins{p}
#define PIN2(p,q)     Pins{p,q}
#define PIN3(p,q,r)   Pins{p,q,r}
#define PIN4(p,q,r,s) Pins{p,q,r,s}
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
    vector<Pins> chips = { PIN1(12), PIN2(13,14), PIN3(15,16,17) };
    for (const auto &c: chips) {
        for (const auto &p: c) {
            cout << (unsigned)p << " ";
        }
        cout << endl;
    }
}
