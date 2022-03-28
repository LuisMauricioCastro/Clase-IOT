#include <iostream>
#include <vector>

using namespace std;

using Byte = uint8_t;

using Pins = vector<Byte>;
void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
    vector<Pins> chips = { Pins{12}, Pins{13,14}, Pins{15,16,17} };
    for (const auto &c: chips) {
        for (const auto &p: c) {
            cout << (unsigned)p << " ";
        }
        cout << endl;
    }
}
