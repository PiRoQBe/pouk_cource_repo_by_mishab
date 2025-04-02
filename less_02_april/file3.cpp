#include <iostream>

using namespace std;

class Country {
public: 
    void language(){
        cout << "English\n";
    }
};

class Australia: public Country{

};

class Lebanon: public Country{
public:
    void language(){
        cout << "Arabic\n";
    }
};

class Spain: public Country{
public: 
    void language(){
        cout << "Spanish\n";
    }
};

int main(){
    Australia au;
    Lebanon lb;
    Spain sp;

    au.language();
    lb.language();
    sp.language();

    return 0;
}