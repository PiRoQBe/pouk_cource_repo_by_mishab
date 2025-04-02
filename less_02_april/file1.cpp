#include <iostream>

using namespace std;

class Employee {
private:
    string name;
    int age;
    double salary;
public:
string getName(){
    return name;
}

int getAge(){
    return age;
}

double getSalary(){
    return salary;
}

void setName(string name){
    this -> name = name;
}

void setAge(int age){
    this -> age = age;
}

void setSalary(double salary){
    this -> salary = salary;
}

};

int main(){
    Employee e;
    e.setAge(10);
    e.setName("Andrey");
    e.setSalary(950.2);

    cout << e.getName() << "\n" << e.getAge() << "\n" << e.getSalary() << endl;
    return 0;
}