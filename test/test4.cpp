#include <iostream>

using namespace std;

class reference{
public:
  reference(int &a): _a(a){};
  ~reference(){};

  void print(){
    cout << _a << "\n";
    _a = _a+1;
  }
private:
  int _a;
};


int main()
{
  int k = 5;
  reference r1(k);
  reference r2(k);
  r1.print();
  r2.print();

}