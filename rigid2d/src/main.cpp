#include <iostream>
#include "rigid2d/rigid2d.hpp"

using std::cout, std::cin;

int main(void)
{
  cout << "Enter two transforms T_ab and Tbc\n";

  rigid2d::Transform2D tab, tbc;
  cin >> tab;
  cin >> tbc;

  cout << "T_ab: " << tab << "\n";
  auto tba = tab.inv();
  cout << "T_ba: " << tba << "\n";

  cout << "T_bc: " << tbc << "\n";
  auto tcb = tbc.inv();
  cout << "T_cb: " << tcb << "\n";

  auto tac = tab * tbc;
  cout << "T_ac: " << tac << "\n";
  auto tca = tac.inv();
  cout << "T_ca: " << tca << "\n";

  cout << "Enter a vector v: \n";
  rigid2d::Vector2D v;
  cin >> v;

  cout << "Enter the vector base (either 'a', 'b' or 'c'):\n";
  char base;
  cin >> base;

  rigid2d::Vector2D va, vb, vc;
  
  switch(base)
  {
    case 'a':
      va = v;
      vb = tba(va);
      vc = tca(va);
      break;
    case 'b':
      vb = v;
      va = tab(vb);
      vc = tcb(vb);
      break;
    case 'c':
      vc = v;
      va = tac(vc);
      vc = tbc(vc);
      break;
  }

  std::cout << "va: " << va << "\n";
  std::cout << "vb: " << vb << "\n";
  std::cout << "vc: " << vc << "\n";
}







