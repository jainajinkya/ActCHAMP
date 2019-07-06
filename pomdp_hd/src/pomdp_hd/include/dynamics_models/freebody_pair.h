#ifndef FREEBODY_PAIR_H_
#define FREEBODY_PAIR_H_

#include "linear_dynamics.h"

namespace dynamics{

class FreebodyPair: public LinearDynamics{
public:  
    FreebodyPair(int nx, int nu, int nz) : 
            LinearDynamics(nx, nu, nz) {};
   ~FreebodyPair(){}
};
}
#endif // FREEBODY_PAIR_H_
