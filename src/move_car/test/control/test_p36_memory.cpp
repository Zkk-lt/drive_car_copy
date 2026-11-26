#include <cstdlib>
#include <new>
#include <cstdio>
#include "control/flatness_reduced_nmpc.h"
static bool counting=false;static size_t allocations=0,bytes=0;
void* operator new(std::size_t n){if(counting){++allocations;bytes+=n;}if(void* p=std::malloc(n))return p;throw std::bad_alloc();}
void operator delete(void* p) noexcept{std::free(p);}
void operator delete(void* p,std::size_t) noexcept{std::free(p);}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void* p) noexcept{::operator delete(p);}
void operator delete[](void* p,std::size_t) noexcept{::operator delete(p);}
int main(){
 using namespace research_control;
 SolveRequest r;r.now=10.;r.state.x=-.3;r.state.speed=.24;r.previous.speed=r.baseline.speed=.24;r.history.push_back({9.,r.previous});
 r.reference.valid=true;r.reference.count=21;
 for(int i=0;i<21;++i){auto& p=r.reference.points[i];p.t=i*.1;p.x=p.s=.24*p.t;p.speed=.24;}
 SolverConfig c;c.budget_ms=1000.;ReducedNMPC original;FlatnessReducedNMPC flat;
 for(int worst=0;worst<2;++worst){
 if(worst){c.steps=40;c.dt=.25;c.iterations=30;c.model.steering_delay=c.model.speed_delay=2.;
  r.reference.count=41;for(int i=0;i<41;++i){auto& p=r.reference.points[i];p.t=i*.25;p.x=p.s=.24*p.t;p.speed=.24;}
  r.history.clear();for(int i=0;i<512;++i)r.history.push_back({7.+i*.005,r.previous});
 }
 for(int backend=0;backend<3;++backend){allocations=bytes=0;counting=true;
  SolveResult out;if(backend==2)out=flat.solve(r,c);else{c.knots=backend==0?6:4;out=original.solve(r,c);}
  counting=false;if(!out.ok)return 1;
  std::printf("%s case=%s allocations=%zu bytes=%zu sizeof_solver=%zu\n",backend==0?"original12":backend==1?"blocked8":"flat6",worst?"max_allowed_horizon_history_iterations":"default",allocations,bytes,backend==2?sizeof(flat):sizeof(original));
 }
 }
}
