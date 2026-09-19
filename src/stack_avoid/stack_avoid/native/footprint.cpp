// Same point/edge slab predicates as station_path. No fast-math or FMA.
#include <cmath>
#include <algorithm>
#include <cstdint>
extern "C" bool footprint(const double* poses,const double* cs,const double* ss,std::int64_t np,
 const double* edges,std::int64_t ne,double width,double front,double rear,double margin){
 for(std::int64_t i=0;i<np;++i){
  for(std::int64_t j=0;j<ne;++j){
   const double* e=edges+j*4;
   double x0=e[0]-poses[i*4],y0=e[1]-poses[i*4+1];
   double x1=e[2]-poses[i*4],y1=e[3]-poses[i*4+1];
   double xy[2][2]={{cs[i]*x0+ss[i]*y0,cs[i]*x1+ss[i]*y1},
                    {-ss[i]*x0+cs[i]*y0,-ss[i]*x1+cs[i]*y1}};
   double lows[2]={-rear-margin,-width/2-margin};
   double highs[2]={front+margin,width/2+margin};
   double near=0.,far=1.;bool possible=true;
   for(int axis=0;axis<2;++axis){
    double a=xy[axis][0],delta=xy[axis][1]-a;
    bool parallel=std::abs(delta)<1e-12;
    possible &= !parallel || (a>=lows[axis]&&a<=highs[axis]);
    double denom=parallel?1.:delta;
    double t0=(lows[axis]-a)/denom,t1=(highs[axis]-a)/denom;
    if(!parallel){near=std::max(near,std::min(t0,t1));far=std::min(far,std::max(t0,t1));}
   }
   if(possible&&near<=far)return false;
  }
 }
 return true;
}
