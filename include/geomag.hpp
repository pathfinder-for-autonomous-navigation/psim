// geomag2015v2.hpp Generated by python script wmmcodeupdate.py
/** \file
 * \author Nathan Zimmerberg (nhz2@cornell.edu)
 * \date 29 SEP 2019
 * \brief Header only library to calculate the magnetic field in the International Terrestrial Reference System(ITRS).
 * \details Designed to minimize ram usage for embedded systems.

The data from WMM2015, is not subject to copyright protection.
Modifications are:
  using ITRS coordinates,
  conversion from nT to T,
  Using unnormalized coefficents genrated by the python script wmmcofparsing.py
  using a different spherical harmonics calculation, described in sections 3.2.4 and 3.2.5:
    Satellite Orbits Models, Methods and Applications,
      by Oliver Montenbruck and Eberhard Gill 2000
*/
#pragma once

#include <math.h>

#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__
#else
#define CUDA_CALLABLE_MEMBER
#endif

namespace geomag
{
constexpr int NMAX= 12;//order of the Model
constexpr int NUMCOF= (NMAX+1)*(NMAX+2)/2;//number of coefficents
struct ConstModel{
    float epoch;//decimal year
    float Main_Field_Coeff_C[NUMCOF];
    float Main_Field_Coeff_S[NUMCOF];
    float Secular_Var_Coeff_C[NUMCOF];
    float Secular_Var_Coeff_S[NUMCOF];
    /** Function for indexing the C spherical component n,m at dyear time.*/
    CUDA_CALLABLE_MEMBER inline float C(int n, int m, float dyear) const{
      int index= (m*(2*NMAX-m+1))/2+n;
      #ifdef PROGMEM
        return pgm_read_float_near(Main_Field_Coeff_C+index)+(dyear-epoch)*pgm_read_float_near(Secular_Var_Coeff_C+index);
      #endif /* PROGMEM */
      return Main_Field_Coeff_C[index]+(dyear-epoch)*Secular_Var_Coeff_C[index];
    }
    /** Function for indexing the S spherical component n,m at dyear time.*/
    CUDA_CALLABLE_MEMBER inline float S(int n, int m, float dyear) const{
      int index= (m*(2*NMAX-m+1))/2+n;
      #ifdef PROGMEM
        return pgm_read_float_near(Main_Field_Coeff_S+index)+(dyear-epoch)*pgm_read_float_near(Secular_Var_Coeff_S+index);
      #endif /* PROGMEM */
      return Main_Field_Coeff_S[index]+(dyear-epoch)*Secular_Var_Coeff_S[index];
    }
};
//mean radius of  ellipsoid in meters from section 1.2 of the WMM2015 Technical report
constexpr float EARTH_R= 6371200.0f;

typedef struct {
    float x;
    float y;
    float z;
} Vector;

/** Get the magnetic feild in ITRS coordinates, units Tesla, and puts the answer in results.
 INPUT:
    position_itrs(Above the surface of earth): The location where the field is predicted, units m
    dyear(should be around the epoch of the model): The decimal year, for example 2015.0
    results(): Magnetic field, units Tesla, in the ITRS frame, not in north east down.
 */
CUDA_CALLABLE_MEMBER inline Vector GeoMag(float dyear,Vector position_itrs, const ConstModel& WMM){
    float x= position_itrs.x;
    float y= position_itrs.y;
    float z= position_itrs.z;
    float px= 0;
    float py= 0;
    float pz= 0;
    float rsqrd= x*x+y*y+z*z;
    float temp= EARTH_R/rsqrd;
    float a= x*temp;
    float b= y*temp;
    float f= z*temp;
    float g= EARTH_R*temp;

    int n,m;
    //first m==0 row, just solve for the Vs
    float Vtop= EARTH_R/sqrtf(rsqrd);//V0,0
    float Wtop= 0;//W0,0
    float Vprev= 0;
    float Wprev= 0;
    float Vnm= Vtop;
    float Wnm= Wtop;

    //iterate through all ms
    for ( m = 0; m < NMAX+1; m++)
    {
        // iterate through all ns
        for (n = m; n <= NMAX+1; n++)
        {
            if (n==m){
                if(m!=0){
                    temp= Vtop;
                    Vtop= (2*m-1)*(a*Vtop-b*Wtop);
                    Wtop= (2*m-1)*(a*Wtop+b*temp);
                    Vprev= 0;
                    Wprev= 0;
                    Vnm= Vtop;
                    Wnm= Wtop;
                }
            }
            else{
                temp= Vnm;
                float invs_temp=1.0f/((float)(n-m));
                Vnm= ((2*n-1)*f*Vnm - (n+m-1)*g*Vprev)*invs_temp;
                Vprev= temp;
                temp= Wnm;
                Wnm= ((2*n-1)*f*Wnm - (n+m-1)*g*Wprev)*invs_temp;
                Wprev= temp;
            }
            if (m<NMAX && n>=m+2){
                px+= 0.5f*(n-m)*(n-m-1)*(WMM.C(n-1,m+1,dyear)*Vnm+WMM.S(n-1,m+1,dyear)*Wnm);
                py+= 0.5f*(n-m)*(n-m-1)*(-WMM.C(n-1,m+1,dyear)*Wnm+WMM.S(n-1,m+1,dyear)*Vnm);
            }
            if (n>=2 && m>=2){
                px+= 0.5f*(-WMM.C(n-1,m-1,dyear)*Vnm-WMM.S(n-1,m-1,dyear)*Wnm);
                py+= 0.5f*(-WMM.C(n-1,m-1,dyear)*Wnm+WMM.S(n-1,m-1,dyear)*Vnm);
            }
            if (m==1 && n>=2){
                px+= -WMM.C(n-1,0,dyear)*Vnm;
                py+= -WMM.C(n-1,0,dyear)*Wnm;
            }
            if (n>=2 && n>m){
                pz+= (n-m)*(-WMM.C(n-1,m,dyear)*Vnm-WMM.S(n-1,m,dyear)*Wnm);
            }
        }
    }
    return {-px*1.0E-9f,-py*1.0E-9f,-pz*1E-9f};
}
// Model parameters
constexpr
#ifdef PROGMEM
    PROGMEM
#endif /* PROGMEM */
ConstModel WMM = {2015.0,
{0.0,-29438.2,-2444.5,1351.8,907.5,-232.9,69.4,81.7,24.2,5.5,-2.0,3.0,-2.0,-1493.5,1740.5378565259646,-960.0366798548202,257.66238375051955,92.97742019795271,14.773351168976683,-14.343751750700173,1.4833333333333334,1.3118265467998769,-0.8225238322051553,-0.17232808737106584,-0.011322770341445958,484.6855509846908,157.96608074731316,8.780293591649174,9.35400219921169,2.49458699702261,-0.18259229745514208,-0.33665605829585415,0.04767312946227961,0.00259499648053841,-0.024830427232533002,0.0045620741787613245,30.689904691934117,-6.685312021543709,-1.4073816874912486,-0.742395530925816,0.1898495645053134,-0.007601327698513121,-0.0055483358571765265,0.0007633810206905742,0.0020197163363339116,0.0008939803125353484,0.4908937629548134,-0.36905069123145223,-0.029817197315172368,0.008224396186997112,-0.006552733146429182,0.00011779224879003657,-6.426094004278728e-05,-7.023772680455712e-05,-5.587376953345928e-05,0.0057164210075713,0.0030442200863181035,0.0008315778366852636,0.0005838511040628811,-0.00030973506456570993,2.4385312214247102e-05,4.977631011946968e-06,4.7911362107834415e-06,-0.00454257222159809,-5.376455980130303e-05,7.857489491257399e-05,-3.029289764645135e-07,-1.0602514176257971e-06,-5.750020635711086e-07,4.7425370883909984e-08,2.8259393279296416e-05,-2.015824859409769e-05,3.803990742829962e-06,4.040908775664057e-07,8.658648477055405e-09,2.66507576873218e-08,-6.492687430614286e-07,-6.823734684648174e-07,5.998887634855538e-08,1.6884656654872546e-08,-1.7767171791547867e-09,-1.838136788012408e-07,-7.298610579975181e-09,-2.564470354147122e-10,-2.4231967148825077e-10,-3.2640378796247345e-09,7.914127330467462e-11,1.193099586284436e-11,1.4763854141620308e-10,-7.916082160021906e-12,-0.0},
{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,4796.3,-1641.0604051445923,-46.417830625741225,89.5873261125702,12.109527929141855,-4.386179593743447,-10.261735442200518,1.6833333333333331,-3.2497521272996948,0.44497190922573976,-0.0,-0.11322770341445958,-184.40567597916643,31.823013161337606,-14.057414018548679,9.588218216719339,1.1317075173214608,-0.5014858873767987,-0.36454472584699005,0.17003416174879726,-0.00518999296107682,0.022671259647095352,0.0027372445072567945,-28.323466909574783,3.5996301617787485,-1.1942325854932816,0.3398572879761095,0.02182178902359924,0.03261214786781436,0.020459488473338443,0.005852587825294402,-0.0005770618103811176,0.0013409704688030226,-2.3241742005034207,0.037562411321267405,-0.07044837816366428,0.01343318043876195,-0.004590078774068751,-0.0013349788196204146,0.0005654962723765281,-9.657687435626604e-05,-0.00013658032552623381,0.07468466926774969,0.0018131016690571056,0.00031983762949433216,0.0007111569838961407,-0.0001619069655684393,-0.00010702442582919562,5.807236180604796e-06,1.5970454035944803e-06,0.003999789765532316,-0.0004964261021653646,4.064218702374517e-05,2.393138914069657e-05,-9.087869293935405e-07,-1.6428630387745962e-07,3.3197759618736983e-07,-1.3890210255925356e-05,-1.13776617831717e-05,4.3724031526781175e-07,-7.714462208085927e-07,-1.818316180181635e-07,-4.441792947886967e-09,7.420214206416327e-07,-2.9244577219920745e-07,-7.248655892117107e-08,-1.4898226460181658e-08,1.33253788436609e-09,1.502323336356295e-07,-4.460262021095944e-09,-3.3338114603912585e-09,9.692786859530031e-11,-7.978759261304907e-09,-3.957063665233731e-10,-5.3689481382799615e-11,-9.701961293064772e-11,-1.759129368893757e-12,1.436323115111301e-12},
{0.0,7.0,-11.0,2.4,-0.8,-0.3,-0.8,-0.3,-0.1,-0.1,0.0,-0.0,0.0,9.0,-3.57957166897568,-2.327015255644019,-0.28460498941515416,0.15491933384829665,-0.10910894511799618,-0.03779644730092272,0.03333333333333333,-0.0149071198499986,-0.0,0.0,0.0,0.08660254037844385,0.2581988897471611,-0.4844813951249545,-0.039036002917941334,-0.003450327796711771,-0.007715167498104595,-0.003984095364447979,-0.0,-0.001297498240269205,-0.0,-0.0,-0.5797509043642028,0.10358647947564745,0.0009960238411119947,0.009200874124564723,0.003273268353539886,0.0012260205965343744,0.0006935419821470658,0.0002544603402301914,0.0,0.0,-0.028171808490950554,0.0028171808490950554,-0.0016798421022632321,5.482930791331409e-05,-3.1655715683232764e-05,-7.85281658600244e-05,-1.2852188008557456e-05,-0.0,-6.208196614828809e-06,0.0010393492741038726,0.0,-5.482930791331408e-05,1.755943170113928e-05,0.0,-2.7094791349163448e-06,-8.296051686578281e-07,-0.0,7.754035086653923e-05,-1.612936794039091e-05,2.7094791349163448e-06,9.087869293935405e-07,-0.0,0.0,0.0,3.352809372119913e-06,-1.2367023677360544e-07,0.0,-1.836776716210935e-08,-0.0,-0.0,1.2367023677360544e-07,-0.0,-4.999073029046282e-09,-0.0,0.0,-5.302317657728099e-09,-4.054783655541767e-10,-1.282235177073561e-10,-0.0,-0.0,-0.0,-0.0,-4.218244040462945e-12,-0.0,-1.7954038938891263e-13},
{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-30.2,-17.089567968012922,2.6536138880151094,-0.1264911064067352,0.051639777949432225,0.06546536707079771,0.11338934190276816,-0.06666666666666667,-0.044721359549995794,0.0,0.0,-0.0,-4.994079828490263,-0.10327955589886445,0.43230647564995933,0.11222850838908131,-0.05175491695067656,0.012858612496840992,0.011952286093343936,0.0015891043154093204,0.001297498240269205,0.0010795837927188264,0.0,-0.10540925533894598,0.0756978119245116,-0.0,-0.006900655593423542,-0.002909571869813232,-0.0002452041193068749,-0.0006935419821470658,-0.0002544603402301914,0.0,-7.44983593779457e-05,-0.024650332429581735,0.007747247335011402,0.00041996052556580803,-0.00010965861582662818,0.00018993429409939658,5.8896124395018286e-05,1.2852188008557456e-05,8.77971585056964e-06,6.208196614828809e-06,-0.00044543540318737394,4.4767942445854464e-05,-0.00010052039784107583,-8.77971585056964e-06,2.346477761861439e-06,-1.3547395674581724e-06,-0.0,-0.0,8.400204677208417e-05,1.7921519933767679e-06,-3.3868489186454308e-06,-0.0,1.5146448823225676e-07,-0.0,0.0,9.579455348914039e-07,6.183511838680272e-07,-4.372403152678118e-08,-0.0,8.658648477055405e-09,-0.0,3.091755919340136e-08,3.749304771784711e-08,-2.499536514523141e-09,-0.0,0.0,3.5348784384854e-09,8.109567311083534e-10,-1.282235177073561e-10,0.0,-0.0,-0.0,-0.0,-4.218244040462945e-12,0.0,-1.7954038938891263e-13}};

}
