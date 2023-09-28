#define DYNEventSpecial
/* DSblock model generated by Dymola from Modelica model FM3217_2023.Tutorial4.ElectricKettle
 Dymola Version 2020 (64-bit), 2019-04-10 translated this at Thu Sep 28 23:27:38 2023

   */

#include <matrixop.h>
/* Declaration of C-structs */
/* Prototypes for functions used in model */
/* Codes used in model */
/* DSblock C-code: */

#define NX_    2
#define NX2_   0
#define NU_    0
#define NY_    0
#define NW_    40
#define NP_    18
#define NPS_   0
#define ND_   0
#define NXP_   0
#define NInitial_   0
#define MAXAuxStr_   0
#define MAXAuxStrLen_   500
#define NHash1_ -399685096
#define NHash2_ -1145799607
#define NHash3_ 0
#define NI_    0
#define NRelF_ 3
#define NRel_  3
#define NTim_  1
#define NSamp_ 1
#define NCons_ 0
#define NA_    47
#define SizePre_ 3
#define SizeEq_ 2
#define SizeDelay_ 0
#define QNLmax_ 0
#define MAXAux 3
#define NrDymolaTimers_ 0
#define NWhen_ 0
#define NCheckIf_ 0
#define NGlobalHelp_ 4
#define NGlobalHelpI_ 0
#ifndef NExternalObject_
#define NExternalObject_ 0
#endif
#include <moutil.c>
DYMOLA_STATIC unsigned int FMIClockValueReferences_[1]={0};
DYMOLA_STATIC unsigned int FMIClockFirstValueReferences_[1]={0};
PreNonAliasDef(0)
PreNonAliasDef(1)
PreNonAliasDef(2)
PreNonAliasDef(3)
PreNonAliasDef(4)
PreNonAliasDef(5)
#if !defined(DYM2CCUR)
 DYMOLA_STATIC const char*modelName="FM3217_2023.Tutorial4.ElectricKettle";
#endif
DYMOLA_STATIC const char*usedLibraries[]={0};
DYMOLA_STATIC const char*dllLibraryPath[]={0};
DYMOLA_STATIC const char*default_dymosim_license_filename=
 "c:/users/arabd/appdata/roaming/dassaultsystemes/dymola/dymola.lic";
#define QJacobianCGDef_ 1
DYMOLA_STATIC int QJacobianCG_[6]={1 , 2 , 1 , 2 , 0 , 0};
DYMOLA_STATIC int QJacobianGC_[3]={
2 , 2 , 0};
DYMOLA_STATIC double QJacobianCD_[3]={0  , 44 , 55};
#include <dsblock1.c>

/* Define variable names. */

#define Sections_

TranslatedEquations

InitialSection
#if defined(DynSimStruct) || defined(BUILDFMU)
DYNX(W_,3) = true;
DYNX(W_,29) = false;
DYNX(W_,35) = 1;
DYNX(W_,36) = 1;
DYNX(W_,22) = false;
DYNX(W_,30) = 293.15;
DYNX(W_,16) = 0;
DYNX(W_,2) = 0;
DYNX(W_,27) = 0;
DYNX(W_,18) = 0;
DYNX(W_,14) = 0.0;
DYNX(W_,17) = 0.0;
DYNX(W_,21) = 0.0;
DYNX(W_,20) = 0.0;
DYNX(W_,19) = 0.0;
DYNX(W_,8) = 0.0;
DYNX(W_,32) = 293.15;
DYNX(W_,15) = 0.0;
#endif
if (!DymolaUserHomotopy) UpdateInitVars(time, X_, XD_, U_, DP_, IP_, LP_, F_, Y_, W_, QZ_, duser_, iuser_, cuser_, did_, 1);
BoundParameterSection
DYNX(W_,4) = DYNX(DP_,1);
DYNX(W_,9) = DYNX(DP_,3);
DYNX(W_,10) = DYNX(DP_,5);
DYNX(W_,11) = DYNX(DP_,4);
DYNX(W_,12) = DYNX(DP_,6);
DYNX(W_,13) = DYNX(DP_,7);
InitialSection
#if defined(DynSimStruct) || defined(BUILDFMU)
DYNX(F_,1) = 0;
#endif
InitialSection
InitialStartSection
InitialSection
if (!DymolaUserHomotopy) UpdateInitVars(time, X_, XD_, U_, DP_, IP_, LP_, F_, Y_, W_, QZ_, duser_, iuser_, cuser_, did_, 1);
DefaultSection
InitializeData(0)
InitialBoundSection
DYNX(Aux_,0) = DYNX(X_,0);
DYNX(Aux_,1) = DYNX(W_,24);
DYNX(Aux_,2) = DYNX(W_,39);
InitialSection
InitialSection2
DYNX(W_,24) = 0;
DYNX(W_,15) = 0.0;
InitialBoundSection
DYNX(Aux_,1) = 0;
InitialSection2
DYNX(W_,4) = DYNX(DP_,1);
DYNX(W_,9) = DYNX(DP_,3);
DYNX(W_,10) = DYNX(DP_,5);
DYNX(W_,11) = DYNX(DP_,4);
DYNX(W_,12) = DYNX(DP_,6);
DYNX(W_,13) = DYNX(DP_,7);
DYNX(W_,23) = DYNTime;
DYNX(X_,0) = DYNX(DP_,9);
InitialBoundSection
DYNX(Aux_,0) = DYNX(X_,0);
DYNX(Aux_,2) = DYNX(DP_,16);
InitialSection
InitialSection2
DYNX(W_,39) = DYNX(Aux_,2);
InitialSection
InitialSectionB
Init_=false;InitializeData(2);Init_=true;
EndInitialSection

OutputSection
AssertModelica(GreaterEqual(1+DYNX(DP_,2)*(DYNX(X_,1)-DYNX(DP_,1)),
  "1+resistor.alpha*(water.T-resistor.T_ref)", 1E-15,"1E-15", 0),
  "1+resistor.alpha*(water.T-resistor.T_ref) >= 1E-15", "Temperature outside scope of model!");

DynamicsSection
DYNX(W_,38) = DYNX(X_,1)-DYNX(DP_,14);
DYNX(W_,37) = DYNX(DP_,13)*DYNX(W_,38);
DYNX(W_,6) = DYNX(DP_,0)*(1+DYNX(DP_,2)*(DYNX(X_,1)-DYNX(DP_,1)));
DYNX(W_,7) = DYNX(W_,12)+(IF LessTime(DYNX(W_,13), 0) THEN 0 ELSE DYNX(W_,9)*sin
  (6.283185307179586*DYNX(W_,10)*(DYNTime-DYNX(W_,13))+DYNX(W_,11)));
DYNX(W_,26) = DYNX(X_,1)-273.15;
DYNX(W_,39) = PRE(DYNX(W_,39), 1) AND Less(DYNX(W_,26),"temperatureSensor.T", 
  DYNX(DP_,17)+DYNX(DP_,15)/(double)(2),"const.k+onOffController.bandwidth/2", 1)
   OR Less(DYNX(W_,26),"temperatureSensor.T", DYNX(DP_,17)-DYNX(DP_,15)/
  (double)(2),"const.k-onOffController.bandwidth/2", 2);
DYNX(W_,33) =  NOT DYNX(W_,39);

 /* Linear system of equations to solve. */
/* Tag: simulation.linear[1] */
/* Introducing 2 common subexpressions used in 2 expressions */
/* Of the common subexpressions 2 are reals, 0 are integers, and 0
   are booleans. */
DYNX(DYNhelp,0) = IF DYNX(W_,33) THEN 1 ELSE DYNX(DP_,11);
DYNX(DYNhelp,1) = IF DYNX(W_,33) THEN DYNX(DP_,12) ELSE 1;
DYNX(W_,34) = RememberSimple_(DYNX(W_,34), 0);
SolveScalarLinear(DYNX(DYNhelp,0)+DYNX(W_,6)*DYNX(DYNhelp,1),"(if switch.off then 1 else switch.Ron)+resistor.R_actual*(if switch.off then switch.Goff else 1)",
   DYNX(W_,7),"sineVoltage.v", DYNX(W_,34),"switch.s");
DYNX(W_,28) = DYNX(W_,34)*DYNX(DYNhelp,0);
DYNX(W_,1) = DYNX(W_,34)*DYNX(DYNhelp,1);
DYNX(W_,0) = DYNX(W_,6)*DYNX(W_,1);
 /* End of Equation Block */ 

DYNX(W_,5) = DYNX(W_,0)*DYNX(W_,1);
DYNX(W_,25) = DYNX(W_,5)-DYNX(W_,37);
 /* Linear system of equations to solve. */
DYNX(F_,1) = RememberSimple_(DYNX(F_,1), 1);
SolveScalarLinearParametric(DYNX(DP_,10),"water.C", DYNX(W_,25),"water.port.Q_flow",
   DYNX(F_,1),"der(water.T)");
 /* End of Equation Block */ 

DYNX(F_,0) = DYNX(W_,0)*DYNX(W_,1);

AcceptedSection1

AcceptedSection2
DYNX(W_,31) = DYNX(W_,28)*DYNX(W_,1);
beginwhenBlock
if (NewParameters_) {
DYNX(DYNhelp,2) = divinvGuarded(DYNX(DP_,8),"mean.f");
}
DYNX(DYNhelp,3) = sampleNew(DYNX(W_,23)+DYNX(DYNhelp,2), DYNX(DYNhelp,2), 0);
whenModelicaS(DYNX(DYNhelp,3))
  DYNX(W_,24) = DYNX(DP_,8)*PRE(DYNX(X_,0), 2);
endwhenModelica()
endwhenBlock



DefaultSection
CrossingSection
/* Start of reinit equations */
beginwhenBlock
whenModelicaS(DYNX(DYNhelp,3))
  reinit(DYNX(X_,0), 0);
endwhenModelica()
endwhenBlock


/* End of reinit equations */
DefaultSection
InitializeData(1)
EndTranslatedEquations

#include <dsblock6.c>

PreNonAliasNew(0)
StartNonAlias(0)
DeclareParameter("resistor.R", "Resistance at temperature T_ref [Ohm]", 0, 26.45,\
 0.0,0.0,0.0,0,560)
DeclareParameter("resistor.T_ref", "Reference temperature [K|degC]", 1, 300.15, \
0.0,1E+100,300.0,0,560)
DeclareParameter("resistor.alpha", "Temperature coefficient of resistance (R_actual = R*(1 + alpha*(T_heatPort - T_ref)) [1/K]",\
 2, 0, 0.0,0.0,0.0,0,560)
DeclareVariable("resistor.v", "Voltage drop of the two pins (= p.v - n.v) [V]", \
0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("resistor.p.v", "Potential at the pin [V]", "resistor.v", 1, 5, 0,\
 4)
DeclareVariable("resistor.p.i", "Current flowing into the pin [A]", 0.0, \
0.0,0.0,0.0,0,776)
DeclareVariable("resistor.n.v", "Potential at the pin [V]", 0, 0.0,0.0,0.0,0,521)
DeclareAlias2("resistor.n.i", "Current flowing into the pin [A]", "resistor.p.i", -1,\
 5, 1, 132)
DeclareAlias2("resistor.i", "Current flowing from pin p to pin n [A]", \
"resistor.p.i", 1, 5, 1, 0)
DeclareVariable("resistor.useHeatPort", "= true, if heatPort is enabled [:#(type=Boolean)]",\
 true, 0.0,0.0,0.0,0,1539)
DeclareVariable("resistor.T", "Fixed device temperature if useHeatPort = false [K|degC]",\
 288.15, 0.0,1E+100,300.0,0,513)
DeclareAlias2("resistor.heatPort.T", "Port temperature [K|degC]", "water.T", 1, 1,\
 1, 4)
DeclareAlias2("resistor.heatPort.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 "resistor.LossPower", -1, 5, 5, 132)
DeclareVariable("resistor.LossPower", "Loss power leaving component via heatPort [W]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("resistor.T_heatPort", "Temperature of heatPort [K|degC]", \
"water.T", 1, 1, 1, 0)
DeclareVariable("resistor.R_actual", "Actual resistance = R*(1 + alpha*(T_heatPort - T_ref)) [Ohm]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareParameter("sineVoltage.V", "Amplitude of sine wave [V]", 3, \
325.2691193458119, 0.0,0.0,0.0,0,560)
DeclareParameter("sineVoltage.phase", "Phase of sine wave [rad|deg]", 4, 0, \
0.0,0.0,0.0,0,560)
DeclareParameter("sineVoltage.f", "Frequency of sine wave [Hz]", 5, 50, 0.0,0.0,\
0.0,0,560)
DeclareVariable("sineVoltage.v", "Voltage drop of the two pins (= p.v - n.v) [V]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("sineVoltage.p.v", "Potential at the pin [V]", "sineVoltage.v", 1,\
 5, 7, 4)
DeclareAlias2("sineVoltage.p.i", "Current flowing into the pin [A]", \
"resistor.p.i", -1, 5, 1, 132)
DeclareVariable("sineVoltage.n.v", "Potential at the pin [V]", 0.0, 0.0,0.0,0.0,\
0,521)
DeclareAlias2("sineVoltage.n.i", "Current flowing into the pin [A]", \
"resistor.p.i", 1, 5, 1, 132)
DeclareAlias2("sineVoltage.i", "Current flowing from pin p to pin n [A]", \
"resistor.p.i", -1, 5, 1, 0)
DeclareVariable("sineVoltage.signalSource.amplitude", "Amplitude of sine wave [V]",\
 0.0, 0.0,0.0,0.0,0,513)
DeclareVariable("sineVoltage.signalSource.f", "Frequency of sine wave [Hz]", 1, \
0.0,0.0,0.0,0,513)
DeclareVariable("sineVoltage.signalSource.phase", "Phase of sine wave [rad|deg]",\
 0.0, 0.0,0.0,0.0,0,513)
DeclareAlias2("sineVoltage.signalSource.y", "Connector of Real output signal [V]",\
 "sineVoltage.v", 1, 5, 7, 0)
DeclareVariable("sineVoltage.signalSource.offset", "Offset of output signal y [V]",\
 0.0, 0.0,0.0,0.0,0,513)
DeclareVariable("sineVoltage.signalSource.startTime", "Output y = offset for time < startTime [s]",\
 0.0, 0.0,0.0,0.0,0,513)
DeclareParameter("sineVoltage.offset", "Voltage offset [V]", 6, 0, 0.0,0.0,0.0,0,560)
DeclareParameter("sineVoltage.startTime", "Time offset [s]", 7, 0, 0.0,0.0,0.0,0,560)
DeclareVariable("ground1.p.v", "Potential at the pin [V]", 0.0, 0.0,0.0,0.0,0,521)
DeclareVariable("ground1.p.i", "Current flowing into the pin [A]", 0.0, 0.0,0.0,\
0.0,0,777)
DeclareAlias2("powerSensor.pc.v", "Potential at the pin [V]", "resistor.v", 1, 5,\
 0, 4)
DeclareAlias2("powerSensor.pc.i", "Current flowing into the pin [A]", \
"resistor.p.i", 1, 5, 1, 132)
DeclareAlias2("powerSensor.nc.v", "Potential at the pin [V]", "resistor.v", 1, 5,\
 0, 4)
DeclareAlias2("powerSensor.nc.i", "Current flowing into the pin [A]", \
"resistor.p.i", -1, 5, 1, 132)
DeclareAlias2("powerSensor.pv.v", "Potential at the pin [V]", "resistor.v", 1, 5,\
 0, 4)
DeclareVariable("powerSensor.pv.i", "Current flowing into the pin [A]", 0, \
0.0,0.0,0.0,0,777)
DeclareVariable("powerSensor.nv.v", "Potential at the pin [V]", 0.0, 0.0,0.0,0.0,\
0,521)
DeclareVariable("powerSensor.nv.i", "Current flowing into the pin [A]", 0, \
0.0,0.0,0.0,0,777)
DeclareAlias2("powerSensor.power", "Instantaneous power as output signal [W]", \
"mean.der(x)", 1, 6, 0, 0)
DeclareAlias2("powerSensor.voltageSensor.p.v", "Potential at the pin [V]", \
"resistor.v", 1, 5, 0, 4)
DeclareVariable("powerSensor.voltageSensor.p.i", "Current flowing into the pin [A]",\
 0.0, 0.0,0.0,0.0,0,777)
DeclareVariable("powerSensor.voltageSensor.n.v", "Potential at the pin [V]", 0.0,\
 0.0,0.0,0.0,0,521)
DeclareVariable("powerSensor.voltageSensor.n.i", "Current flowing into the pin [A]",\
 0.0, 0.0,0.0,0.0,0,777)
DeclareAlias2("powerSensor.voltageSensor.v", "Voltage between pin p and n (= p.v - n.v) as output signal [V]",\
 "resistor.v", 1, 5, 0, 0)
DeclareAlias2("powerSensor.currentSensor.p.v", "Potential at the pin [V]", \
"resistor.v", 1, 5, 0, 4)
DeclareAlias2("powerSensor.currentSensor.p.i", "Current flowing into the pin [A]",\
 "resistor.p.i", 1, 5, 1, 132)
DeclareAlias2("powerSensor.currentSensor.n.v", "Potential at the pin [V]", \
"resistor.v", 1, 5, 0, 4)
DeclareAlias2("powerSensor.currentSensor.n.i", "Current flowing into the pin [A]",\
 "resistor.p.i", -1, 5, 1, 132)
DeclareAlias2("powerSensor.currentSensor.i", "Current in the branch from p to n as output signal [A]",\
 "resistor.p.i", 1, 5, 1, 0)
DeclareAlias2("powerSensor.product.u1", "Connector of Real input signal 1 [V]", \
"resistor.v", 1, 5, 0, 0)
DeclareAlias2("powerSensor.product.u2", "Connector of Real input signal 2 [A]", \
"resistor.p.i", 1, 5, 1, 0)
DeclareAlias2("powerSensor.product.y", "Connector of Real output signal [W]", \
"mean.der(x)", 1, 6, 0, 0)
DeclareAlias2("mean.u", "Connector of Real input signal [W]", "mean.der(x)", 1, 6,\
 0, 0)
DeclareAlias2("mean.y", "Connector of Real output signal [W]", "mean.y_last", 1,\
 5, 24, 64)
DeclareParameter("mean.f", "Base frequency [Hz]", 8, 50, 0.0,0.0,0.0,0,560)
DeclareParameter("mean.x0", "Start value of integrator state [J]", 9, 0, \
0.0,0.0,0.0,0,560)
DeclareVariable("mean.yGreaterOrEqualZero", "= true, if output y is guaranteed to be >= 0 for the exact solution [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,515)
DeclareVariable("mean.t0", "Start time of simulation [s]", 0.0, 0.0,0.0,0.0,0,2561)
DeclareState("mean.x", "Integrator state [J]", 0, 0.0, 0.0,0.0,0.0,0,2592)
DeclareDerivative("mean.der(x)", "der(Integrator state) [W]", 0.0, 0.0,0.0,0.0,0,2560)
DeclareVariable("mean.y_last", "Last sampled mean value [W]", 0.0, 0.0,0.0,0.0,0,2688)
DeclareParameter("water.C", "Heat capacity of element (= cp*m) [J/K]", 10, \
7106.0, 0.0,0.0,0.0,0,560)
DeclareState("water.T", "Temperature of element [K|degC]", 1, 283.15, 0.0,1E+100,\
300.0,0,560)
DeclareDerivative("water.der(T)", "der(Temperature of element) [K/s]", 0, \
0.0,0.0,0.0,0,512)
DeclareAlias2("water.der_T", "Time derivative of temperature (= der(T)) [K/s]", \
"water.der(T)", 1, 6, 1, 0)
DeclareAlias2("water.port.T", "Port temperature [K|degC]", "water.T", 1, 1, 1, 4)
DeclareVariable("water.port.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 0.0, 0.0,0.0,0.0,0,776)
DeclareVariable("temperatureSensor.T", "Absolute temperature in degree Celsius as output signal [degC]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("temperatureSensor.port.T", "Port temperature [K|degC]", "water.T", 1,\
 1, 1, 4)
DeclareVariable("temperatureSensor.port.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 0, 0.0,0.0,0.0,0,777)
DeclareVariable("switch.v", "Voltage drop of the two pins (= p.v - n.v) [V]", \
0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("switch.p.v", "Potential at the pin [V]", "sineVoltage.v", 1, 5, 7,\
 4)
DeclareAlias2("switch.p.i", "Current flowing into the pin [A]", "resistor.p.i", 1,\
 5, 1, 132)
DeclareAlias2("switch.n.v", "Potential at the pin [V]", "resistor.v", 1, 5, 0, 4)
DeclareAlias2("switch.n.i", "Current flowing into the pin [A]", "resistor.p.i", -1,\
 5, 1, 132)
DeclareAlias2("switch.i", "Current flowing from pin p to pin n [A]", \
"resistor.p.i", 1, 5, 1, 0)
DeclareParameter("switch.Ron", "Closed switch resistance [Ohm]", 11, 1E-05, 0.0,\
1E+100,0.0,0,560)
DeclareParameter("switch.Goff", "Opened switch conductance [S]", 12, 1E-05, 0.0,\
1E+100,0.0,0,560)
DeclareVariable("switch.useHeatPort", "= true, if heatPort is enabled [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,1539)
DeclareVariable("switch.T", "Fixed device temperature if useHeatPort = false [K|degC]",\
 293.15, 0.0,1E+100,300.0,0,513)
DeclareVariable("switch.LossPower", "Loss power leaving component via heatPort [W]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("switch.T_heatPort", "Temperature of heatPort [K|degC]", 293.15,\
 0.0,1E+100,300.0,0,513)
DeclareVariable("switch.off", "Indicates off-state [:#(type=Boolean)]", false, \
0.0,0.0,0.0,0,2690)
DeclareVariable("switch.s", "Auxiliary variable [1]", 0.0, 0.0,0.0,0.0,0,2560)
DeclareVariable("switch.unitVoltage", "[V]", 1, 0.0,0.0,0.0,0,1537)
DeclareVariable("switch.unitCurrent", "[A]", 1, 0.0,0.0,0.0,0,1537)
DeclareAlias2("switch.control", "true => p--n connected, false => switch open [:#(type=Boolean)]",\
 "onOffController.y", 1, 5, 39, 65)
DeclareVariable("kettleWall.Q_flow", "Heat flow rate from port_a -> port_b [W]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("kettleWall.dT", "port_a.T - port_b.T [K,]", 0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("kettleWall.port_a.T", "Port temperature [K|degC]", "water.T", 1, 1,\
 1, 4)
DeclareAlias2("kettleWall.port_a.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 "kettleWall.Q_flow", 1, 5, 37, 132)
DeclareAlias2("kettleWall.port_b.T", "Port temperature [K|degC]", \
"roomTemperature.T", 1, 7, 14, 4)
DeclareAlias2("kettleWall.port_b.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 "kettleWall.Q_flow", -1, 5, 37, 132)
DeclareParameter("kettleWall.G", "Constant thermal conductance of material [W/K]",\
 13, 5, 0.0,0.0,0.0,0,560)
DeclareParameter("roomTemperature.T", "Fixed temperature at port [K|degC]", 14, \
294.15, 0.0,1E+100,300.0,0,560)
DeclareAlias2("roomTemperature.port.T", "Port temperature [K|degC]", \
"roomTemperature.T", 1, 7, 14, 4)
DeclareAlias2("roomTemperature.port.Q_flow", "Heat flow rate (positive if flowing from outside into the component) [W]",\
 "kettleWall.Q_flow", 1, 5, 37, 132)
DeclareAlias2("onOffController.reference", "Connector of Real input signal used as reference signal",\
 "const.k", 1, 7, 17, 0)
DeclareAlias2("onOffController.u", "Connector of Real input signal used as measurement signal [degC]",\
 "temperatureSensor.T", 1, 5, 26, 0)
DeclareVariable("onOffController.y", "Connector of Real output signal used as actuator signal [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,642)
DeclareParameter("onOffController.bandwidth", "Bandwidth around reference signal",\
 15, 3, 0.0,0.0,0.0,0,560)
DeclareParameter("onOffController.pre_y_start", "Value of pre(y) at initial time [:#(type=Boolean)]",\
 16, false, 0.0,0.0,0.0,0,562)
DeclareParameter("const.k", "Constant output value", 17, 95, 0.0,0.0,0.0,0,560)
DeclareAlias2("const.y", "Connector of Real output signal", "const.k", 1, 7, 17,\
 0)
EndNonAlias(0)

#define DymolaHaveUpdateInitVars 1
#include <dsblock5.c>

DYMOLA_STATIC void UpdateInitVars(double*time, double* X_, double* XD_, double* U_, double* DP_, int IP_[], Dymola_bool LP_[], double* F_, double* Y_, double* W_, double QZ_[], double duser_[], int iuser_[], void*cuser_[],struct DYNInstanceData*did_,int initialCall) {
}
StartDataBlock
StartPreBlock
preCont(DYNX(X_,0),"mean.x", 0.0, 2);
pre(DYNX(W_,39),"onOffController.y", false, 1);
preWD(DYNX(W_,33),"switch.off", false, 0);
EndPreBlock
StartEqBlock
DoRemember_(DYNX(F_,1), 0, 1);
DoRemember_(DYNX(W_,34), 0.0, 0);
EndEqBlock
UpdateSampleCounters(1)
EndDataBlock