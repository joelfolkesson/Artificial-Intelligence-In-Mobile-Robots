//=============================================================================
//fuzzy.c
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "fuzzy.h"
//=============================================================================


//=============================================================================
//Global variables
//=============================================================================
//Fuzzy predicate
FPred ante;

//Control values
double f_vlin;
double f_vrot;

//Fuzzy sets
FSet f_set_vlin;
FSet f_set_vrot;
//=============================================================================


//=============================================================================
//Functions
//=============================================================================

//=============================================================================
//Ramp functions
//=============================================================================
FPred RampUp(double Value, double RampStart, double RampEnd)
{
  if (Value > RampEnd) return(1.0);
  if (Value < RampStart) return(0.0);
  if (RampStart == RampEnd) return(0.0);
  return((Value - RampStart) / (RampEnd - RampStart));
}
//=============================================================================
FPred RampDown(double Value, double RampStart, double RampEnd)
{
  if (Value < RampStart) return(1.0);
  if (Value > RampEnd) return(0.0);
  if (RampStart == RampEnd) return(0.0);
  return((RampEnd - Value) / (RampEnd - RampStart));
}
//=============================================================================


//=============================================================================
//Possibilistic logical functions
//=============================================================================
FPred AND (FPred a, FPred b)
{
	return MIN(a, b);
}
//=============================================================================
FPred OR  (FPred a, FPred b)
{
	return MAX(a, b);
}
//=============================================================================
FPred NOT (FPred a)
{
	return (1.0 - a);
}
//=============================================================================
void IF(FPred a)
{
	ante = a;
}
//=============================================================================


//=============================================================================
//Control values
//=============================================================================
void ROT (int rot)
{
  //Set rotation speed
  f_set_vrot[rot] = MAX(f_set_vrot[rot], ante);
}
//=============================================================================
void VEL (int vel)
{
  //Set linear speed
  f_set_vlin[vel] = MAX(f_set_vlin[vel], ante);
}
//=============================================================================


//=============================================================================
//Behaviour functions
//=============================================================================
void RunBehaviour(FBehaviour behaviour)
{
  //Run behaviour, defuzzify

  //Reset Fuzzy sets
  ClearFSet(f_set_vlin);
  ClearFSet(f_set_vrot);

	//Run the behaviour
  behaviour();

	//Defuzzify and put in f_v_rot and f_v_lin
  DeFuzzify(f_set_vlin, 4, &f_vlin);
  DeFuzzify(f_set_vrot, 3, &f_vrot);
}
//=============================================================================
void SetFSet(FSet fset)
{
  int i;
  for (i = 0; i < F_SET_SIZE; i++)
    fset[i] = 1.0;
}
//=============================================================================
void ClearFSet(FSet fset)
{
  int i;
  for (i = 0; i < F_SET_SIZE; i++)
    fset[i] = 0.0;
}
//=============================================================================
void DeFuzzify(FSet fset, int size, double* result)
{
  //Defuzzify using Center of Gravity (CoG)
  double sum;
  double wsum;
  int i;

  sum = 0.0;
  wsum = 0.0;
  for (i = 0; i < size; i++)
  {
    sum += fset[i];
    wsum += fset[i] * (double)i;
  }

  //Prevent division by zero
  if (sum > 0.0001)
  {
  	 *result = (wsum / sum)/(size-1);
  }
  else
  {
    printf("fuzzy.c: DeFuzzify: warning: empty fuzzy set, returning 0\n");
    *result = 0.0;
  }
}
//=============================================================================
