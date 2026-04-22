#include "vector_transfs.h"

#define SQRT_3 sqrtf(3.0f)

/**
  * @brief  Forward full Clarke transformation function: ABC to Al-Be.
  * @param  ptFFClarke: pointer to user data structure with type "tFFClarke".
  */
void tFFClarke_abc2albe(struct sFFClarke* ptFFClarke)
{
	ptFFClarke->fAl = (2.0f/3.0f)*ptFFClarke->fA - (1.0f/3.0f)*
			  (ptFFClarke->fB - ptFFClarke->fC);
	ptFFClarke->fBe = (2.0f/SQRT_3)*(ptFFClarke->fB - ptFFClarke->fC);
}

/**
  * @brief  Forward reduce Clarke transformation function: AB to Al-Be.
  * @param  ptFRClarke: pointer to user data structure with type "tFRClarke".
  */
void tFRClarke_ab2albe(struct sFRClarke* ptFRClarke)
{
	ptFRClarke->fAl = ptFRClarke->fA;
	ptFRClarke->fBe = (1.0f/SQRT_3)*(ptFRClarke->fA + 2.0f*ptFRClarke->fB);
}

/**
  * @brief  Inverse full Clarke transformation function: Al-Be to ABC.
  * @param  ptIFClarke: pointer to user data structure with type "tIFClarke".
  */
void tIFClarke_albe2abc(struct sIFClarke* ptIFClarke)
{
	ptIFClarke->fA = ptIFClarke->fAl;
	ptIFClarke->fB = 0.5f*(-ptIFClarke->fAl + SQRT_3*ptIFClarke->fBe);
	ptIFClarke->fC = 0.5f*(-ptIFClarke->fAl - SQRT_3*ptIFClarke->fBe);
}

/**
  * @brief  Inverse reduce Clarke transformation function: Al-Be to AB.
  * @param  ptIRClarke: pointer to user data structure with type "tIRClarke".
  */
void tIRClarke_albe2ab(struct sIRClarke* ptIRClarke)
{
	ptIRClarke->fA = ptIRClarke->fAl;
	ptIRClarke->fB = 0.5f*(SQRT_3*ptIRClarke->fBe - ptIRClarke->fAl);
}

/**
  * @brief  Forward Park transformation function: Al-Be to DQ.
  * @param  ptFPark: pointer to user data structure with type "tFPark".
  */
void tFPark_albe2dq(struct sFPark* ptFPark)
{
	ptFPark->fD = ptFPark->fAl*ptFPark->fCosAng + ptFPark->fBe*ptFPark->fSinAng;
	ptFPark->fQ = ptFPark->fBe*ptFPark->fCosAng - ptFPark->fAl*ptFPark->fSinAng;
}

/**
  * @brief  Inverse Park transformation function: DQ to Al-Be.
  * @param  ptIPark: pointer to user data structure with type "tIPark".
  */
void tIPark_dq2albe(struct sIPark* ptIPark)
{
	ptIPark->fAl = ptIPark->fD*ptIPark->fCosAng - ptIPark->fQ*ptIPark->fSinAng;
	ptIPark->fBe = ptIPark->fQ*ptIPark->fCosAng + ptIPark->fD*ptIPark->fSinAng;
}
