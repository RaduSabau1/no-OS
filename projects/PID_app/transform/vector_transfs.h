#ifndef __VECTOR_TRANSFS_H__
#define __VECTOR_TRANSFS_H__
#include <math.h>

/**
  * @brief "Forward full Clarke transformation Module" data structure: ABC to Al-Be
  */
struct sFFClarke {
// Inputs:
	float fA;				// A input
	float fB;				// B input
	float fC;				// C input
// Outputs:
	float fAl;				// Alpha output
	float fBe;				// Beta output
};

/**
  * @brief "Forward reduce Clarke transformation Module" data structure: AB to Al-Be
  */
struct sFRClarke {
// Inputs:
	float fA;				// A input
	float fB;				// B input
// Outputs:
	float fAl;				// Alpha output
	float fBe;				// Beta output
};

/**
  * @brief "Inverse full Clarke transformation Module" data structure: Al-Be to ABC
  */
struct sIFClarke {
// Inputs:
	float fAl;				// Alpha input
	float fBe;				// Beta input
// Outputs:
	float fA;				// A output
	float fB;				// B output
	float fC;				// C output
};
/**
  * @brief "Inverse reduce Clarke transformation Module" data structure: Al-Be to AB
  */
struct sIRClarke {
// Inputs:
	float fAl;				// Alpha input
	float fBe;				// Beta input
// Outputs:
	float fA;				// A output
	float fB;				// B output
// Functions:
	void  (*m_albe2ab)(struct sIRClarke*);	// pointer to calculation function
};

/**
  * @brief "Forward Park transformation Module" data structure: Al-Be to DQ
  */
struct sFPark {
// Inputs:
	float fAl;				// Alpha input
	float fBe;				// Beta input
	float fSinAng;				// sin(Angle) input
	float fCosAng;				// cos(Angle) input
// Outputs:
	float fD;				// D output
	float fQ;				// Q output
};

/**
  * @brief "Inverse Park transformation Module" data structure: DQ to Al-Be
  */
struct sIPark {
// Inputs:
	float fD;				// D input
	float fQ;				// Q input
	float fSinAng;				// sin(Angle) input
	float fCosAng;				// cos(Angle) input
// Outputs:
	float fAl;				// Alpha output
	float fBe;				// Beta output
};

/* Forward full Clarke transformation function */
void tFFClarke_abc2albe(struct sFFClarke*);

/* Forward reduce Clarke transformation function prototype */
void tFRClarke_ab2albe(struct sFRClarke*);

/* Inverse full Clarke transformation function prototype */
void tIFClarke_albe2abc(struct sIFClarke*);

/* Inverse reduce Clarke transformation function prototype */
void tIRClarke_albe2ab(struct sIRClarke*);

/* Forward Park transformation function prototype */
void tFPark_albe2dq(struct sFPark*);

/* Inverse Park transformation function prototype */
void tIPark_dq2albe(struct sIPark*);

#endif /* __VECTOR_TRANSFS_H__ */
