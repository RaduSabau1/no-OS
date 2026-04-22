#ifndef __FP_PID_H__
#define __FP_PID_H__

/* Includes -----------------------------------------------------------------------*/
/* Exported types -----------------------------------------------------------------*/

/**
  * @brief "Floating point P Controller Module" data structure
  */
struct sP {
// Inputs:
	float fIn;			// Controller's input
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
	float fKp;			// Proportional coefficient value
// Outputs:
	float fOut;			// Controller's output
};

/**
  * @brief "Floating point PI Controller Module" data structure
  */
struct sPI {
// Inputs:
	float fDtSec;			// Discretization time, Sec
	float fIn;			// Controller's input
	float fKp;			// Proportional coefficient value
	float fKi;			// Integral coefficient value
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
// Internal variables:
	float fPout;			// Proportional link's output
	float fIout;			// Integral link's output
	float fIprevIn;			// Integral link's previous input
	float fIprevOut;		// Integral link's previous output
// Outputs:
	float fOut;			// Controller's output
};

/**
  * @brief "Floating point PID Controller Module" data structure
  */
struct sPID {
// Inputs:
	float fDtSec;			// Discretization time, Sec
	float fIn;			// Controller's input
	float fKp;			// Proportional coefficient value
	float fKi;			// Integral coefficient value
	float fKd;			// Derivative coefficient value
	float fUpOutLim;		// Controller's output upper limit
	float fLowOutLim;		// Controller's output lower limit
// Internal variables:
	float fPout;			// Proportional link's output
	float fIout;			// Integral link's output
	float fDout;			// Derivative link's output
	float fIprevIn;			// Integral link's previous input
	float fIprevOut;		// Integral link's previous output
	float fDprevIn;			// Derivative link's previous input
	float fDprevOut;		// Derivative link's previous output
// Outputs:
	float fOut;			// Controller's output
};

/* P controller's output calculation function prototype */
void tP_calc(struct sP*);

/* Reset the internal variables of P cnotroller */
void tP_rst(struct sP*);

/* PI controller's output calculation function prototype */
void tPI_calc(struct sPI*);

/* Reset the internal variables of PI cnotroller */
void tPI_rst(struct sPI*);

/* PID controller's output calculation function prototype */
void tPID_calc(struct sPID*);

/* Reset the internal variables of PID cnotroller */
void tPID_rst(struct sPID*);

#endif /* __FP_PID_H__ */
