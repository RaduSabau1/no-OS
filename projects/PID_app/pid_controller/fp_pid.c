#include "fp_pid.h"

/**
  * @brief  Calculate and update the P-controller output.
  * @param  ptP: pointer to user data structure with type "ptP".
  */
void tP_calc(struct sP *ptP)
{
	float fPreOut = ptP->fIn * ptP->fKp;

	if(fPreOut > ptP->fUpOutLim) fPreOut = ptP->fUpOutLim;
	if(fPreOut < ptP->fLowOutLim) fPreOut = ptP->fLowOutLim;

	ptP->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of P-controller to defaults.
  * @param  ptP: pointer to user data structure with type "ptP".
  */
void tP_rst(struct sP *ptP)
{
	ptP->fIn = 0.0f;
	ptP->fOut = 0.0f;
}

/**
  * @brief  Calculate and update the PI-controller output.
  * @param  ptPI: pointer to user data structure with type "ptPI".
  */
void tPI_calc(struct sPI *ptPI)
{
	float fPreOut;

	ptPI->fPout = ptPI->fIn * ptPI->fKp;

	ptPI->fIout = ptPI->fIprevOut + 10 * ptPI->fDtSec * (ptPI->fPout * ptPI->fKi +
			ptPI->fIprevIn);
	ptPI->fIprevIn = ptPI->fPout;
	ptPI->fIprevOut = ptPI->fIout;

	fPreOut = ptPI->fPout + ptPI->fIout;

	if(fPreOut > ptPI->fUpOutLim) fPreOut = ptPI->fUpOutLim;
	if(fPreOut < ptPI->fLowOutLim) fPreOut = ptPI->fLowOutLim;

	ptPI->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of PI-controller to defaults.
  * @param  ptPI: pointer to user data structure with type "ptPI".
  */
void tPI_rst(struct sPI *ptPI)
{
	ptPI->fIn = 0.0f;
	ptPI->fIout = 0.0f;
	ptPI->fIprevIn = 0.0f;
	ptPI->fIprevOut = 0.0f;
	ptPI->fOut = 0.0f;
	ptPI->fPout = 0.0f;
}

/**
  * @brief  Calculate and update the PID-controller output.
  * @param  ptPID: pointer to user data structure with type "tPID".
  */
void tPID_calc(struct sPID *ptPID)
{
	float fPreOut;

	ptPID->fPout = ptPID->fIn * ptPID->fKp;

	ptPID->fIout = ptPID->fIprevOut + 10 * ptPID->fDtSec * (ptPID->fPout *
			ptPID->fKi + ptPID->fIprevIn);
	ptPID->fIprevIn = ptPID->fPout;
	ptPID->fIprevOut = ptPID->fIout;

	ptPID->fDout = (ptPID->fPout * ptPID->fKd - ptPID->fDprevIn) / ptPID->fDtSec;
	ptPID->fDprevIn = ptPID->fPout;
	ptPID->fDprevOut = ptPID->fDout;

	fPreOut = ptPID->fPout + ptPID->fIout + ptPID->fDout;

	if(fPreOut > ptPID->fUpOutLim) fPreOut = ptPID->fUpOutLim;
	if(fPreOut < ptPID->fLowOutLim) fPreOut = ptPID->fLowOutLim;

	ptPID->fOut = fPreOut;
}

/**
  * @brief  Reset the internal variables of PID-controller to defaults.
  * @param  ptPID: pointer to user data structure with type "tPID".
  * @retval None
  */
void tPID_rst(struct sPID *ptPID)
{
	ptPID->fDout = 0.0f;
	ptPID->fDprevIn = 0.0f;
	ptPID->fDprevOut = 0.0f;
	ptPID->fIn = 0.0f;
	ptPID->fIout = 0.0f;
	ptPID->fIprevIn = 0.0f;
	ptPID->fIprevOut = 0.0f;
	ptPID->fOut = 0.0f;
	ptPID->fPout = 0.0f;
}
