//#############################################################################
// $Copyright:
// Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef EST_LS_STATES_H
#define EST_LS_STATES_H

//! \file   libraries/observers/est/include/est_Ls_states.h
//! \brief  Contains the states for the stator
//!         inductance estimator (EST_Ls) module routines
//!


// **************************************************************************
// the includes


//!
//!
//! \addtogroup EST
//!
//! @{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

// **************************************************************************
// the typedefs

//! \brief Enumeration for the stator inductance estimator error codes
//!
typedef enum
{
  EST_LS_ERRORCODE_NOERROR=0,       //!< no error error code
  EST_LS_ERRORCODE_SHIFTOVERFLOW,   //!< inductance shift overflow error code
  EST_LS_NUMERRORCODES              //!< the number of estimator error codes
} EST_Ls_ErrorCode_e;


//! \brief Enumeration for the stator inductance estimator states
//!
typedef enum
{
  EST_LS_STATE_ERROR = 0,   //!< error
  EST_LS_STATE_IDLE = 1,    //!< idle
  EST_LS_STATE_RAMPUP = 2,  //!< the ramp up state
  EST_LS_STATE_INIT = 3,    //!< the init state
  EST_LS_STATE_COARSE = 4,  //!< the coarse estimation state
  EST_LS_STATE_FINE = 5,    //!< the fine estimation state
  EST_LS_STATE_DONE = 6,    //!< the done state
  EST_LS_NUMSTATES = 7      //!< number of stator inductance estimator states
} EST_Ls_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//! @} // ingroup
#endif // end of EST_LS_STATES_H definition

