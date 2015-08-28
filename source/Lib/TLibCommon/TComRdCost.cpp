/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComRdCost.cpp
    \brief    RD cost computation class
*/

#include <math.h>
#include <assert.h>
#include <limits>
#include "TComRom.h"
#include "TComRdCost.h"

#include <arm_neon.h>

#if defined (_MSC_VER)
#define __static_assert
#endif // !_MSC_VER

//! \ingroup TLibCommon
//! \{

TComRdCost::TComRdCost()
{
  init();
}

TComRdCost::~TComRdCost()
{
}

// Calculate RD functions
Double TComRdCost::calcRdCost( UInt uiBits, Distortion uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;

  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
#if RExt__HIGH_BIT_DEPTH_SUPPORT
      dLambda = m_dLambdaMotionSAD[0]; // 0 is valid, because for lossless blocks, the cost equation is modified to compensate.
#else
      dLambda = (Double)m_uiLambdaMotionSAD[0]; // 0 is valid, because for lossless blocks, the cost equation is modified to compensate.
#endif
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }

  if (bFlag) //NOTE: this "bFlag" is never true
  {
    // Intra8x8, Intra4x4 Block only...
    if (m_costMode != COST_STANDARD_LOSSY)
    {
      dRdCost = (Double(uiDistortion) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
    }
    else
    {
      dRdCost = (((Double)uiDistortion) + ((Double)uiBits * dLambda));
    }
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      if (m_costMode != COST_STANDARD_LOSSY)
      {
        dRdCost = ((Double(uiDistortion) * 65536) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
      }
      else
      {
        dRdCost = floor(Double(uiDistortion) + (floor((Double(uiBits) * dLambda) + 0.5) / 65536.0));
      }
    }
    else
    {
      if (m_costMode != COST_STANDARD_LOSSY)
      {
        dRdCost = (Double(uiDistortion) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
      }
      else
      {
        dRdCost = floor(Double(uiDistortion) + (Double(uiBits) * dLambda) + 0.5);
      }
    }
  }

  return dRdCost;
}

Double TComRdCost::calcRdCost64( UInt64 uiBits, UInt64 uiDistortion, Bool bFlag, DFunc eDFunc )
{
  Double dRdCost = 0.0;
  Double dLambda = 0.0;

  switch ( eDFunc )
  {
    case DF_SSE:
      assert(0);
      break;
    case DF_SAD:
#if RExt__HIGH_BIT_DEPTH_SUPPORT
      dLambda = m_dLambdaMotionSAD[0]; // 0 is valid, because for lossless blocks, the cost equation is modified to compensate.
#else
      dLambda = (Double)m_uiLambdaMotionSAD[0]; // 0 is valid, because for lossless blocks, the cost equation is modified to compensate.
#endif
      break;
    case DF_DEFAULT:
      dLambda =         m_dLambda;
      break;
    case DF_SSE_FRAME:
      dLambda =         m_dFrameLambda;
      break;
    default:
      assert (0);
      break;
  }

  if (bFlag) //NOTE: this "bFlag" is never true
  {
    // Intra8x8, Intra4x4 Block only...
    if (m_costMode != COST_STANDARD_LOSSY)
    {
      dRdCost = (Double(uiDistortion) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
    }
    else
    {
      dRdCost = (((Double)(Int64)uiDistortion) + ((Double)(Int64)uiBits * dLambda));
    }
  }
  else
  {
    if (eDFunc == DF_SAD)
    {
      if (m_costMode != COST_STANDARD_LOSSY)
      {
        dRdCost = ((Double(uiDistortion) * 65536) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
      }
      else
      {
        dRdCost = floor(Double(uiDistortion) + (floor((Double(uiBits) * dLambda) + 0.5) / 65536.0));
      }
    }
    else
    {
      if (m_costMode != COST_STANDARD_LOSSY)
      {
        dRdCost = (Double(uiDistortion) / dLambda) + Double(uiBits); // all lossless costs would have uiDistortion=0, and therefore this cost function can be used.
      }
      else
      {
        dRdCost = floor(Double(uiDistortion) + (Double(uiBits) * dLambda) + 0.5);
      }
    }
  }

  return dRdCost;
}

Void TComRdCost::setLambda( Double dLambda, const BitDepths &bitDepths )
{
  m_dLambda           = dLambda;
  m_sqrtLambda        = sqrt(m_dLambda);
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  m_dLambdaMotionSAD[0] = 65536.0 * m_sqrtLambda;
  m_dLambdaMotionSSE[0] = 65536.0 * m_dLambda;
#if FULL_NBIT
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0));
#else
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (bitDepths.recon[CHANNEL_TYPE_LUMA] - 8)) / 3.0));
#endif
  m_dLambdaMotionSAD[1] = 65536.0 * sqrt(dLambda);
  m_dLambdaMotionSSE[1] = 65536.0 * dLambda;
#else
  m_uiLambdaMotionSAD[0] = (UInt)floor(65536.0 * m_sqrtLambda);
  m_uiLambdaMotionSSE[0] = (UInt)floor(65536.0 * m_dLambda   );
#if FULL_NBIT
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12) / 3.0));
#else
  dLambda = 0.57 * pow(2.0, ((LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME - 12 - 6 * (bitDepths.recon[CHANNEL_TYPE_LUMA] - 8)) / 3.0));
#endif
  m_uiLambdaMotionSAD[1] = (UInt)floor(65536.0 * sqrt(dLambda));
  m_uiLambdaMotionSSE[1] = (UInt)floor(65536.0 * dLambda   );
#endif
}


// Initalize Function Pointer by [eDFunc]
Void TComRdCost::init()
{
  m_afpDistortFunc[DF_DEFAULT] = NULL;                  // for DF_DEFAULT

  m_afpDistortFunc[DF_SSE    ] = TComRdCost::xGetSSE;
  m_afpDistortFunc[DF_SSE4   ] = TComRdCost::xGetSSE4;
  m_afpDistortFunc[DF_SSE8   ] = TComRdCost::xGetSSE8;
  m_afpDistortFunc[DF_SSE16  ] = TComRdCost::xGetSSE16;
  m_afpDistortFunc[DF_SSE32  ] = TComRdCost::xGetSSE32;
  m_afpDistortFunc[DF_SSE64  ] = TComRdCost::xGetSSE64;
  m_afpDistortFunc[DF_SSE16N ] = TComRdCost::xGetSSE16N;

  m_afpDistortFunc[DF_SAD    ] = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SAD4   ] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SAD8   ] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SAD16  ] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SAD32  ] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SAD64  ] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SAD16N ] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SADS   ] = TComRdCost::xGetSAD;
  m_afpDistortFunc[DF_SADS4  ] = TComRdCost::xGetSAD4;
  m_afpDistortFunc[DF_SADS8  ] = TComRdCost::xGetSAD8;
  m_afpDistortFunc[DF_SADS16 ] = TComRdCost::xGetSAD16;
  m_afpDistortFunc[DF_SADS32 ] = TComRdCost::xGetSAD32;
  m_afpDistortFunc[DF_SADS64 ] = TComRdCost::xGetSAD64;
  m_afpDistortFunc[DF_SADS16N] = TComRdCost::xGetSAD16N;

  m_afpDistortFunc[DF_SAD12  ] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SAD24  ] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SAD48  ] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[DF_SADS12 ] = TComRdCost::xGetSAD12;
  m_afpDistortFunc[DF_SADS24 ] = TComRdCost::xGetSAD24;
  m_afpDistortFunc[DF_SADS48 ] = TComRdCost::xGetSAD48;

  m_afpDistortFunc[DF_HADS   ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS4  ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS8  ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS16 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS32 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS64 ] = TComRdCost::xGetHADs;
  m_afpDistortFunc[DF_HADS16N] = TComRdCost::xGetHADs;

  m_costMode                   = COST_STANDARD_LOSSY;

#if RExt__HIGH_BIT_DEPTH_SUPPORT
  m_dCost                      = 0;
#else
  m_uiCost                     = 0;
#endif
  m_iCostScale                 = 0;
}

// Static member function
UInt TComRdCost::xGetExpGolombNumberOfBits( Int iVal )
{
  assert(iVal != std::numeric_limits<Int>::min());
  UInt uiLength = 1;
  UInt uiTemp   = ( iVal <= 0) ? (UInt(-iVal)<<1)+1: UInt(iVal<<1);

  while ( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }

  return uiLength;
}

Void TComRdCost::setDistParam( UInt uiBlkWidth, UInt uiBlkHeight, DFunc eDFunc, DistParam& rcDistParam )
{
  // set Block Width / Height
  rcDistParam.iCols    = uiBlkWidth;
  rcDistParam.iRows    = uiBlkHeight;
  rcDistParam.DistFunc = m_afpDistortFunc[eDFunc + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];

  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (ME)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, DistParam& rcDistParam )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;

  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();
  rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];

  if (rcDistParam.iCols == 12)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD12];
  }
  else if (rcDistParam.iCols == 24)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD24];
  }
  else if (rcDistParam.iCols == 48)
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SAD48];
  }

  // initialize
  rcDistParam.iSubShift  = 0;
}

// Setting the Distortion Parameter for Inter (subpel ME with step)
Void TComRdCost::setDistParam( TComPattern* pcPatternKey, Pel* piRefY, Int iRefStride, Int iStep, DistParam& rcDistParam, Bool bHADME )
{
  // set Original & Curr Pointer / Stride
  rcDistParam.pOrg = pcPatternKey->getROIY();
  rcDistParam.pCur = piRefY;

  rcDistParam.iStrideOrg = pcPatternKey->getPatternLStride();
  rcDistParam.iStrideCur = iRefStride * iStep;

  // set Step for interpolated buffer
  rcDistParam.iStep = iStep;

  // set Block Width / Height
  rcDistParam.iCols    = pcPatternKey->getROIYWidth();
  rcDistParam.iRows    = pcPatternKey->getROIYHeight();

  // set distortion function
  if ( !bHADME )
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
    if (rcDistParam.iCols == 12)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS12];
    }
    else if (rcDistParam.iCols == 24)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS24];
    }
    else if (rcDistParam.iCols == 48)
    {
      rcDistParam.DistFunc = m_afpDistortFunc[DF_SADS48];
    }
  }
  else
  {
    rcDistParam.DistFunc = m_afpDistortFunc[DF_HADS + g_aucConvertToBit[ rcDistParam.iCols ] + 1 ];
  }

  // initialize
  rcDistParam.iSubShift  = 0;
}

Void TComRdCost::setDistParam( DistParam& rcDP, Int bitDepth, Pel* p1, Int iStride1, Pel* p2, Int iStride2, Int iWidth, Int iHeight, Bool bHadamard )
{
  rcDP.pOrg       = p1;
  rcDP.pCur       = p2;
  rcDP.iStrideOrg = iStride1;
  rcDP.iStrideCur = iStride2;
  rcDP.iCols      = iWidth;
  rcDP.iRows      = iHeight;
  rcDP.iStep      = 1;
  rcDP.iSubShift  = 0;
  rcDP.bitDepth   = bitDepth;
  rcDP.DistFunc   = m_afpDistortFunc[ ( bHadamard ? DF_HADS : DF_SADS ) + g_aucConvertToBit[ iWidth ] + 1 ];
}

Distortion TComRdCost::calcHAD( Int bitDepth, Pel* pi0, Int iStride0, Pel* pi1, Int iStride1, Int iWidth, Int iHeight )
{
  Distortion uiSum = 0;
  Int x, y;

  if ( ( (iWidth % 8) == 0 ) && ( (iHeight % 8) == 0 ) )
  {
    for ( y=0; y<iHeight; y+= 8 )
    {
      for ( x=0; x<iWidth; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*8;
      pi1 += iStride1*8;
    }
  }
  else
  {
    assert ( ( (iWidth % 4) == 0 ) && ( (iHeight % 4) == 0 ) );

    for ( y=0; y<iHeight; y+= 4 )
    {
      for ( x=0; x<iWidth; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &pi0[x], &pi1[x], iStride0, iStride1, 1 );
      }
      pi0 += iStride0*4;
      pi1 += iStride1*4;
    }
  }

  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(bitDepth-8) );
}

Distortion TComRdCost::getDistPart( Int bitDepth, Pel* piCur, Int iCurStride,  Pel* piOrg, Int iOrgStride, UInt uiBlkWidth, UInt uiBlkHeight, const ComponentID compID, DFunc eDFunc )
{
  DistParam cDtParam;
  setDistParam( uiBlkWidth, uiBlkHeight, eDFunc, cDtParam );
  cDtParam.pOrg       = piOrg;
  cDtParam.pCur       = piCur;
  cDtParam.iStrideOrg = iOrgStride;
  cDtParam.iStrideCur = iCurStride;
  cDtParam.iStep      = 1;

  cDtParam.bApplyWeight = false;
  cDtParam.compIdx      = MAX_NUM_COMPONENT; // just for assert: to be sure it was set before use
  cDtParam.bitDepth     = bitDepth;

  if (isChroma(compID))
  {
    return ((Distortion) (m_distortionWeight[compID] * cDtParam.DistFunc( &cDtParam )));
  }
  else
  {
    return cDtParam.DistFunc( &cDtParam );
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

Distortion TComRdCost::xGetSAD( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ ) // loop vectorized + peeled.
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD4( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  int16x4_t v_iSum0 = vdup_n_s16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_iSum0 += |piOrg - piCur|  in  [0; 3]
    v_iSum0 = vaba_s16(
        v_iSum0,                      
        vld1_s16((int16_t *)&piOrg[0]),
        vld1_s16((int16_t *)&piCur[0])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum = (UInt)
    (
      vget_lane_s16(v_iSum0, 0) + vget_lane_s16(v_iSum0, 1) +
      vget_lane_s16(v_iSum0, 2) + vget_lane_s16(v_iSum0, 3)
    );

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD8( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg      = pcDtParam->pOrg;
  const Pel* piCur      = pcDtParam->pCur;
  Int  iRows      = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  int16x8_t v_iSum0 = vdupq_n_s16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_iSum0 += |piOrg - piCur|  in  [0; 7]
    v_iSum0 = vabaq_s16(
        v_iSum0,
        vld1q_s16((int16_t *)&piOrg[0]),
        vld1q_s16((int16_t *)&piCur[0])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum = (UInt)
    (
      vgetq_lane_s16(v_iSum0, 0) + vgetq_lane_s16(v_iSum0, 1) +
      vgetq_lane_s16(v_iSum0, 2) + vgetq_lane_s16(v_iSum0, 3) +
      vgetq_lane_s16(v_iSum0, 4) + vgetq_lane_s16(v_iSum0, 5) +
      vgetq_lane_s16(v_iSum0, 6) + vgetq_lane_s16(v_iSum0, 7)
    );

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD16( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur; // Pel is Short
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  uint16x8_t v_uiSum0 = vdupq_n_u16(0);
  uint16x8_t v_uiSum1 = vdupq_n_u16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_uiSum0 += |piOrg - piCur|  -  [0; 7]
    v_uiSum0 = vabaq_u16(
      v_uiSum0,
      vld1q_u16((uint16_t *)&piOrg[0]),
      vld1q_u16((uint16_t *)&piCur[0])
      );

    // v_uiSum1 += |piOrg - piCur|  -  [8; 15]
    v_uiSum1 = vabaq_u16(
      v_uiSum1,
      vld1q_u16((uint16_t *)&piOrg[8]),
      vld1q_u16((uint16_t *)&piCur[8])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum =
    vgetq_lane_u16(v_uiSum0, 0) + vgetq_lane_u16(v_uiSum0, 1) +
    vgetq_lane_u16(v_uiSum0, 2) + vgetq_lane_u16(v_uiSum0, 3) +
    vgetq_lane_u16(v_uiSum0, 4) + vgetq_lane_u16(v_uiSum0, 5) +
    vgetq_lane_u16(v_uiSum0, 6) + vgetq_lane_u16(v_uiSum0, 7) +
    vgetq_lane_u16(v_uiSum1, 0) + vgetq_lane_u16(v_uiSum1, 1) +
    vgetq_lane_u16(v_uiSum1, 2) + vgetq_lane_u16(v_uiSum1, 3) +
    vgetq_lane_u16(v_uiSum1, 4) + vgetq_lane_u16(v_uiSum1, 5) +
    vgetq_lane_u16(v_uiSum1, 6) + vgetq_lane_u16(v_uiSum1, 7);

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD12( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  uint32x4_t v_uiSum1 = vdupq_n_u32(0);
  uint32x4_t v_uiSum2 = vdupq_n_u32(0);
  uint32x4_t v_uiSum3 = vdupq_n_u32(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_uiSum1 += |piOrg - piCur|  -  [0; 3]
    v_uiSum1 = vabal_u16(
      v_uiSum1,
      vld1_u16((uint16_t *)&piOrg[0]),
      vld1_u16((uint16_t *)&piCur[0])
      );

    // v_uiSum2 += |piOrg - piCur|  -  [4; 7]
    v_uiSum2 = vabal_u16(
      v_uiSum2,
      vld1_u16((uint16_t *)&piOrg[4]),
      vld1_u16((uint16_t *)&piCur[4])
      );

    // v_uiSum3 += |piOrg - piCur|  -  [8; 11]
    v_uiSum3 = vabal_u16(
      v_uiSum3,
      vld1_u16((uint16_t *)&piOrg[8]),
      vld1_u16((uint16_t *)&piCur[8])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum =
    vgetq_lane_u32(v_uiSum1, 0) + vgetq_lane_u32(v_uiSum2, 0) +
    vgetq_lane_u32(v_uiSum3, 0) +
    vgetq_lane_u32(v_uiSum1, 1) + vgetq_lane_u32(v_uiSum2, 1) +
    vgetq_lane_u32(v_uiSum3, 1) + 
    vgetq_lane_u32(v_uiSum1, 2) + vgetq_lane_u32(v_uiSum2, 2) +
    vgetq_lane_u32(v_uiSum3, 2) + 
    vgetq_lane_u32(v_uiSum1, 3) + vgetq_lane_u32(v_uiSum2, 3) +
    vgetq_lane_u32(v_uiSum3, 3);

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD16N( DistParam* pcDtParam )
{
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (Int n = 0; n < iCols; n+=16 ) // loop vectorized.
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD32( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  int16x8_t v_iSum0 = vdupq_n_s16(0);
  int16x8_t v_iSum1 = vdupq_n_s16(0);
  int16x8_t v_iSum2 = vdupq_n_s16(0);
  int16x8_t v_iSum3 = vdupq_n_s16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_iSum0 += |piOrg - piCur|  in  [0; 7]
    v_iSum0 = vabaq_s16(
        v_iSum0,
        vld1q_s16((int16_t *)&piOrg[0]),
        vld1q_s16((int16_t *)&piCur[0])
      );

    // v_iSum1 += |piOrg - piCur|  in  [8; 15]
    v_iSum1 = vabaq_s16(
        v_iSum1,
        vld1q_s16((int16_t *)&piOrg[8]),
        vld1q_s16((int16_t *)&piCur[8])
      );

    // v_iSum2 += |piOrg - piCur|  in  [16; 23]
    v_iSum2 = vabaq_s16(
        v_iSum2,
        vld1q_s16((int16_t *)&piOrg[16]),
        vld1q_s16((int16_t *)&piCur[16])
      );

    // v_iSum3 += |piOrg - piCur|  in  [24; 31]
    v_iSum3 = vabaq_s16(
        v_iSum3,
        vld1q_s16((int16_t *)&piOrg[24]),
        vld1q_s16((int16_t *)&piCur[24])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum = (UInt)
    (
      vgetq_lane_s16(v_iSum0, 0) + vgetq_lane_s16(v_iSum0, 1) +
      vgetq_lane_s16(v_iSum0, 2) + vgetq_lane_s16(v_iSum0, 3) +
      vgetq_lane_s16(v_iSum0, 4) + vgetq_lane_s16(v_iSum0, 5) +
      vgetq_lane_s16(v_iSum0, 6) + vgetq_lane_s16(v_iSum0, 7) +
      vgetq_lane_s16(v_iSum1, 0) + vgetq_lane_s16(v_iSum1, 1) +
      vgetq_lane_s16(v_iSum1, 2) + vgetq_lane_s16(v_iSum1, 3) +
      vgetq_lane_s16(v_iSum1, 4) + vgetq_lane_s16(v_iSum1, 5) +
      vgetq_lane_s16(v_iSum1, 6) + vgetq_lane_s16(v_iSum1, 7) +
      vgetq_lane_s16(v_iSum2, 0) + vgetq_lane_s16(v_iSum2, 1) +
      vgetq_lane_s16(v_iSum2, 2) + vgetq_lane_s16(v_iSum2, 3) +
      vgetq_lane_s16(v_iSum2, 4) + vgetq_lane_s16(v_iSum2, 5) +
      vgetq_lane_s16(v_iSum2, 6) + vgetq_lane_s16(v_iSum2, 7) +
      vgetq_lane_s16(v_iSum3, 0) + vgetq_lane_s16(v_iSum3, 1) +
      vgetq_lane_s16(v_iSum3, 2) + vgetq_lane_s16(v_iSum3, 3) +
      vgetq_lane_s16(v_iSum3, 4) + vgetq_lane_s16(v_iSum3, 5) +
      vgetq_lane_s16(v_iSum3, 6) + vgetq_lane_s16(v_iSum3, 7)
    );

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD24( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  int16x8_t v_iSum0 = vdupq_n_s16(0);
  int16x8_t v_iSum1 = vdupq_n_s16(0);
  int16x8_t v_iSum2 = vdupq_n_s16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_iSum0 += |piOrg - piCur|  in  [0; 7]
    v_iSum0 = vabaq_s16(
        v_iSum0,
        vld1q_s16((int16_t *)&piOrg[0]),
        vld1q_s16((int16_t *)&piCur[0])
      );

    // v_iSum1 += |piOrg - piCur|  in  [8; 15]
    v_iSum1 = vabaq_s16(
        v_iSum1,
        vld1q_s16((int16_t *)&piOrg[8]),
        vld1q_s16((int16_t *)&piCur[8])
      );

    // v_iSum2 += |piOrg - piCur|  in  [16; 23]
    v_iSum2 = vabaq_s16(
        v_iSum2,
        vld1q_s16((int16_t *)&piOrg[16]),
        vld1q_s16((int16_t *)&piCur[16])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum = (UInt)
    (
      vgetq_lane_s16(v_iSum0, 0) + vgetq_lane_s16(v_iSum0, 1) +
      vgetq_lane_s16(v_iSum0, 2) + vgetq_lane_s16(v_iSum0, 3) +
      vgetq_lane_s16(v_iSum0, 4) + vgetq_lane_s16(v_iSum0, 5) +
      vgetq_lane_s16(v_iSum0, 6) + vgetq_lane_s16(v_iSum0, 7) +
      vgetq_lane_s16(v_iSum1, 0) + vgetq_lane_s16(v_iSum1, 1) +
      vgetq_lane_s16(v_iSum1, 2) + vgetq_lane_s16(v_iSum1, 3) +
      vgetq_lane_s16(v_iSum1, 4) + vgetq_lane_s16(v_iSum1, 5) +
      vgetq_lane_s16(v_iSum1, 6) + vgetq_lane_s16(v_iSum1, 7) +
      vgetq_lane_s16(v_iSum2, 0) + vgetq_lane_s16(v_iSum2, 1) +
      vgetq_lane_s16(v_iSum2, 2) + vgetq_lane_s16(v_iSum2, 3) +
      vgetq_lane_s16(v_iSum2, 4) + vgetq_lane_s16(v_iSum2, 5) +
      vgetq_lane_s16(v_iSum2, 6) + vgetq_lane_s16(v_iSum2, 7)
    );

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD64( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  int16x8_t v_iSum0 = vdupq_n_s16(0);
  int16x8_t v_iSum1 = vdupq_n_s16(0);
  int16x8_t v_iSum2 = vdupq_n_s16(0);
  int16x8_t v_iSum3 = vdupq_n_s16(0);
  int16x8_t v_iSum4 = vdupq_n_s16(0);
  int16x8_t v_iSum5 = vdupq_n_s16(0);
  int16x8_t v_iSum6 = vdupq_n_s16(0);
  int16x8_t v_iSum7 = vdupq_n_s16(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_iSum0 += |piOrg - piCur|  in  [0; 7]
    v_iSum0 = vabaq_s16(
        v_iSum0,
        vld1q_s16((int16_t *)&piOrg[0]),
        vld1q_s16((int16_t *)&piCur[0])
      );

    // v_iSum1 += |piOrg - piCur|  in  [8; 15]
    v_iSum1 = vabaq_s16(
        v_iSum1,
        vld1q_s16((int16_t *)&piOrg[8]),
        vld1q_s16((int16_t *)&piCur[8])
      );

    // v_iSum2 += |piOrg - piCur|  in  [16; 23]
    v_iSum2 = vabaq_s16(
        v_iSum2,
        vld1q_s16((int16_t *)&piOrg[16]),
        vld1q_s16((int16_t *)&piCur[16])
      );

    // v_iSum3 += |piOrg - piCur|  in  [24; 31]
    v_iSum3 = vabaq_s16(
        v_iSum3,
        vld1q_s16((int16_t *)&piOrg[24]),
        vld1q_s16((int16_t *)&piCur[24])
      );

    // v_iSum4 += |piOrg - piCur|  in  [32; 39]
    v_iSum4 = vabaq_s16(
        v_iSum4,
        vld1q_s16((int16_t *)&piOrg[32]),
        vld1q_s16((int16_t *)&piCur[32])
      );

    // v_iSum5 += |piOrg - piCur|  in  [40; 47]
    v_iSum5 = vabaq_s16(
        v_iSum5,
        vld1q_s16((int16_t *)&piOrg[40]),
        vld1q_s16((int16_t *)&piCur[40])
      );

    // v_iSum6 += |piOrg - piCur|  in  [48; 55]
    v_iSum6 = vabaq_s16(
        v_iSum6,
        vld1q_s16((int16_t *)&piOrg[48]),
        vld1q_s16((int16_t *)&piCur[48])
      );

    // v_iSum7 += |piOrg - piCur|  in  [56; 63]
    v_iSum7 = vabaq_s16(
        v_iSum7,
        vld1q_s16((int16_t *)&piOrg[56]),
        vld1q_s16((int16_t *)&piCur[56])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum = (UInt)
    (
      vgetq_lane_s16(v_iSum0, 0) + vgetq_lane_s16(v_iSum0, 1) +
      vgetq_lane_s16(v_iSum0, 2) + vgetq_lane_s16(v_iSum0, 3) +
      vgetq_lane_s16(v_iSum0, 4) + vgetq_lane_s16(v_iSum0, 5) +
      vgetq_lane_s16(v_iSum0, 6) + vgetq_lane_s16(v_iSum0, 7) +
      vgetq_lane_s16(v_iSum1, 0) + vgetq_lane_s16(v_iSum1, 1) +
      vgetq_lane_s16(v_iSum1, 2) + vgetq_lane_s16(v_iSum1, 3) +
      vgetq_lane_s16(v_iSum1, 4) + vgetq_lane_s16(v_iSum1, 5) +
      vgetq_lane_s16(v_iSum1, 6) + vgetq_lane_s16(v_iSum1, 7) +
      vgetq_lane_s16(v_iSum2, 0) + vgetq_lane_s16(v_iSum2, 1) +
      vgetq_lane_s16(v_iSum2, 2) + vgetq_lane_s16(v_iSum2, 3) +
      vgetq_lane_s16(v_iSum2, 4) + vgetq_lane_s16(v_iSum2, 5) +
      vgetq_lane_s16(v_iSum2, 6) + vgetq_lane_s16(v_iSum2, 7) +
      vgetq_lane_s16(v_iSum3, 0) + vgetq_lane_s16(v_iSum3, 1) +
      vgetq_lane_s16(v_iSum3, 2) + vgetq_lane_s16(v_iSum3, 3) +
      vgetq_lane_s16(v_iSum3, 4) + vgetq_lane_s16(v_iSum3, 5) +
      vgetq_lane_s16(v_iSum3, 6) + vgetq_lane_s16(v_iSum3, 7) +
      vgetq_lane_s16(v_iSum4, 0) + vgetq_lane_s16(v_iSum4, 1) +
      vgetq_lane_s16(v_iSum4, 2) + vgetq_lane_s16(v_iSum4, 3) +
      vgetq_lane_s16(v_iSum4, 4) + vgetq_lane_s16(v_iSum4, 5) +
      vgetq_lane_s16(v_iSum4, 6) + vgetq_lane_s16(v_iSum4, 7) +
      vgetq_lane_s16(v_iSum5, 0) + vgetq_lane_s16(v_iSum5, 1) +
      vgetq_lane_s16(v_iSum5, 2) + vgetq_lane_s16(v_iSum5, 3) +
      vgetq_lane_s16(v_iSum5, 4) + vgetq_lane_s16(v_iSum5, 5) +
      vgetq_lane_s16(v_iSum5, 6) + vgetq_lane_s16(v_iSum5, 7) +
      vgetq_lane_s16(v_iSum6, 0) + vgetq_lane_s16(v_iSum6, 1) +
      vgetq_lane_s16(v_iSum6, 2) + vgetq_lane_s16(v_iSum6, 3) +
      vgetq_lane_s16(v_iSum6, 4) + vgetq_lane_s16(v_iSum6, 5) +
      vgetq_lane_s16(v_iSum6, 6) + vgetq_lane_s16(v_iSum6, 7) +
      vgetq_lane_s16(v_iSum7, 0) + vgetq_lane_s16(v_iSum7, 1) +
      vgetq_lane_s16(v_iSum7, 2) + vgetq_lane_s16(v_iSum7, 3) +
      vgetq_lane_s16(v_iSum7, 4) + vgetq_lane_s16(v_iSum7, 5) +
      vgetq_lane_s16(v_iSum7, 6) + vgetq_lane_s16(v_iSum7, 7)
    );

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

Distortion TComRdCost::xGetSAD48( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSADw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iSubShift  = pcDtParam->iSubShift;
  Int  iSubStep   = ( 1 << iSubShift );
  Int  iStrideCur = pcDtParam->iStrideCur*iSubStep;
  Int  iStrideOrg = pcDtParam->iStrideOrg*iSubStep;

  uint32x4_t v_uiSum1  = vdupq_n_u32(0);
  uint32x4_t v_uiSum2  = vdupq_n_u32(0);
  uint32x4_t v_uiSum3  = vdupq_n_u32(0);
  uint32x4_t v_uiSum4  = vdupq_n_u32(0);
  uint32x4_t v_uiSum5  = vdupq_n_u32(0);
  uint32x4_t v_uiSum6  = vdupq_n_u32(0);
  uint32x4_t v_uiSum7  = vdupq_n_u32(0);
  uint32x4_t v_uiSum8  = vdupq_n_u32(0);
  uint32x4_t v_uiSum9  = vdupq_n_u32(0);
  uint32x4_t v_uiSum10 = vdupq_n_u32(0);
  uint32x4_t v_uiSum11 = vdupq_n_u32(0);
  uint32x4_t v_uiSum12 = vdupq_n_u32(0);

  for( ; iRows != 0; iRows-=iSubStep )
  {
    // v_uiSum1  += |piOrg - piCur|  -  [0; 3]
    v_uiSum1 = vabal_u16(
      v_uiSum1,
      vld1_u16((uint16_t *)&piOrg[0]),
      vld1_u16((uint16_t *)&piCur[0])
      );

    // v_uiSum2  += |piOrg - piCur|  -  [4; 7]
    v_uiSum2 = vabal_u16(
      v_uiSum2,
      vld1_u16((uint16_t *)&piOrg[4]),
      vld1_u16((uint16_t *)&piCur[4])
      );

    // v_uiSum3  += |piOrg - piCur|  -  [8; 11]
    v_uiSum3 = vabal_u16(
      v_uiSum3,
      vld1_u16((uint16_t *)&piOrg[8]),
      vld1_u16((uint16_t *)&piCur[8])
      );

    // v_uiSum4  += |piOrg - piCur|  -  [12; 15]
    v_uiSum4 = vabal_u16(
      v_uiSum4,
      vld1_u16((uint16_t *)&piOrg[12]),
      vld1_u16((uint16_t *)&piCur[12])
      );

    // v_uiSum5  += |piOrg - piCur|  -  [16; 19]
    v_uiSum5 = vabal_u16(
      v_uiSum5,
      vld1_u16((uint16_t *)&piOrg[16]),
      vld1_u16((uint16_t *)&piCur[16])
      );

    // v_uiSum6  += |piOrg - piCur|  -  [20; 23]
    v_uiSum6 = vabal_u16(
      v_uiSum6,
      vld1_u16((uint16_t *)&piOrg[20]),
      vld1_u16((uint16_t *)&piCur[20])
      );

    // v_uiSum7  += |piOrg - piCur|  -  [24; 27]
    v_uiSum7 = vabal_u16(
      v_uiSum7,
      vld1_u16((uint16_t *)&piOrg[24]),
      vld1_u16((uint16_t *)&piCur[24])
      );

    // v_uiSum8  += |piOrg - piCur|  -  [28; 31]
    v_uiSum8 = vabal_u16(
      v_uiSum8,
      vld1_u16((uint16_t *)&piOrg[28]),
      vld1_u16((uint16_t *)&piCur[28])
      );

    // v_uiSum9  += |piOrg - piCur|  -  [32; 35]
    v_uiSum9 = vabal_u16(
      v_uiSum9,
      vld1_u16((uint16_t *)&piOrg[32]),
      vld1_u16((uint16_t *)&piCur[32])
      );

    // v_uiSum10  += |piOrg - piCur|  -  [36; 39]
    v_uiSum10 = vabal_u16(
      v_uiSum10,
      vld1_u16((uint16_t *)&piOrg[36]),
      vld1_u16((uint16_t *)&piCur[36])
      );

    // v_uiSum11  += |piOrg - piCur|  -  [40; 43]
    v_uiSum11 = vabal_u16(
      v_uiSum11,
      vld1_u16((uint16_t *)&piOrg[40]),
      vld1_u16((uint16_t *)&piCur[40])
      );

    // v_uiSum12  += |piOrg - piCur|  -  [44; 47]
    v_uiSum12 = vabal_u16(
      v_uiSum12,
      vld1_u16((uint16_t *)&piOrg[44]),
      vld1_u16((uint16_t *)&piCur[44])
      );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  Distortion uiSum =
    vgetq_lane_u32(v_uiSum1,  0) + vgetq_lane_u32(v_uiSum2,  0) +
    vgetq_lane_u32(v_uiSum3,  0) + vgetq_lane_u32(v_uiSum4,  0) +
    vgetq_lane_u32(v_uiSum5,  0) + vgetq_lane_u32(v_uiSum6,  0) +
    vgetq_lane_u32(v_uiSum7,  0) + vgetq_lane_u32(v_uiSum8,  0) +
    vgetq_lane_u32(v_uiSum9,  0) + vgetq_lane_u32(v_uiSum10, 0) +
    vgetq_lane_u32(v_uiSum11, 0) + vgetq_lane_u32(v_uiSum12, 0) +
    vgetq_lane_u32(v_uiSum1,  1) + vgetq_lane_u32(v_uiSum2,  1) +
    vgetq_lane_u32(v_uiSum3,  1) + vgetq_lane_u32(v_uiSum4,  1) +
    vgetq_lane_u32(v_uiSum5,  1) + vgetq_lane_u32(v_uiSum6,  1) +
    vgetq_lane_u32(v_uiSum7,  1) + vgetq_lane_u32(v_uiSum8,  1) +
    vgetq_lane_u32(v_uiSum9,  1) + vgetq_lane_u32(v_uiSum10, 1) +
    vgetq_lane_u32(v_uiSum11, 1) + vgetq_lane_u32(v_uiSum12, 1) +
    vgetq_lane_u32(v_uiSum1,  2) + vgetq_lane_u32(v_uiSum2,  2) +
    vgetq_lane_u32(v_uiSum3,  2) + vgetq_lane_u32(v_uiSum4,  2) +
    vgetq_lane_u32(v_uiSum5,  2) + vgetq_lane_u32(v_uiSum6,  2) +
    vgetq_lane_u32(v_uiSum7,  2) + vgetq_lane_u32(v_uiSum8,  2) +
    vgetq_lane_u32(v_uiSum9,  2) + vgetq_lane_u32(v_uiSum10, 2) +
    vgetq_lane_u32(v_uiSum11, 2) + vgetq_lane_u32(v_uiSum12, 2) +
    vgetq_lane_u32(v_uiSum1,  3) + vgetq_lane_u32(v_uiSum2,  3) +
    vgetq_lane_u32(v_uiSum3,  3) + vgetq_lane_u32(v_uiSum4,  3) +
    vgetq_lane_u32(v_uiSum5,  3) + vgetq_lane_u32(v_uiSum6,  3) +
    vgetq_lane_u32(v_uiSum7,  3) + vgetq_lane_u32(v_uiSum8,  3) +
    vgetq_lane_u32(v_uiSum9,  3) + vgetq_lane_u32(v_uiSum10, 3) +
    vgetq_lane_u32(v_uiSum11, 3) + vgetq_lane_u32(v_uiSum12, 3);

  uiSum <<= iSubShift;
  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

Distortion TComRdCost::xGetSSE( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n++ ) // loop vectorized + peeled.
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE4( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 4 );
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- ) // loop vectorized.
  {

    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE8( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 8 );
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- ) // loop vectorized.
  {
    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[4] - piCur[4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[5] - piCur[5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[6] - piCur[6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[7] - piCur[7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE16( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 16 );
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- ) // loop vectorized.
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE16N( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (Int n = 0; n < iCols; n+=16 ) // loop vectorized.
    {

      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE32( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 32 );
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- ) // loop vectorized.
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion TComRdCost::xGetSSE64( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    assert( pcDtParam->iCols == 64 );
    return TComRdCostWeightPrediction::xGetSSEw( pcDtParam );
  }
  const Pel* piOrg   = pcDtParam->pOrg;
  const Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStrideCur = pcDtParam->iStrideCur;

  Distortion uiSum   = 0;
  UInt       uiShift = DISTORTION_PRECISION_ADJUSTMENT((pcDtParam->bitDepth-8) << 1);

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- ) // loop vectorized + peeled.
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[32] - piCur[32]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[33] - piCur[33]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[34] - piCur[34]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[35] - piCur[35]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[36] - piCur[36]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[37] - piCur[37]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[38] - piCur[38]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[39] - piCur[39]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[40] - piCur[40]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[41] - piCur[41]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[42] - piCur[42]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[43] - piCur[43]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[44] - piCur[44]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[45] - piCur[45]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[46] - piCur[46]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[47] - piCur[47]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[48] - piCur[48]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[49] - piCur[49]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[50] - piCur[50]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[51] - piCur[51]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[52] - piCur[52]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[53] - piCur[53]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[54] - piCur[54]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[55] - piCur[55]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[56] - piCur[56]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[57] - piCur[57]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[58] - piCur[58]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[59] - piCur[59]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[60] - piCur[60]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[61] - piCur[61]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[62] - piCur[62]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[63] - piCur[63]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

Distortion TComRdCost::xCalcHADs2x2( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Distortion satd = 0;
  TCoeff diff[4], m[4];
  assert( iStep == 1 );
  diff[0] = piOrg[0             ] - piCur[0];
  diff[1] = piOrg[1             ] - piCur[1];
  diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
  diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];

  satd += abs(m[0] + m[1]);
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);

  return satd;
}

Distortion TComRdCost::xCalcHADs4x4( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k;
  Distortion satd = 0;
  TCoeff diff[16], m[16], d[16];

  assert( iStep == 1 );
  for( k = 0; k < 16; k+=4 ) // loop vectorized
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];

  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];

  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];

  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];

  for (k=0; k<16; ++k) // loop vectorized
  {
    satd += abs(d[k]);
  }
  satd = ((satd+1)>>1);

  return satd;
}

Distortion TComRdCost::xCalcHADs8x8( Pel *piOrg, Pel *piCur, Int iStrideOrg, Int iStrideCur, Int iStep )
{
  Int k, i, j, jj;
  Distortion sad = 0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];
  assert( iStep == 1 );
  for( k = 0; k < 64; k += 8 ) // loop vectorized
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++) // loop vectorized
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++) // loop vectorized
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }

  sad=((sad+2)>>2);

  return sad;
}


Distortion TComRdCost::xGetHADs( DistParam* pcDtParam )
{
  if ( pcDtParam->bApplyWeight )
  {
    return TComRdCostWeightPrediction::xGetHADsw( pcDtParam );
  }
  Pel* piOrg   = pcDtParam->pOrg;
  Pel* piCur   = pcDtParam->pCur;
  Int  iRows   = pcDtParam->iRows;
  Int  iCols   = pcDtParam->iCols;
  Int  iStrideCur = pcDtParam->iStrideCur;
  Int  iStrideOrg = pcDtParam->iStrideOrg;
  Int  iStep  = pcDtParam->iStep;

  Int  x, y;

  Distortion uiSum = 0;

  if( ( iRows % 8 == 0) && (iCols % 8 == 0) ) // %%OPT rimuovi la %
  {
    Int  iOffsetOrg = iStrideOrg<<3;
    Int  iOffsetCur = iStrideCur<<3;
    for ( y=0; y<iRows; y+= 8 ) // %%OPT rimuovi la y
    {
      for ( x=0; x<iCols; x+= 8 )
      {
        uiSum += xCalcHADs8x8( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 4 == 0) && (iCols % 4 == 0) ) // %%OPT rimuovi la %
  {
    Int  iOffsetOrg = iStrideOrg<<2;
    Int  iOffsetCur = iStrideCur<<2;

    for ( y=0; y<iRows; y+= 4 ) // %%OPT rimuovi la y
    {
      for ( x=0; x<iCols; x+= 4 )
      {
        uiSum += xCalcHADs4x4( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else if( ( iRows % 2 == 0) && (iCols % 2 == 0) ) // %%OPT rimuovi la %
  {
    Int  iOffsetOrg = iStrideOrg<<1;
    Int  iOffsetCur = iStrideCur<<1;
    for ( y=0; y<iRows; y+=2 )
    {
      for ( x=0; x<iCols; x+=2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x*iStep], iStrideOrg, iStrideCur, iStep );
      }
      piOrg += iOffsetOrg;
      piCur += iOffsetCur;
    }
  }
  else
  {
    assert(false);
  }

  return ( uiSum >> DISTORTION_PRECISION_ADJUSTMENT(pcDtParam->bitDepth-8) );
}

//! \}
