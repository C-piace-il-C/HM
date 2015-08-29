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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <thread>
#include <time.h>
#include <iostream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"

//! \ingroup TAppEncoder
//! \{

#include "../Lib/TLibCommon/Debug.h"


void encoding_thread(int argc, char** argv)
{
  TAppEncTop cTAppEncTop;
  cTAppEncTop.create();
  cTAppEncTop.parseCfg(argc, argv);
  cTAppEncTop.encode();
}


// ====================================================================================================================
// Main function
// ====================================================================================================================
int cfgGetParam(char* configureFilename, char* paramName);
int main(int argc, char* argv[])
{
  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "HM software: Encoder Version [%s] (including RExt)", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n\n" );

  // analyze configure files
  int cfg_cnt = 0;
  int IntraPeriod, FrameSkip, FramesToBeEncoded;
  std::string sequence_cfg;
  std::string coding_cfg;
  for (int C = 0; C < argc; C++)
  {
    if (strcmp("-c", argv[C]) == 0)
    {
      cfg_cnt++;
      if (cfg_cnt == 1)
      {
        sequence_cfg = argv[C + 1];
        FrameSkip = cfgGetParam(argv[C + 1], "FrameSkip");
        FramesToBeEncoded = cfgGetParam(argv[C + 1], "FramesToBeEncoded");
      }
      else if (cfg_cnt == 2)
      {
        coding_cfg = argv[C + 1];
        IntraPeriod = cfgGetParam(argv[C + 1], "IntraPeriod");
      }
      
    }
  }

  // build two new configure files
  int periodCount   = FramesToBeEncoded / IntraPeriod;
  int frameCount_t1 = periodCount / 2 * IntraPeriod;
  int frameCount_t0 = FramesToBeEncoded - frameCount_t1;
  printf("multithreading settings:\nmain thread encodes frames\t[0,%i]\n",frameCount_t0-1);
  printf("secondary thread encodes frames\t[%i, %i]\n", frameCount_t0, frameCount_t0 + frameCount_t1 - 1);
  std::ifstream in0(sequence_cfg);
  std::ofstream out0(sequence_cfg + "0");
  std::ofstream out1(sequence_cfg + "1");
  std::string line;
  while (std::getline(in0, line))
  {
    if (line.find("FrameSkip") != string::npos) // correct line
    {
      out0 << "FrameSkip                     : 0\n";
      out1 << "FrameSkip                     : " << frameCount_t0 << "\n";
    }
    else if (line.find("FramesToBeEncoded") != string::npos)
    {
      out0 << "FramesToBeEncoded             : " << frameCount_t0 << "\n";
      out1 << "FramesToBeEncoded             : " << frameCount_t1 << "\n";
    }
    else
    {
      out0 << line << "\n";
      out1 << line << "\n";
    }
  }
  out0.close();
  out1.close();
  std::ifstream in1(coding_cfg);
  std::ofstream cod0(coding_cfg + "0");
  std::ofstream cod1(coding_cfg + "1");
  while (std::getline(in1, line))
  {
    if (line.find("BitstreamFile") != string::npos) // correct line
    {
      cod0 << "BitstreamFile                 : str0.bin \n";
      cod1 << "BitstreamFile                 : str1.bin \n";
    }
    else if (line.find("ReconFile") != string::npos)
    {
      cod0 << "ReconFile                     : rec0.yuv \n";
      cod1 << "ReconFile                     : rec1.yuv \n";
    }
    else
    {
      cod0 << line << "\n";
      cod1 << line << "\n";
    }
  }
  cod0.close();
  cod1.close();
  // build two new argvs
  char* seq_cfg0 = new char[sequence_cfg.length() + 2];
  sprintf(seq_cfg0, "%s0", sequence_cfg.c_str());
  char* seq_cfg1 = new char[sequence_cfg.length() + 2];
  sprintf(seq_cfg1, "%s1", sequence_cfg.c_str());
  char* cod_cfg0 = new char[coding_cfg.length() + 2];
  sprintf(cod_cfg0, "%s0", coding_cfg.c_str());
  char* cod_cfg1 = new char[coding_cfg.length() + 2];
  sprintf(cod_cfg1, "%s1", coding_cfg.c_str());
  char* argv0[] = { argv[0], argv[1], seq_cfg0, argv[3], cod_cfg0 };
  char* argv1[] = { argv[0], argv[1], seq_cfg1, argv[3], cod_cfg1 };



  // starting time
  Double dResult;
  clock_t lBefore = clock();

  // multithreaded execution
  std::thread(encoding_thread, argc, argv1).detach();
  TAppEncTop cTAppEncTop;
  cTAppEncTop.create();
  cTAppEncTop.parseCfg(argc, argv0);
  cTAppEncTop.encode();

  // ending time
  dResult = (Double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);


  return 0;
}

int cfgGetParam(char* configureFilename, char* paramName)
{
  std::ifstream file(configureFilename);
  std::string str;
  while (std::getline(file, str))
  {
    if (str.find(paramName) != string::npos) // correct line
    {
      std::string paramValue = str.substr(str.find(":") + 1);
      file.close();
      return(stoi(paramValue));
    }
  }
  return(16);
}

//! \}
