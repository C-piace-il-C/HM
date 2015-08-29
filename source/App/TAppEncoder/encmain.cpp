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

//#include <thread>
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
  for (int C = 0; C < argc; C++)
  {
    if (strcmp("-c", argv[C]) == 0)
    {
      cfg_cnt++;
      if (cfg_cnt == 1)
      {
        FrameSkip = cfgGetParam(argv[C + 1], "FrameSkip");
        FramesToBeEncoded = cfgGetParam(argv[C + 1], "FramesToBeEncoded");
      }
      else if (cfg_cnt == 2)
        IntraPeriod = cfgGetParam(argv[C + 1], "IntraPeriod");
    }
  }




  // starting time
  Double dResult;
  clock_t lBefore = clock();

  // multithreaded execution
  


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
