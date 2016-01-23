/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 *         Marco Miozzo <mmiozzo@cttc.es>
 */


#include <ns3/mobility-model.h>
#include <ns3/spectrum-value.h>
#include <ns3/log.h>
#include <ns3/string.h>
#include <ns3/double.h>
#include "ns3/uinteger.h"
#include <fstream>
#include <ns3/simulator.h>
#include "matlab-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MatlabLossModel");

NS_OBJECT_ENSURE_REGISTERED (MatlabLossModel);
  


MatlabLossModel::MatlabLossModel ()
  : m_streamsAssigned (false)
{
  NS_LOG_FUNCTION (this);
  SetNext (NULL);
}


MatlabLossModel::~MatlabLossModel ()
{
  m_fadingTrace.clear ();
  m_windowOffsetsMap.clear ();
  m_startVariableMap.clear ();
}


TypeId
MatlabLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MatlabLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName("Lte")
    .AddConstructor<MatlabLossModel> ()
    .AddAttribute ("TraceLength",
                  "The total length of the fading trace (default value 10 s.)",
                  TimeValue (Seconds (10.0)),
                  MakeTimeAccessor (&MatlabLossModel::SetTraceLength),
                  MakeTimeChecker ())
    .AddAttribute ("SamplesNum",
                  "The number of samples the trace is made of (default 10000)",
                   UintegerValue (10000),
                   MakeUintegerAccessor (&MatlabLossModel::m_samplesNum),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("WindowSize",
                  "The size of the window for the fading trace (default value 0.5 s.)",
                  TimeValue (Seconds (0.5)),
                  MakeTimeAccessor (&MatlabLossModel::m_windowSize),
                  MakeTimeChecker ())
    .AddAttribute ("RbNum",
                    "The number of RB the trace is made of (default 100)",
                    UintegerValue (100),
                   MakeUintegerAccessor (&MatlabLossModel::m_rbNum),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("RngStreamSetSize",
                    "The number of RNG streams reserved for the fading model. The maximum number of streams that are needed for an LTE FDD scenario is 2 * numUEs * numeNBs.",
                    UintegerValue (200000),
                   MakeUintegerAccessor (&MatlabLossModel::m_streamSetSize),
                   MakeUintegerChecker<uint64_t> ())
  ;
  return tid;
}

void
MatlabLossModel::SetTraceFileName (std::string fileName)
{
  NS_LOG_FUNCTION (this << "Set Fading Trace " << fileName);
  
  m_traceFile = fileName;
}

void 
MatlabLossModel::SetTraceLength (Time t)
{
  m_traceLength = t;
}

void 
MatlabLossModel::DoInitialize ()
{
	/*
	 * Call engOpen with a NULL string. This starts a MATLAB process
     * on the current host using the command "matlab".
	 */
	if (!(m_ep = engOpen(""))) {
		NS_LOG_ERROR("Can't start MATLAB engine");
	}
	NS_ASSERT(m_ep);

	engEvalString(m_ep, "c=3e8;"
						"fc = 1930e6;"
						"lambda = c/fc;"
						"v_km_h = 3.0;"
						"v_m_s = v_km_h / 3.6;"
						"fd = v_m_s / lambda;");
	engEvalString(m_ep, "delays_pedestrianEPA = [0 30e-9 70e-9 90e-9 120e-9 190e-9 410e-9];");
	engEvalString(m_ep, "power_pedestrianEPA = [0.0 -1.0 -2.0 -3.0 -8.0 -17.2 -20.8];");
//	engEvalString(m_ep, "fs=1.4e6 ;ts=1/fs;");
//	engEvalString(m_ep, "c = rayleighchan(ts, fd, delays_pedestrianEPA, power_pedestrianEPA);");
//	engEvalString(m_ep, "c.ResetBeforeFiltering = 0;c.NormalizePathGains = 1;");
//	engEvalString(m_ep, " TTI = 0.001; numSamples = TTI / ts;");
}


void
MatlabLossModel::LoadTrace ()
{
  NS_LOG_FUNCTION (this << "Loading Fading Trace " << m_traceFile);
  std::ifstream ifTraceFile;
  ifTraceFile.open (m_traceFile.c_str (), std::ifstream::in);
  m_fadingTrace.clear ();
  if (!ifTraceFile.good ())
    {
      NS_LOG_INFO (this << " File: " << m_traceFile);
      NS_ASSERT_MSG(ifTraceFile.good (), " Fading trace file not found");
    }

//   NS_LOG_INFO (this << " length " << m_traceLength.GetSeconds ());
//   NS_LOG_INFO (this << " RB " << (uint32_t)m_rbNum << " samples " << m_samplesNum);
  for (uint32_t i = 0; i < m_rbNum; i++)
    {
      FadingTraceSample rbTimeFadingTrace;
      for (uint32_t j = 0; j < m_samplesNum; j++)
        {
          double sample;
          ifTraceFile >> sample;
          rbTimeFadingTrace.push_back (sample);
        }
      m_fadingTrace.push_back (rbTimeFadingTrace);
    }
  m_timeGranularity = m_traceLength.GetMilliSeconds () / m_samplesNum;
  m_lastWindowUpdate = Simulator::Now ();
}


Ptr<SpectrumValue>
MatlabLossModel::DoCalcRxPowerSpectralDensity (
  Ptr<const SpectrumValue> txPsd,
  Ptr<const MobilityModel> a,
  Ptr<const MobilityModel> b) const
{
  NS_LOG_FUNCTION (this << *txPsd << a << b);
  

  double numBands = (double)txPsd->GetSpectrumModel()->GetNumBands();
  double FS = 1.4e6;

  if(numBands ==  6)
	  FS=1.4e6;
  if(numBands ==  15)
	  FS=3e6;
  if(numBands ==  25)
	  FS=5e6;
  if(numBands ==  50)
	  FS=10e6;
  if(numBands ==  75)
	  FS=15e6;
  if(numBands == 100)
	  FS=20e6;

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(txPsd);
  mxArray* psdrx = mxCreateDoubleMatrix(1,numBands, mxREAL);
  mxArray* numRBs = mxCreateDoubleScalar(numBands);
  mxArray* fs = mxCreateDoubleScalar(FS);

  engPutVariable(m_ep,"numRBs",numRBs);
  engPutVariable(m_ep,"fs", fs);

  engEvalString(m_ep,"ts = 1/fs;"
		  	  	  	 "TTI = 0.001;"
		  	  	  	 "numSamples = TTI / ts;");

  engEvalString(m_ep,"c = rayleighchan(ts, fd, delays_pedestrianEPA, power_pedestrianEPA);"
		  	  	  	 "c.ResetBeforeFiltering = 0;"
		  	  	  	 "c.NormalizePathGains = 1");

  engEvalString(m_ep,"sig = zeros(numSamples, 1);"
		  	  	  	 "sig(1) = 1;"
		  	  	  	 "[psdsig,F] = pwelch(sig,[],[],numRBs,fs,'twosided');");

  engEvalString(m_ep,"y = filter(c,sig);");
  engEvalString(m_ep,"[psdy,F] = pwelch(y,[],[],numRBs,fs);"
		  	  	  	 "psdy = psdy ./ psdsig;");

  psdrx = engGetVariable(m_ep, "psdy");
  double* fadingValues = (double *)mxGetData(psdrx);

  Values::iterator vit = rxPsd->ValuesBegin ();
//  Values::iterator fit = fadingValues->ValuesBegin();
  while (vit != rxPsd->ValuesEnd ())
  {
	  if(*vit == 0) continue;

	  double power = *vit;
	  power = 10 * std::log10 (180000 * power); // in dB
	  double fading = *fadingValues;
	  fading = 10 * std::log10 (fading); // in dB
	  *vit = std::pow (10., ((power + fading) / 10)) / 180000; // in Watt

	  ++vit;
	  ++fadingValues;
  }

  NS_LOG_LOGIC (this << *rxPsd);
  return rxPsd;
}

int64_t
MatlabLossModel::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  NS_ASSERT (m_streamsAssigned == false);  
  m_streamsAssigned = true;
  m_currentStream = stream;
  m_lastStream = stream + m_streamSetSize - 1;
  std::map <ChannelRealizationId_t, Ptr<UniformRandomVariable> >::iterator itVar;
  itVar = m_startVariableMap.begin ();
  // the following loop is for eventually pre-existing ChannelRealization instances
  // note that more instances are expected to be created at run time
  while (itVar!=m_startVariableMap.end ())
    {
      NS_ASSERT_MSG (m_currentStream <= m_lastStream, "not enough streams, consider increasing the StreamSetSize attribute");
      (*itVar).second->SetStream (m_currentStream);
      m_currentStream += 1;
    }
  return m_streamSetSize;
}



} // namespace ns3
