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
	Engine *ep;
	if (!(ep = engOpen(""))) {
		NS_LOG_ERROR("Can't start MATLAB engine");
	}
	m_ep = ep;
	NS_ASSERT(m_ep);

	engEvalString(m_ep, "c=3e8;fc = 1930e6;lambda = c/fc;v_km_h = 3.0;v_m_s = v_km_h / 3.6;fd = v_m_s / lambda;");
	engEvalString(m_ep, "delays_pedestrianEPA = [0 30e-9 70e-9 90e-9 120e-9 190e-9 410e-9];");
	engEvalString(m_ep, "power_pedestrianEPA = [0.0 -1.0 -2.0 -3.0 -8.0 -17.2 -20.8];");
	engEvalString(m_ep, "fs=20e6;ts=1/fs;");
	engEvalString(m_ep, "c = rayleighchan(ts, fd, delays_pedestrianEPA, power_pedestrianEPA);");
	engEvalString(m_ep, "c.ResetBeforeFiltering = 0;c.NormalizePathGains = 1; TTI = 0.001;");
	engEvalString(m_ep, "numSamples = TTI / ts;");
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
  
//  Engine* ep = (Engine*) m_ep;

  int numSamples = txPsd->GetSpectrumModel()->GetNumBands();
  mxArray* sig = mxCreateDoubleMatrix(numSamples, 1, mxREAL);
  mxArray* y = mxCreateDoubleMatrix(numSamples, 1, mxREAL);
  memcpy((void *)mxGetPr(sig), (void *)&txPsd[0], numSamples);

  engPutVariable(m_ep, "sig", sig);
  engPutVariable(m_ep, "y", y);

  engEvalString(m_ep,"y = filter(c,sig);");

  sig = engGetVariable(m_ep, "y");

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);

  memcpy((void *)&rxPsd[0],(void *)mxGetPr(sig), numSamples);
//  std::map <ChannelRealizationId_t, int >::iterator itOff;
//  ChannelRealizationId_t mobilityPair = std::make_pair (a,b);
//  itOff = m_windowOffsetsMap.find (mobilityPair);
//  if (itOff!=m_windowOffsetsMap.end ())
//    {
//      if (Simulator::Now ().GetSeconds () >= m_lastWindowUpdate.GetSeconds () + m_windowSize.GetSeconds ())
//        {
//          // update all the offsets
//          NS_LOG_INFO ("Fading Windows Updated");
//          std::map <ChannelRealizationId_t, int >::iterator itOff2;
//          for (itOff2 = m_windowOffsetsMap.begin (); itOff2 != m_windowOffsetsMap.end (); itOff2++)
//            {
//              std::map <ChannelRealizationId_t, Ptr<UniformRandomVariable> >::iterator itVar;
//              itVar = m_startVariableMap.find ((*itOff2).first);
//              (*itOff2).second = (*itVar).second->GetValue ();
//            }
//          m_lastWindowUpdate = Simulator::Now ();
//        }
//    }
//  else
//    {
//      NS_LOG_LOGIC (this << "insert new channel realization, m_windowOffsetMap.size () = " << m_windowOffsetsMap.size ());
//      Ptr<UniformRandomVariable> startV = CreateObject<UniformRandomVariable> ();
//      startV->SetAttribute ("Min", DoubleValue (1.0));
//      startV->SetAttribute ("Max", DoubleValue ((m_traceLength.GetSeconds () - m_windowSize.GetSeconds ()) * 1000.0));
//      if (m_streamsAssigned)
//        {
//          NS_ASSERT_MSG (m_currentStream <= m_lastStream, "not enough streams, consider increasing the StreamSetSize attribute");
//          startV->SetStream (m_currentStream);
//          m_currentStream += 1;
//        }
//      ChannelRealizationId_t mobilityPair = std::make_pair (a,b);
//      m_startVariableMap.insert (std::pair<ChannelRealizationId_t,Ptr<UniformRandomVariable> > (mobilityPair, startV));
//      m_windowOffsetsMap.insert (std::pair<ChannelRealizationId_t,int> (mobilityPair, startV->GetValue ()));
//    }
//
//

  Values::iterator vit = rxPsd->ValuesBegin ();
  
  //Vector aSpeedVector = a->GetVelocity ();
  //Vector bSpeedVector = b->GetVelocity ();
  
  //double speed = std::sqrt (std::pow (aSpeedVector.x-bSpeedVector.x,2) + std::pow (aSpeedVector.y-bSpeedVector.y,2));

  NS_LOG_LOGIC (this << *rxPsd);
//  NS_ASSERT (!m_fadingTrace.empty ());
//  int now_ms = static_cast<int> (Simulator::Now ().GetMilliSeconds () * m_timeGranularity);
//  int lastUpdate_ms = static_cast<int> (m_lastWindowUpdate.GetMilliSeconds () * m_timeGranularity);
//  int index = ((*itOff).second + now_ms - lastUpdate_ms) % m_samplesNum;
//  int subChannel = 0;
//  while (vit != rxPsd->ValuesEnd ())
//    {
//      NS_ASSERT (subChannel < 100);
//      if (*vit != 0.)
//        {
//          double fading = m_fadingTrace.at (subChannel).at (index);
//          NS_LOG_INFO (this << " FADING now " << now_ms << " offset " << (*itOff).second << " id " << index << " fading " << fading);
//          double power = *vit; // in Watt/Hz
//          power = 10 * std::log10 (180000 * power); // in dB
//
//          NS_LOG_LOGIC (this << subChannel << *vit  << power << fading);
//
//          *vit = std::pow (10., ((power + fading) / 10)) / 180000; // in Watt
//
//          NS_LOG_LOGIC (this << subChannel << *vit);
//
//        }
//
//      ++vit;
//      ++subChannel;
//
//    }

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
