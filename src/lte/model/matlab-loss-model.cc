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
{
  NS_LOG_FUNCTION (this);
  SetNext (NULL);
}


MatlabLossModel::~MatlabLossModel ()
{

}


TypeId
MatlabLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MatlabLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName("lte")
    .AddConstructor<MatlabLossModel> ()
    .AddAttribute ("FadingScenario",
                    "The number of RB the trace is made of (default 100)",
                    StringValue ("EPA"),
                   MakeStringAccessor(&MatlabLossModel::m_fadingScenario),
                   MakeStringChecker())
  ;
  return tid;
}

void
MatlabLossModel::SetFadingScenario (std::string scenario)
{
  NS_LOG_FUNCTION (this << "Set Fading Scenario" << scenario);
  
  m_fadingScenario = scenario;
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
//	engEvalString(m_ep, "delays_pedestrianEPA = [0 30e-9 70e-9 90e-9 120e-9 190e-9 410e-9];");
//	engEvalString(m_ep, "power_pedestrianEPA = [0.0 -1.0 -2.0 -3.0 -8.0 -17.2 -20.8];");
}


Ptr<SpectrumValue>
MatlabLossModel::DoCalcRxPowerSpectralDensity (
  Ptr<const SpectrumValue> txPsd,
  Ptr<const MobilityModel> a,
  Ptr<const MobilityModel> b) const
{
  NS_LOG_FUNCTION (this << *txPsd << a << b);
  
  double numBands = (double)txPsd->GetSpectrumModel()->GetNumBands();
  double FS = 1.92e6;

  if(numBands ==  6)
	  FS=1.92e6;
  if(numBands ==  15)
	  FS=3.84e6;
  if(numBands ==  25)
	  FS=7.68e6;
  if(numBands ==  50)
	  FS=15.36e6;
  if(numBands ==  75)
	  FS=23.04e6;
  if(numBands == 100)
	  FS=30.72e6;

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(txPsd);
  mxArray* psdrx = mxCreateDoubleMatrix(1,numBands, mxREAL);
  mxArray* numRBs = mxCreateDoubleScalar(numBands);
  mxArray* fs = mxCreateDoubleScalar(FS);
  mxArray* scenario = mxCreateString(m_fadingScenario.c_str());
  Time NOW = Now();
  mxArray* now = mxCreateDoubleScalar((double)NOW.GetDouble()*1e6);

  /*
   * The LTE model generates a spectrum model with NumBands that corresponds to the number of resource
   * blocks used for that specific transmission (bandwidth)
   */
  engPutVariable(m_ep, "scenario", scenario);
  engPutVariable(m_ep, "now", now);
  engPutVariable(m_ep,"numRBs",numRBs);
  engPutVariable(m_ep,"fs", fs);

  engEvalString(m_ep, "chcfg.DelayProfile = scenario;"
		      "chcfg.NRxAnts = 1;"
		      "chcfg.DopplerFreq = 5;"
		      "chcfg.MIMOCorrelation = 'Low';"
		      "chcfg.SamplingRate = 20e6;"
		      "chcfg.Seed = 1;"
		      "chcfg.InitPhase = 'Random';"
		      "chcfg.ModelType = 'GMEDS';"
		      "chcfg.NTerms = 16;"
		      "chcfg.NormalizeTxAnts = 'On';"
		      "chcfg.NormalizePathGains = 'On';"
		      "chcfg.InitTime = now;");

  engEvalString(m_ep,"ts = 1/fs;"
		     "TTI = 0.001;"
		     "numSamples = TTI / ts;");

  engEvalString(m_ep,"sig = zeros(numSamples, 1);"
		     "sig(1) = 1;"
		     "[psdsig,F] = pwelch(sig,[],[],numRBs,fs,'twosided');");

//  engEvalString(m_ep,"y = filter(c,sig);");
  engEvalString(m_ep,"y = lteFadingChannel(chcfg, sig);");
  engEvalString(m_ep,"[psdy,F] = pwelch(y,[],[],numRBs,fs);"
		     "psdy = psdy ./ psdsig;");

  psdrx = engGetVariable(m_ep, "psdy");
  double* fadingValues = (double *)mxGetData(psdrx);

  Values::iterator vit = rxPsd->ValuesBegin ();
//  Values::iterator fit = fadingValues->ValuesBegin();
  while (vit != rxPsd->ValuesEnd ())
  {
	  if(*vit == 0)
	    {
	      ++vit;
	      continue;
	    }

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



} // namespace ns3
