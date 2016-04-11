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


#ifndef MATLAB_LOSS_MODEL_H
#define MATLAB_LOSS_MODEL_H


#include <ns3/object.h>
#include <ns3/spectrum-propagation-loss-model.h>
#include <map>
#include "ns3/random-variable-stream.h"
#include <ns3/nstime.h>
#include "engine.h"

namespace ns3 {


class MobilityModel;

/**
 * \ingroup lte
 *
 * \brief fading loss model based on precalculated fading traces
 */
class MatlabLossModel : public SpectrumPropagationLossModel
{
public:
  MatlabLossModel ();
  virtual ~MatlabLossModel ();

  static TypeId GetTypeId ();
  
  virtual void DoInitialize (void);

  
private:
  /**
   * \param txPsd set of values vs frequency representing the
   *              transmission power. See SpectrumChannel for details.
   * \param a sender mobility
   * \param b receiver mobility
   * \return set of values vs frequency representing the received
   *         power in the same units used for the txPsd parameter.
   */
  Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                   Ptr<const MobilityModel> a,
                                                   Ptr<const MobilityModel> b) const;
                                                   
  
  void SetFadingScenario(std::string scenario);

  std::string m_fadingScenario;


  
  Engine  *m_ep;

};



} // namespace ns3

#endif /* MATLAB_LOSS_MODEL_H */
