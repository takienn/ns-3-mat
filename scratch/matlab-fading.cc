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
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 */


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include <ns3/string.h>
#include <fstream>
#include <ns3/buildings-helper.h>
#include <ns3/log.h>
#include <string>

//#include "ns3/gtk-config-store.h"

using namespace ns3;

//NS_LOG_COMPONENT_DEFINE("MatlabFading");

void
UpdateSinrPerceived (const SpectrumValue& sinr)
{

}

int main (int argc, char *argv[])
{	
  CommandLine cmd;

//  cmd.Parse (argc, argv);

  // to save a template default attribute file run it like this:
  // ./waf --command-template="%s --ns3::ConfigStore::Filename=input-defaults.txt --ns3::ConfigStore::Mode=Save --ns3::ConfigStore::FileFormat=RawText" --run src/lte/examples/lena-first-sim
  //
  // to load a previously created default attribute file
  // ./waf --command-template="%s --ns3::ConfigStore::Filename=input-defaults.txt --ns3::ConfigStore::Mode=Load --ns3::ConfigStore::FileFormat=RawText" --run src/lte/examples/lena-first-sim
//
//  ConfigStore inputConfig;
//  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  std::string fadingModelName = "ns3::MatlabLossModel";//"ns3::TraceFadingLossModel";//
  std::string fadingScenario = "EPA";
  cmd.AddValue("FadingModel","Fading Model",fadingModelName);
  cmd.AddValue("FadingScenario", "EPA or EVA", fadingScenario);
  cmd.Parse (argc, argv);


  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  // Uncomment to enable logging
//  lteHelper->EnableLogComponents ();
  lteHelper->SetAttribute ("FadingModel", StringValue (fadingModelName));
  if(fadingModelName == "ns3::TraceFadingLossModel")
    {
      if(fadingScenario.compare("EPA"))
	fadingScenario = "src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad";
      else if(fadingScenario.compare("EVA"))
	fadingScenario = "src/lte/model/fading-traces/fading_trace_EVA_60kmph.fad";
      else
	NS_FATAL_ERROR("Unsupported fading scenario");

	  // script launched as an example
	  lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue (fadingScenario));
    }
  
  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (1);
  ueNodes.Create (1);

  // Install Mobility Model
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> allocator = Create<ListPositionAllocator> ();
  allocator->Add(Vector(0.0,0.0,0.0));
  allocator->Add(Vector(0.0,300.0,0.0));
  mobility.SetPositionAllocator(allocator);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enbNodes);
  BuildingsHelper::Install (enbNodes);
//  Ptr<ConstantVelocityMobilityModel> mobModel = Create<ConstantVelocityMobilityModel>();
//  mobModel->SetVelocity(Vector(0,1.5,0));
//  mobModel->SetPosition(Vector(0,300,0));
//  ueNodes.Get(0)->AggregateObject(mobModel);
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install(ueNodes);
  Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
  mob->SetPosition(Vector(0,300,0));
  mob->SetVelocity(Vector(0,16.67,0));

  BuildingsHelper::Install (ueNodes);

  // Create Devices and install them in the Nodes (eNB and UE)
  NetDeviceContainer enbDevs;
  NetDeviceContainer ueDevs;
  enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  ueDevs = lteHelper->InstallUeDevice (ueNodes);

  // Attach a UE to a eNB
  lteHelper->Attach (ueDevs, enbDevs.Get (0));

  // Activate an EPS bearer
  enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
  EpsBearer bearer (q);
  lteHelper->ActivateDataRadioBearer (ueDevs, bearer);

  PointerValue tmp;
  Ptr<LteEnbNetDevice> enbDevice = DynamicCast<LteEnbNetDevice>(enbDevs.Get(0));
  enbDevice->GetAttribute("LteEnbPhy", tmp);
  Ptr<LteEnbPhy> enbPhy = DynamicCast<LteEnbPhy>(tmp.GetObject());
  Ptr<LteSpectrumPhy> dlPhy = enbPhy->GetDownlinkSpectrumPhy();

  Ptr<LteChunkProcessor> pData = Create<LteChunkProcessor> ();
  pData->AddCallback(MakeCallback(&UpdateSinrPerceived));
  dlPhy->AddCtrlSinrChunkProcessor(pData);

  Ptr<LteChunkProcessor> pCtrl = Create<LteChunkProcessor> ();
  pCtrl->AddCallback (MakeCallback (&UpdateSinrPerceived));
  dlPhy->AddCtrlSinrChunkProcessor (pCtrl);

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();

  Ptr<RadioBearerStatsCalculator> rlcCalc = lteHelper->GetRlcStats();
  rlcCalc->SetAttribute("EpochDuration", TimeValue(MilliSeconds(40.0)));

  Simulator::Stop (Seconds (5.005));

  Simulator::Run ();

  //GtkConfigStore config;
  //config.ConfigureAttributes ();

  Simulator::Destroy ();
  return 0;
}
