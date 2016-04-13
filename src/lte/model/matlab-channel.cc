#include <ns3/log.h>
#include <ns3/lte-spectrum-signal-parameters.h>
#include <ns3/lte-control-messages.h>
#include <ns3/packet-burst.h>
#include <ns3/spectrum-value.h>
#include <ns3/lte-spectrum-value-helper.h>
#include <ns3/spectrum-phy.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device.h>
#include <ns3/assert.h>
#include <ns3/node.h>
#include <ns3/lte-rrc-header.h>
#include <ns3/lte-rrc-sap.h>

#include <list>
#include "matlab-channel.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MatlabChannel");

NS_OBJECT_ENSURE_REGISTERED (MatlabChannel);

Engine* MatlabChannel::m_ep = NULL;

TypeId
MatlabChannel::GetTypeId()
{
  static TypeId tid = TypeId("ns3::MatlabChannel")
    .SetParent<SpectrumChannel> ()
    .SetGroupName("lte")
    .AddConstructor<MatlabChannel> ();

  return tid;
}

MatlabChannel::~MatlabChannel ()
{
}




/*
 * In the MatlabChannel constructor we initialize network wide settings and parameters
 * that are fixed throughout the simulation, for instance
 */
MatlabChannel::MatlabChannel()
{
  if(!m_ep)
    {
      m_ep = engOpen("");
      NS_ASSERT(m_ep);
      engEvalString(m_ep, "ENB.NDLRB = 25;\n"
	"ENB.CellRefP = 1;\n"
	"ENB.NCellID = 0;\n"
	"ENB.CyclicPrefix = 'Normal';\n"
	"ENB.PHICHDuration = 'Normal';\n"
	"ENB.DuplexMode = 'FDD';\n"
	"ENB.CFI = 3;\n"
	"ENB.Ng = 'Sixth';\n"
	"ENB.DuplexMode = 'FDD';\n"
	"dciConfig.DCIFormat = 'Format1A';\n"
	"dciConfig.Allocation.RIV = 26;\n"
	"chcfg.DelayProfile = 'EPA';\n"
       "chcfg.NRxAnts = 1;\n"
       "chcfg.DopplerFreq = 5;\n"
       "chcfg.MIMOCorrelation = 'Low';\n"
       "chcfg.Seed = 1;\n"
       "chcfg.InitPhase = 'Random';\n"
       "chcfg.ModelType = 'GMEDS';\n"
       "chcfg.NTerms = 16;\n"
       "chcfg.NormalizeTxAnts = 'On';\n"
       "chcfg.NormalizePathGains = 'On';\n");

      engEvalString(m_ep, "hSpecAnalTx = dsp.SpectrumAnalyzer('SampleRate', 7.68e6, "
	  "'SpectrumType', 'Power density', 'PowerUnits', 'dBm', "
	  "'RBWSource', 'Property',   'RBW', 15e3, "
	  "'FrequencySpan', 'Span and center frequency', "
	  "'Span', 7.68e6, 'CenterFrequency', 0, "
	  "'Window', 'Rectangular', 'SpectralAverages', 10, "
	  "'YLimits', [-100 -60], "
	  "'YLabel', 'PSD', "
	  "'Title', 'NS-3 Generated Signal', "
	  "'ShowLegend', false);");
    }


}

/*
 * The entry point to MATLAB interfacing with ns-3
 * txParams contains all the necessary information for signal (subframe) description and will be examined to convert
 * all its content to MATLAB LTE Toolbox structures.
 */
void
MatlabChannel::StartTx (Ptr<SpectrumSignalParameters> txParams)
{
    NS_LOG_FUNCTION(this << txParams);

    NS_ASSERT(txParams->txPhy);
    NS_ASSERT(txParams->psd);

    Ptr<MobilityModel> txMobility = txParams->txPhy->GetMobility ();
    SpectrumModelUid_t txSpectrumModelUid =
	txParams->psd->GetSpectrumModelUid ();
    NS_LOG_LOGIC(" txSpectrumModelUid " << txSpectrumModelUid);

    //
    TxSpectrumModelInfoMap_t::const_iterator txInfoIteratorerator = FindAndEventuallyAddTxSpectrumModel (txParams->psd->GetSpectrumModel ());
    NS_ASSERT(txInfoIteratorerator != m_txSpectrumModelInfoMap.end ());

    NS_LOG_LOGIC("converter map for TX SpectrumModel with Uid " << txInfoIteratorerator->first);
    NS_LOG_LOGIC("converter map size: " << txInfoIteratorerator->second.m_spectrumConverterMap.size ());
    NS_LOG_LOGIC("converter map first element: " << txInfoIteratorerator->second.m_spectrumConverterMap.begin ()->first);


    Ptr <const SpectrumValue> rxPsd = txParams->psd;
    Time duration = txParams->duration;

    // the device might start RX only if the signal is of a type
    // understood by this device - in this case, an LTE signal.
    Ptr<LteSpectrumSignalParametersDataFrame> lteDataRxParams = DynamicCast<LteSpectrumSignalParametersDataFrame> (txParams);
    Ptr<LteSpectrumSignalParametersDlCtrlFrame> lteDlCtrlRxParams = DynamicCast<LteSpectrumSignalParametersDlCtrlFrame> (txParams);
    Ptr<LteSpectrumSignalParametersUlSrsFrame> lteUlSrsRxParams = DynamicCast<LteSpectrumSignalParametersUlSrsFrame> (txParams);
    Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(txParams->txPhy->GetDevice());
    Ptr<LteUeNetDevice> ueDev = DynamicCast<LteUeNetDevice>(txParams->txPhy->GetDevice());

    NS_LOG_LOGIC (" copying signal parameters " << txParams);
    Ptr<SpectrumSignalParameters> rxParams;

    bool isEnb;
    if(ueDev)
      isEnb = 0;
    else if (enbDev)
      isEnb = 1;
    else
      NS_FATAL_ERROR("Unknown Lte Device");

    if (lteDlCtrlRxParams != 0)
      {
	if(isEnb)
	  {
	    uint8_t dlBw = enbDev->GetDlBandwidth();

	    double dl_fc = LteSpectrumValueHelper::GetCarrierFrequency(enbDev->GetDlEarfcn());
	    double ul_fc = LteSpectrumValueHelper::GetCarrierFrequency(enbDev->GetUlEarfcn());
	    engPutVariable(m_ep,"NFrame", mxCreateDoubleScalar(lteDlCtrlRxParams->frameN));
	    engPutVariable(m_ep,"NSubframe", mxCreateDoubleScalar(lteDlCtrlRxParams->subframeN));
	    engPutVariable(m_ep,"DL_FC", mxCreateDoubleScalar(dl_fc));
	    engPutVariable(m_ep,"UL_FC", mxCreateDoubleScalar(ul_fc));
	    engPutVariable(m_ep,"dlBw", mxCreateDoubleScalar((double)dlBw));
	    engEvalString(m_ep, "ENB.NDLRB = dlBw;"
				"ENB.NFrame = NFrame;"
				"ENB.NSubframe = NSubframe;"
				"subframe = lteDLResourceGrid(ENB);"
				"[dciMessage, dciMessageBits] = lteDCI(ENB, dciConfig);"
				"C_RNTI = 100;"
				"pdcchConfig.RNTI = C_RNTI;"
				"pdcchConfig.PDCCHFormat = 0;"
				"codedDciBits = lteDCIEncode(pdcchConfig, dciMessageBits);"
				"pdcchDims = ltePDCCHInfo(ENB);"
				"pdcchBits = -1*ones(pdcchDims.MTot, 1);"
				"candidates = ltePDCCHSpace(ENB, pdcchConfig, {'bits', '1based'});"
				"pdcchBits ( candidates(1, 1) : candidates(1, 2) ) = codedDciBits;"
				"pdcchSymbols = ltePDCCH(ENB, pdcchBits);"
				"pdcchIndices = ltePDCCHIndices(ENB,{'1based'});"
				"subframe(pdcchIndices) = pdcchSymbols;");
	    double FS = 1.92e6;

	    if(dlBw ==  6)
	            FS=1.92e6;
	    if(dlBw ==  15)
	            FS=3.84e6;
	    if(dlBw ==  25)
	            FS=7.68e6;
	    if(dlBw ==  50)
	            FS=15.36e6;
	    if(dlBw ==  75)
	            FS=23.04e6;
	    if(dlBw == 100)
	            FS=30.72e6;
	    engPutVariable(m_ep,"FS", mxCreateDoubleScalar(FS));

	  }
	else
	  {

	  }

	//Generate Data Frame in MATLAB
	SerializeCtrlParams (lteDlCtrlRxParams);
	PassThroughChannel ();
	rxParams = DeserializeCtlrParams(lteDlCtrlRxParams);
//	rxParams = txParams->Copy();
      }
    else if (lteDataRxParams!=0)
      {
	if(isEnb)
	  {
	    uint8_t dlBw = enbDev->GetDlBandwidth();

	    double dl_fc = LteSpectrumValueHelper::GetCarrierFrequency(enbDev->GetDlEarfcn());
	    double ul_fc = LteSpectrumValueHelper::GetCarrierFrequency(enbDev->GetUlEarfcn());
	    engPutVariable(m_ep,"NFrame", mxCreateDoubleScalar(lteDataRxParams->frameN));
	    engPutVariable(m_ep,"NSubframe", mxCreateDoubleScalar(lteDataRxParams->subframeN));
	    engPutVariable(m_ep,"DL_FC", mxCreateDoubleScalar(dl_fc));
	    engPutVariable(m_ep,"UL_FC", mxCreateDoubleScalar(ul_fc));
	    engPutVariable(m_ep,"dlBw", mxCreateDoubleScalar((double)dlBw));
	    engEvalString(m_ep, "ENB.NDLRB = dlBw;"
				"ENB.NFrame = NFrame;"
				"ENB.NSubframe = NSubframe;"
				"subframe = lteDLResourceGrid(ENB);");

	    double FS = 1.92e6;

	    if(dlBw ==  6)
	            FS=1.92e6;
	    if(dlBw ==  15)
	            FS=3.84e6;
	    if(dlBw ==  25)
	            FS=7.68e6;
	    if(dlBw ==  50)
	            FS=15.36e6;
	    if(dlBw ==  75)
	            FS=23.04e6;
	    if(dlBw == 100)
	            FS=30.72e6;
	    engPutVariable(m_ep,"FS", mxCreateDoubleScalar(FS));

	  }
	else
	  {

	  }

	//Generate Ctrl Frame in MATLAB
	SerializeDataParams (lteDataRxParams);
	PassThroughChannel ();
	rxParams = DeserializeDataParams(lteDataRxParams);
//	rxParams = txParams->Copy();
      }
    else if (lteUlSrsRxParams!=0)
      {
	//Generate UL SRS in Matlab
	SerializeSrsParams (lteUlSrsRxParams);
	PassThroughChannel ();
	rxParams = DeserializeUlSrsParams(lteUlSrsRxParams);
      }
    else
      {
	//NOT LTE
      }


    for (RxSpectrumModelInfoMap_t::const_iterator rxInfoIterator = m_rxSpectrumModelInfoMap.begin ();
         rxInfoIterator != m_rxSpectrumModelInfoMap.end ();
         ++rxInfoIterator)
      {
        SpectrumModelUid_t rxSpectrumModelUid = rxInfoIterator->second.m_rxSpectrumModel->GetUid ();
        NS_LOG_LOGIC (" rxSpectrumModelUids " << rxSpectrumModelUid);

        Ptr <SpectrumValue> convertedTxPowerSpectrum;
        if (txSpectrumModelUid == rxSpectrumModelUid)
          {
            NS_LOG_LOGIC ("no spectrum conversion needed");
            convertedTxPowerSpectrum = txParams->psd;
          }
        else
          {
            NS_LOG_LOGIC (" converting txPowerSpectrum SpectrumModelUids" << txSpectrumModelUid << " --> " << rxSpectrumModelUid);
            SpectrumConverterMap_t::const_iterator rxConverterIterator = txInfoIteratorerator->second.m_spectrumConverterMap.find (rxSpectrumModelUid);
            NS_ASSERT (rxConverterIterator != txInfoIteratorerator->second.m_spectrumConverterMap.end ());
            convertedTxPowerSpectrum = rxConverterIterator->second.Convert (txParams->psd);
          }


        for (std::set<Ptr<SpectrumPhy> >::const_iterator rxPhyIterator = rxInfoIterator->second.m_rxPhySet.begin ();
             rxPhyIterator != rxInfoIterator->second.m_rxPhySet.end ();
             ++rxPhyIterator)
          {
            NS_ASSERT_MSG ((*rxPhyIterator)->GetRxSpectrumModel ()->GetUid () == rxSpectrumModelUid,
                           "SpectrumModel change was not notified to MultiModelSpectrumChannel (i.e., AddRx should be called again after model is changed)");

            if ((*rxPhyIterator) != txParams->txPhy)
              {
                rxParams->psd = Copy<SpectrumValue> (convertedTxPowerSpectrum);
                Time delay = MicroSeconds (0);

                Ptr<NetDevice> netDev = (*rxPhyIterator)->GetDevice ();
                if (netDev)
                  {
                    // the receiver has a NetDevice, so we expect that it is attached to a Node
                    uint32_t dstNode =  netDev->GetNode ()->GetId ();
                    Simulator::ScheduleWithContext (dstNode, delay, &MatlabChannel::StartRx, this,
                                                    rxParams, *rxPhyIterator);
                  }
                else
                  {
                    // the receiver is not attached to a NetDevice, so we cannot assume that it is attached to a node
                    Simulator::Schedule (delay, &MatlabChannel::StartRx, this,
                                         rxParams, *rxPhyIterator);
                  }
              }
          }

      }
}

void
MatlabChannel::SerializeDataParams (Ptr<LteSpectrumSignalParametersDataFrame> params)
{
  std::list<Ptr<LteControlMessage> > ctrlMsgList = params->ctrlMsgList;
  Ptr<PacketBurst> packetBurst = params->packetBurst;
  uint16_t cellId = params->cellId;

  NS_ASSERT(m_ep);

  engPutVariable(m_ep, "CELLID", mxCreateDoubleScalar((double) cellId));
  engEvalString(m_ep,"ENB.NCellID = CELLID;");

  SerializeCtlrMessages(ctrlMsgList);
}

/*
 * Starting ns-3 data serialization to Matlab.
 */
void
MatlabChannel::SerializeCtrlParams (Ptr<LteSpectrumSignalParametersDlCtrlFrame> params)
{
  std::list<Ptr<LteControlMessage> > ctrlMsgList = params->ctrlMsgList;
  uint16_t cellId = params->cellId;
  bool pss = params->pss;

  NS_ASSERT(m_ep);

  engPutVariable(m_ep, "CELLID", mxCreateDoubleScalar((double) cellId));
  engEvalString(m_ep,"ENB.NCellID = CELLID;\n");
  engEvalString(m_ep, "rs = lteCellRS(ENB);\n"
		      "rsIndices = lteCellRSIndices(ENB);\n"
		      "subframe(rsIndices) = rs;\n");
  if(pss)
    {
      engEvalString(m_ep, "pss = ltePSS(ENB);\n"
	  "pssIndices = ltePSSIndices(ENB);\n"
	  "subframe(pssIndices) = pss;\n"
	  "sss = lteSSS(ENB);\n"
	  "sssIndices = lteSSSIndices(ENB);\n"
	  "subframe(sssIndices) = sss;\n");
    }

  SerializeCtlrMessages(ctrlMsgList);
}

void
MatlabChannel::SerializeSrsParams (Ptr<LteSpectrumSignalParametersUlSrsFrame> params)
{
//TODO To be implemented for SRS UL handling
}


//Serializing Control messages other than the reference signals (these are handled in the calling function)
void
MatlabChannel::SerializeCtlrMessages(std::list<Ptr<LteControlMessage> > ctrlMsgList)
{

  NS_ASSERT(m_ep);

  std::list<Ptr<LteControlMessage> >::iterator it = ctrlMsgList.begin();
  for(; it != ctrlMsgList.end(); ++it)
    {
      Ptr<LteControlMessage> msg = *it;
      switch(msg->GetMessageType())
      {
	case LteControlMessage::BSR:
	  //TODO BSR message handling
	  break;
	case LteControlMessage::DL_CQI:
	  {
	    //TODO DL_CQI message handling
//	    Ptr<DlCqiLteControlMessage> cqiMsg = StaticCast<DlCqiLteControlMessage> (msg);
//	    CqiListElement_s cqiElements = cqiMsg->GetDlCqi();
//	    cqiElements.m_cqiType;
	    break;
	  }
	case LteControlMessage::DL_DCI:
	  {
	    //TODO DL_DCI Proper message handling
//	          Ptr<DlDciLteControlMessage> msg2 = DynamicCast<DlDciLteControlMessage> (msg);
//
//	          DlDciListElement_s dci = msg2->GetDci ();
//	          uint16_t rnti = dci.m_rnti;
	  }

	  break;
	case LteControlMessage::DL_HARQ:
	  {
	    //TODO DL_HARQ message handling
	  }
	  break;
	case LteControlMessage::MIB:
	    {
	      Ptr<MibLteControlMessage> mibMsg;
	      mibMsg = StaticCast<MibLteControlMessage> (msg);
	      LteRrcSap::MasterInformationBlock mib = mibMsg->GetMib ();

	      double ndlrb = (double)mib.dlBandwidth;
	      double nframe = (double)mib.systemFrameNumber;
	      mxArray* NDLRB = mxCreateDoubleScalar (ndlrb);
	      mxArray* NFrame = mxCreateDoubleScalar (nframe);
	      engPutVariable (m_ep, "NDLRB", NDLRB);
	      engPutVariable (m_ep, "NFrame", NFrame);
	      engEvalString (m_ep, "enb = ENB; \n");
	      engEvalString (m_ep, "enb.NDLRB = NDLRB; \n");
	      engEvalString (m_ep, "enb.NFrame = NFrame;\n");
	      engEvalString (m_ep, "fprintf('%d',enb.NDLRB)\n");
	      engEvalString (m_ep, "fprintf('%d',ENB.NDLRB)\n");
	      engEvalString (m_ep, "mib = lteMIB(enb);\n");
	      engEvalString (m_ep, "bch = lteBCH(ENB, mib);\n");
	      engEvalString (m_ep, "pbchSymbols = ltePBCH(enb, bch);\n");
	      engEvalString (m_ep, "pbchIndices = ltePBCHIndices(enb);\n");
	      engEvalString (m_ep, "subframe(pbchIndices) = pbchSymbols(1:length(pbchIndices));\n");
	    }

	  break;
	case LteControlMessage::RACH_PREAMBLE:
	  {
	    //TODO RACH_PREAMBLE message handling
	  }
	  break;
	case LteControlMessage::RAR:
	  {
	    //TODO RAR message handling
	  }

	  break;
	case LteControlMessage::SIB1:
	  {
	    Ptr<Sib1LteControlMessage> sibMsg = StaticCast<Sib1LteControlMessage>(msg);
	    LteRrcSap::SystemInformationBlockType1 sib1 = sibMsg->GetSib1();

	    Sib1Message s;
	    s.SetMessage(sib1);

	    std::cout << "SIB1 \n";
	    s.Print(std::cout);

	    Buffer buffer;
	    uint32_t bufferSize = s.GetSerializedSize();
	    buffer.AddAtStart (bufferSize);
	    s.Serialize(buffer.Begin());

	    mxArray *SIB1 = mxCreateNumericMatrix(bufferSize, 1, mxUINT8_CLASS, mxREAL);
	    buffer.CopyData((uint8_t*)mxGetPr(SIB1), bufferSize);
	    engPutVariable(m_ep, "SIB1", SIB1);

	    //TODO Finish SIB1 handling implementation, the above properly serializes ns-3 SIB1 structure
	    break;
	  }
	case LteControlMessage::UL_CQI:
	  {
	    //TODO UL_CQI message handling
	  }

	  break;
	case LteControlMessage::UL_DCI:
	  {
	    //TODO UL_DCI message handling
	  }

	  break;
	default:
	  NS_FATAL_ERROR("Unsupported control message type");
      }
    }

  engEvalString(m_ep, "txWaveform = lteOFDMModulate(ENB,subframe);\n");
  engEvalString(m_ep, "step(hSpecAnalTx, txWaveform);");

}


/*
 * At this point we properly configure the fading channel model to receive our subframe signal
 * then attempt to decode it and save the decoded data for later relaying back to ns-3
 */
void
MatlabChannel::PassThroughChannel()
{
  engEvalString(m_ep,"chcfg.SamplingRate = FS;\n"
		     "chcfg.InitTime = (ENB.NFrame * 10 + ENB.NSubframe) * 10e-3;\n"
		     "rxWaveform = lteFadingChannel(chcfg,txWaveform);\n");

  engEvalString(m_ep,""
		    "cec.PilotAverage = 'UserDefined';     % Type of pilot averaging\n"
		    "cec.FreqWindow = 9;                   % Frequency window size\n"
		    "cec.TimeWindow = 9;                   % Time window size\n"
		    "cec.InterpType = 'cubic';             % 2D interpolation type\n"
		    "cec.InterpWindow = 'Centered';        % Interpolation window type\n"
		    "cec.InterpWinSize = 1;                % Interpolation window size\n");

  engEvalString(m_ep, "griddims = lteResourceGridSize(ENB);\n"
		      "L = griddims(2);\n"
		      "rxgrid = lteOFDMDemodulate(ENB, rxWaveform);\n"
		      "[hest, nest] = lteDLChannelEstimate(ENB, cec, rxgrid(:,1:L,:));\n"
		      "pbchIndices = ltePBCHIndices(ENB);\n"
		      "[pbchRx, pbchHest] = lteExtractResources( ...\n"
		      "		pbchIndices, rxgrid(:,1:L,:), hest(:,1:L,:,:));\n"
		      ""
		      "[bchBits, pbchSymbols, nfmod4, mib, CellRefP] = ltePBCHDecode(ENB, pbchRx, pbchHest, nest);\n"
		      "clear enb;\n"
		      "enb = lteMIB(mib, ENB);\n"
		      "enb.NFrame = enb.NFrame+nfmod4;\n"
		      "FAIL = 0;");

  engEvalString(m_ep,"if (enb.CellRefP==0) FAIL=1;return;end\n"
		     "if (enb.NDLRB==0) FAIL=1;return; end\n");


//  mxArray* FAIL = engGetVariable(m_ep, "FAIL");
//  double fail = mxGetScalar(FAIL);
//  if(fail)
//    NS_FATAL_ERROR("Demodulcation failed miserably");

}


/*
 * This function deserlizes back data from MATLAB to ns-3 to close the communication loop
 */
Ptr<SpectrumSignalParameters>
MatlabChannel::DeserializeCtlrParams(Ptr<LteSpectrumSignalParametersDlCtrlFrame> txParams)
{
  engEvalString(m_ep, "NDLRB = enb.NDLRB;\n"
		      "NFrame = enb.NFrame;\n"
		      "NCellID = enb.NCellID;\n");

  mxArray* NLDRB = engGetVariable(m_ep, "NDLRB");
  mxArray* NFrame = engGetVariable(m_ep, "NFrame");
  mxArray* CELLID = engGetVariable(m_ep, "NCellID");

  Ptr<MibLteControlMessage> mibMsg;
  LteRrcSap::MasterInformationBlock mib;
  mib.dlBandwidth = mxGetScalar(NLDRB);
  mib.systemFrameNumber = mxGetScalar(NFrame);
  mibMsg = Create<MibLteControlMessage>();
  mibMsg->SetMib(mib);

  std::list<Ptr<LteControlMessage> > ctrlMsgList;
  ctrlMsgList.push_back(mibMsg);
  Ptr<LteSpectrumSignalParametersDlCtrlFrame> rxParams = DynamicCast<LteSpectrumSignalParametersDlCtrlFrame>(txParams->Copy());
  rxParams->ctrlMsgList = ctrlMsgList;
  rxParams->cellId = *mxGetPr(CELLID);

  return rxParams;

  //TODO For the moment only MIB part is handled here, for the future the rest of PDCCH and other to be implemented channels shall also be handled here
}


//TODO implement ULSrs deserialize after SRS UL is properly handled
Ptr<SpectrumSignalParameters>
MatlabChannel::DeserializeUlSrsParams(Ptr<LteSpectrumSignalParametersUlSrsFrame> txParams)
{
  Ptr<SpectrumSignalParameters> rxParams = txParams->Copy();

  return rxParams;

}


//TODO current simulations are restricted to control messages, complete network operation will need this and other functions
Ptr<SpectrumSignalParameters>
MatlabChannel::DeserializeDataParams(Ptr<LteSpectrumSignalParametersDataFrame> txParams)
{
  Ptr<SpectrumSignalParameters> rxParams = txParams->Copy();

  return rxParams;

}

TxSpectrumModelInfoMap_t::const_iterator
MatlabChannel::FindAndEventuallyAddTxSpectrumModel (Ptr<const SpectrumModel> txSpectrumModel)
{
  NS_LOG_FUNCTION (this << txSpectrumModel);
  SpectrumModelUid_t txSpectrumModelUid = txSpectrumModel->GetUid ();
  TxSpectrumModelInfoMap_t::iterator txInfoIterator = m_txSpectrumModelInfoMap.find (txSpectrumModelUid);

  if (txInfoIterator == m_txSpectrumModelInfoMap.end ())
    {
      // first time we see this TX SpectrumModel
      // we add it to the list
      std::pair<TxSpectrumModelInfoMap_t::iterator, bool> ret;
      ret = m_txSpectrumModelInfoMap.insert (std::make_pair (txSpectrumModelUid, TxSpectrumModelInfo (txSpectrumModel)));
      NS_ASSERT (ret.second);
      txInfoIterator = ret.first;

      // and we create the converters for all the RX SpectrumModels that we know of
      for (RxSpectrumModelInfoMap_t::const_iterator rxInfoIterator = m_rxSpectrumModelInfoMap.begin ();
           rxInfoIterator != m_rxSpectrumModelInfoMap.end ();
           ++rxInfoIterator)
        {
          Ptr<const SpectrumModel> rxSpectrumModel = rxInfoIterator->second.m_rxSpectrumModel;
          SpectrumModelUid_t rxSpectrumModelUid = rxSpectrumModel->GetUid ();

          if (rxSpectrumModelUid != txSpectrumModelUid)
            {
              NS_LOG_LOGIC ("Creating converters between SpectrumModelUids " << txSpectrumModelUid << " and " << rxSpectrumModelUid );

              SpectrumConverter converter (txSpectrumModel, rxSpectrumModel);
              std::pair<SpectrumConverterMap_t::iterator, bool> ret2;
              ret2 = txInfoIterator->second.m_spectrumConverterMap.insert (std::make_pair (rxSpectrumModelUid, converter));
              NS_ASSERT (ret2.second);
            }
        }
    }
  else
    {
      NS_LOG_LOGIC ("SpectrumModelUid " << txSpectrumModelUid << " already present");
    }
  return txInfoIterator;
}


void
MatlabChannel::StartRx (Ptr<SpectrumSignalParameters> params, Ptr<SpectrumPhy> receiver)
{
  NS_LOG_FUNCTION (this);
  receiver->StartRx (params);
}


void
MatlabChannel::AddRx (Ptr<SpectrumPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);

  Ptr<const SpectrumModel> rxSpectrumModel = phy->GetRxSpectrumModel ();

  NS_ASSERT_MSG ((0 != rxSpectrumModel), "phy->GetRxSpectrumModel () returned 0. Please check that the RxSpectrumModel is already set for the phy before calling MultiModelSpectrumChannel::AddRx (phy)");

  SpectrumModelUid_t rxSpectrumModelUid = rxSpectrumModel->GetUid ();

  std::vector<Ptr<SpectrumPhy> >::const_iterator it;

  // remove a previous entry of this phy if it exists
  // we need to scan for all rxSpectrumModel values since we don't
  // know which spectrum model the phy had when it was previously added
  // (it's probably different than the current one)
  for (RxSpectrumModelInfoMap_t::iterator rxInfoIterator = m_rxSpectrumModelInfoMap.begin ();
       rxInfoIterator !=  m_rxSpectrumModelInfoMap.end ();
       ++rxInfoIterator)
    {
      std::set<Ptr<SpectrumPhy> >::iterator phyIt = rxInfoIterator->second.m_rxPhySet.find (phy);
      if (phyIt !=  rxInfoIterator->second.m_rxPhySet.end ())
        {
          rxInfoIterator->second.m_rxPhySet.erase (phyIt);
          --m_numDevices;
          break; // there should be at most one entry
        }
    }

  ++m_numDevices;

  RxSpectrumModelInfoMap_t::iterator rxInfoIterator = m_rxSpectrumModelInfoMap.find (rxSpectrumModelUid);

  if (rxInfoIterator == m_rxSpectrumModelInfoMap.end ())
    {
      // spectrum model unknown, add it to the list of RxSpectrumModels
      std::pair<RxSpectrumModelInfoMap_t::iterator, bool> ret;
      ret = m_rxSpectrumModelInfoMap.insert (std::make_pair (rxSpectrumModelUid, RxSpectrumModelInfo (rxSpectrumModel)));
      NS_ASSERT (ret.second);
      // also add the phy to the newly created set of SpectrumPhy for this RxSpectrumModel
      std::pair<std::set<Ptr<SpectrumPhy> >::iterator, bool> ret2 = ret.first->second.m_rxPhySet.insert (phy);
      NS_ASSERT (ret2.second);

      // and create the necessary converters for all the TX spectrum models that we know of
      for (TxSpectrumModelInfoMap_t::iterator txInfoIterator = m_txSpectrumModelInfoMap.begin ();
           txInfoIterator != m_txSpectrumModelInfoMap.end ();
           ++txInfoIterator)
        {
          Ptr<const SpectrumModel> txSpectrumModel = txInfoIterator->second.m_txSpectrumModel;
          NS_LOG_LOGIC ("Creating converters between SpectrumModelUids " << txSpectrumModel->GetUid () << " and " << rxSpectrumModelUid );
          SpectrumConverter converter (txSpectrumModel, rxSpectrumModel);
          std::pair<SpectrumConverterMap_t::iterator, bool> ret2;
          ret2 = txInfoIterator->second.m_spectrumConverterMap.insert (std::make_pair (rxSpectrumModelUid, converter));
          NS_ASSERT (ret2.second);
        }
    }
  else
    {
      // spectrum model is already known, just add the device to the corresponding list
      std::pair<std::set<Ptr<SpectrumPhy> >::iterator, bool> ret2 = rxInfoIterator->second.m_rxPhySet.insert (phy);
      NS_ASSERT (ret2.second);
    }

}

uint32_t
MatlabChannel::GetNDevices (void) const
{
  return 0;
}

Ptr<NetDevice>
MatlabChannel::GetDevice (uint32_t i) const
{
  return NULL;
}

void
MatlabChannel::AddPropagationLossModel(Ptr<PropagationLossModel> loss)
{

}

void
MatlabChannel::SetPropagationDelayModel(Ptr<PropagationDelayModel> delay)
{

}

void
MatlabChannel::AddSpectrumPropagationLossModel(Ptr<SpectrumPropagationLossModel> loss)
{

}
}
