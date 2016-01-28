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
#include <list>
#include "matlab-channel.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MatlabChannel");

NS_OBJECT_ENSURE_REGISTERED (MatlabChannel);

TypeId
MatlabChannel::GetTypeId()
{
  static TypeId tid = TypeId("ns3::MatlabChannel")
    .SetParent<SpectrumChannel> ()
    .SetGroupName("lte");

  return tid;
}

MatlabChannel::~MatlabChannel ()
{
}

MatlabChannel::MatlabChannel()
{
  m_ep = engOpen("");
  NS_ASSERT(m_ep);
  engEvalString(m_ep, "ENB.NDLRB = 25;"
      "ENB.CellRefP = 1"
      "ENB.NCellID = 0"
      "ENB.CyclicPrefix = 'Normal';"
      "ENB.PHICHDuration = 'Normal';"
      "ENB.DuplexMode = 'FDD';"
      "ENB.CFI = 3;"
      "ENB.Ng = 'Sixth';");

}
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
    if (lteDlCtrlRxParams != 0)
      {

	Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(txParams->txPhy->GetDevice());
	if(dev)
	  {
	    uint8_t dlBw = dev->GetDlBandwidth();

	    double dl_fc = LteSpectrumValueHelper::GetCarrierFrequency(dev->GetDlEarfcn());
	    double ul_fc = LteSpectrumValueHelper::GetCarrierFrequency(dev->GetUlEarfcn());
	    engPutVariable(m_ep,"NFrame", mxCreateDoubleScalar(lteDlCtrlRxParams->frameN));
	    engPutVariable(m_ep,"NSubframe", mxCreateDoubleScalar(lteDlCtrlRxParams->subframeN));
	    engPutVariable(m_ep,"DL_FC", mxCreateDoubleScalar(dl_fc));
	    engPutVariable(m_ep,"UL_FC", mxCreateDoubleScalar(ul_fc));
	    engPutVariable(m_ep,"dlBw", mxCreateDoubleScalar((double)dlBw));
	    engEvalString(m_ep, "ENB.NDLRB = dlBw"
				"ENB.NFrame = NFrame;"
				"ENB.NSubframe = NSubframe;"
				"subframe = lteDLResourceGrid(ENB)");
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

	//Generate Data Frame in MATLAB
	SerializeDataParams (lteDataRxParams);
      }
    else if (lteDlCtrlRxParams!=0)
      {
	//Generate Ctrl Frame in MATLAB
	SerializeCtrlParams (lteDlCtrlRxParams);
      }
    else if (lteUlSrsRxParams!=0)
      {
	//Generate UL SRS in Matlab
	SerializeSrsParams (lteUlSrsRxParams);
      }
    else
      {
	//NOT LTE
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

void
MatlabChannel::SerializeCtrlParams (Ptr<LteSpectrumSignalParametersDlCtrlFrame> params)
{
  std::list<Ptr<LteControlMessage> > ctrlMsgList = params->ctrlMsgList;
  uint16_t cellId = params->cellId;
  bool pss = params->pss;

  NS_ASSERT(m_ep);

  NS_ASSERT(m_ep);

  engPutVariable(m_ep, "CELLID", mxCreateDoubleScalar((double) cellId));
  engEvalString(m_ep,"ENB.NCellID = CELLID;");

  if(pss)
    {
      engEvalString(m_ep, "pss = ltePSS(ENB);"
	  "pssIndices = ltePSSIndices(ENB);"
	  "subframe[pssIndices] = pss;"
	  "sss = lteSSS(ENB);"
	  "sssIndices = lteSSSIndices(ENB);"
	  "subframe[sssIndices] = sss;");
    }

  SerializeCtlrMessages(ctrlMsgList);
}

void
MatlabChannel::SerializeSrsParams (Ptr<LteSpectrumSignalParametersUlSrsFrame> params)
{

}

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

	  break;
	case LteControlMessage::DL_CQI:

	  break;
	case LteControlMessage::DL_DCI:

	  break;
	case LteControlMessage::DL_HARQ:

	  break;
	case LteControlMessage::MIB:
	    {
	      Ptr<MibLteControlMessage> mibMsg;
	      mibMsg = StaticCast<MibLteControlMessage> (msg);
	      LteRrcSap::MasterInformationBlock mib = mibMsg->GetMib ();

	      mxArray* NDLRB = mxCreateDoubleScalar (mib.dlBandwidth);
	      mxArray* NFrame = mxCreateDoubleScalar (mib.systemFrameNumber);
	      engPutVariable (m_ep, "NLRB", NDLRB);
	      engPutVariable (m_ep, "NFrame", NFrame);
	      engEvalString (m_ep, "enb = ENB; "
				   "enb.NDLRB = NDLRB; "
				   "enb.NFrame = NFrame;"
				   "mib = lteMib(enb);"
				   "bch = lteBCH(ENB, mib);"
				   "pbchSymbols = ltePBCH(ENB, bch);"
				   "pbchIndices = ltePBCHIndices(ENB)"
				   "subframe(pbchIndices) = pbchSymbols");
	    }

	  break;
	case LteControlMessage::RACH_PREAMBLE:

	  break;
	case LteControlMessage::RAR:

	  break;
	case LteControlMessage::SIB1:
//	  Sib1LteControlMessage sibMsg = StaticCast<Sib1LteControlMessage>(msg);
//	  LteRrcSap::SystemInformationBlockType1 sib1 = sibMsg.GetSib1();
//	  sib1.cellAccessRelatedInfo.plmnIdentityInfo;

	  break;
	case LteControlMessage::UL_CQI:

	  break;
	case LteControlMessage::UL_DCI:

	  break;
	default:
	  NS_FATAL_ERROR("Unsupported control message type");
      }
    }

  engEvalString(m_ep, "txWaveform = lteOFDMModulate(ENB,subframe);");

  engEvalString(m_ep,"chcfg.DelayProfile = 'EPA';"
		     "chcfg.NRxAnts = 1;"
		     "chcfg.DopplerFreq = 5;"
		     "chcfg.MIMOCorrelation = 'Low';"
		     "chcfg.SamplingRate = FS;"
		     "chcfg.Seed = 1;"
		     "chcfg.InitPhase = 'Random';"
		     "chcfg.ModelType = 'GMEDS';"
		     "chcfg.NTerms = 16;"
		     "chcfg.NormalizeTxAnts = 'On';"
		     "chcfg.NormalizePathGains = 'On';"
		     "chcfg.InitTime = (ENB.NFrame * 10 + ENB.NSubframe) * 10e-3;"
		     "rxWaveform = lteFadingChannel(chcfg,txWaveform);");

  engEvalString(m_ep,""
		    "cec.PilotAverage = 'UserDefined';     % Type of pilot averaging"
		    "cec.FreqWindow = 9;                   % Frequency window size"
		    "cec.TimeWindow = 9;                   % Time window size"
		    "cec.InterpType = 'cubic';             % 2D interpolation type"
		    "cec.InterpWindow = 'Centered';        % Interpolation window type"
		    "cec.InterpWinSize = 1;                % Interpolation window size");
//  engEvalString(m_ep,"if (~isfield(ENB,'DuplexMode'))"
//		      "    duplexModes = {'TDD' 'FDD'};"
//		      "else"
//		      "    duplexModes = {ENB.DuplexMode};"
//		      "end"
//		      "if (~isfield(ENB,'CyclicPrefix'))"
//		      "    cyclicPrefixes = {'Normal' 'Extended'};"
//		      "else"
//		      "    cyclicPrefixes = {ENB.CyclicPrefix};"
//		      "end"
//		      "searchalg.MaxCellCount = 1;"
//		      "searchalg.SSSDetection = 'PostFFT';"
//		      "peakMax = -Inf;"
//		      "enb = ENB;"
//		      "for duplexMode = duplexModes"
//		      "	for cyclicPrefix = cyclicPrefixes"
//		      "		enb.DuplexMode = duplexMode{1};"
//		      "		enb.CyclicPrefix = cyclicPrefix{1};"
//		      "		[enb.NCellID, offset, peak] = lteCellSearch(enb, rxWaveform, searchalg);"
//		      "		if (peak>peakMax)"
//		      "			enbMax = enb;"
//		      "			offsetMax = offset;"
//		      "			peakMax = peak;"
//		      "		end"
//		      "	end"
//		      "end"
//		      "enb = enbMax;"
//		      "offset = offsetMax;"
//		      "corr = cell(1,3);"
//		      "for i = 0:2"
//		      "    enb.NCellID = mod(enbMax.NCellID + i,504);"
//		      "    [~,corr{i+1}] = lteDLFrameOffset(enb, downsampled);"
//		      "    corr{i+1} = sum(corr{i+1},2);"
//		      "end"
//		      "threshold = 1.3 * max([corr{2}; corr{3}]); % multiplier of 1.3 empirically obtained"
//		      "if (max(corr{1})<threshold)"
//		      "    warning('Synchronization signal correlation was weak; detected cell identity may be incorrect.');"
//		      "end"
//		      "% return to originally detected cell identity"
//		      "enb.NCellID = enbMax.NCellID;"
//		      ""
  engEvalString(m_ep, "griddims = lteResourceGridSize(ENB);"
		      "L = griddims(2);"
		      "enb = ENB;"
		      "rxgrid = lteOFDMDemodulate(ENB, rxWaveform);"
		      "[hest, nest] = lteDLChannelEstimate(ENB, cec, rxgrid(:,1:L,:));"
		      "pbchIndices = ltePBCHIndices(enb);"
		      "[pbchRx, pbchHest] = lteExtractResources( ..."
		      "		pbchIndices, rxgrid(:,1:L,:), hest(:,1:L,:,:));"
		      ""
		      "[bchBits, pbchSymbols, nfmod4, mib, enb.CellRefP] = ltePBCHDecode( ..."
		      "		ENB, pbchRx, pbchHest, nest);"
		      "enb = lteMIB(mib, enb);"
		      "enb.NFrame = enb.NFrame+nfmod4;"
		      "if (enb.CellRefP==0)"
		      "	fprintf('MIB decoding failed (enb.CellRefP=0).\n\n');"
		      "	return;"
		      "end"
		      "if (enb.NDLRB==0)"
		      "	fprintf('MIB decoding failed (enb.NDLRB=0).\n\n');"
		      "	return;"
		      "end");
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
  mxArray* NLDRB = engGetVariable(m_ep, "enb.NLDRB");
  mxArray* NFrame = engGetVariable(m_ep, "enb.NFrame");

  Ptr<MibLteControlMessage> mibMsg;
  LteRrcSap::MasterInformationBlock mib;
  mib.dlBandwidth = *mxGetPr(NLDRB);
  mib.systemFrameNumber = *mxGetPr(NFrame);
  mibMsg = Create<MibLteControlMessage>();
  mibMsg->SetMib(mib);

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
