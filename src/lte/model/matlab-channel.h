/*
 * matlab-channel.h
 *
 *  Created on: Jan 27, 2016
 *      Author: kentux
 */

#ifndef SRC_LTE_MODEL_MATLAB_CHANNEL_H_
#define SRC_LTE_MODEL_MATLAB_CHANNEL_H_

#include <ns3/spectrum-channel.h>
#include <ns3/spectrum-converter.h>
#include <ns3/spectrum-model.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/type-id.h>

#include <list>
#include "engine.h"

namespace ns3
{

class SpectrumPhy;
class MobilityModel;
class NetDevice;

/**
 * \ingroup spectrum
 * Container: SpectrumModelUid_t, SpectrumConverter
 */
typedef std::map<SpectrumModelUid_t, SpectrumConverter> SpectrumConverterMap_t;

/**
 * \ingroup spectrum
 * The Tx spectrum model information. This class is used to convert
 * one spectrum model into another one.
 */
class TxSpectrumModelInfo
{
public:
  /**
   * Constructor.
   * \param txSpectrumModel the Tx Spectrum model.
   */
  TxSpectrumModelInfo (Ptr<const SpectrumModel> txSpectrumModel);

  Ptr<const SpectrumModel> m_txSpectrumModel;     //!< Tx Spectrum model.
  SpectrumConverterMap_t m_spectrumConverterMap;  //!< Spectrum converter.
};

/**
 * \ingroup spectrum
 * The Rx spectrum model information. This class is used to convert
 * one spectrum model into another one.
 */
class RxSpectrumModelInfo
{
public:
  /**
   * Constructor.
   * \param rxSpectrumModel the Rx Spectrum model.
   */
  RxSpectrumModelInfo (Ptr<const SpectrumModel> rxSpectrumModel);

  Ptr<const SpectrumModel> m_rxSpectrumModel;  //!< Rx Spectrum model.
  std::set<Ptr<SpectrumPhy> > m_rxPhySet;      //!< Container of the Rx Spectrum phy objects.
};


/**
 * \ingroup spectrum
 * Container: SpectrumModelUid_t, TxSpectrumModelInfo
 */
typedef std::map<SpectrumModelUid_t, TxSpectrumModelInfo> TxSpectrumModelInfoMap_t;
typedef std::map<SpectrumModelUid_t, RxSpectrumModelInfo> RxSpectrumModelInfoMap_t;

class MatlabChannel : public SpectrumChannel
{
public:
  virtual ~MatlabChannel ();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  MatlabChannel ();


  // inherited from SpectrumChannel
  virtual void
  AddPropagationLossModel (Ptr<PropagationLossModel> loss);
  virtual void
  AddSpectrumPropagationLossModel (Ptr<SpectrumPropagationLossModel> loss);
  virtual void
  SetPropagationDelayModel (Ptr<PropagationDelayModel> delay);
  virtual void
  AddRx (Ptr<SpectrumPhy> phy);
  virtual void
  StartTx (Ptr<SpectrumSignalParameters> params);

  // inherited from Channel
  virtual uint32_t GetNDevices (void) const;
  virtual Ptr<NetDevice> GetDevice (uint32_t i) const;

private:
  /**
   * This method checks if m_rxSpectrumModelInfoMap contains an entry
   * for the given TX SpectrumModel. If such entry exists, it returns
   * an interator pointing to it. If not, it creates a new entry in
   * m_txSpectrumMpodelInfoMap, and returns an iterator to it.
   *
   * @param txSpectrumModel The TX SpectrumModel  being considered
   *
   * @return An iterator pointing to the corresponding entry in m_txSpectrumModelInfoMap
   */
  TxSpectrumModelInfoMap_t::const_iterator FindAndEventuallyAddTxSpectrumModel (Ptr<const SpectrumModel> txSpectrumModel);

  /**
   * Used internally to reschedule transmission after the propagation delay.
   *
   * @param params The signal paramters.
   * @param receiver A pointer to the receiver SpectrumPhy.
   */
  virtual void StartRx (Ptr<SpectrumSignalParameters> params, Ptr<SpectrumPhy> receiver);

  void SerializeDataParams (Ptr<LteSpectrumSignalParametersDataFrame> params);
  void SerializeCtrlParams(Ptr<LteSpectrumSignalParametersDlCtrlFrame> params);
  void SerializeSrsParams(Ptr<LteSpectrumSignalParametersUlSrsFrame> params);
  void SerializeCtlrMessages(std::list<Ptr<LteControlMessage> > ctrlMsgList);

  void PassThroughChannel();
  Ptr<SpectrumSignalParameters> DeserializeCtlrParams(Ptr<LteSpectrumSignalParametersDlCtrlFrame> txParams);
  Ptr<SpectrumSignalParameters> DeserializeUlSrsParams(Ptr<LteSpectrumSignalParametersUlSrsFrame> txParams);
  Ptr<SpectrumSignalParameters> DeserializeDataParams(Ptr<LteSpectrumSignalParametersDataFrame> txParams);
  /**
    * Data structure holding, for each TX SpectrumModel,  all the
    * converters to any RX SpectrumModel, and all the corresponding
    * SpectrumPhy instances.
    *
    */
   TxSpectrumModelInfoMap_t m_txSpectrumModelInfoMap;


   /**
    * Data structure holding, for each RX spectrum model, all the
    * corresponding SpectrumPhy instances.
    */
   RxSpectrumModelInfoMap_t m_rxSpectrumModelInfoMap;

   /**
    * Number of devices connected to the channel.
    */
   uint32_t m_numDevices;

   static Engine* m_ep;

};

}
#endif /* SRC_LTE_MODEL_MATLAB_CHANNEL_H_ */
