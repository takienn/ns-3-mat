#include <ns3/lte-rrc-header.h>
#include "engine.h"

using namespace ns3;
int main(int argc, char **argv)
{

  Engine* ep = engOpen("");


  LteRrcSap::SystemInformationBlockType1 sib1;

  sib1.cellAccessRelatedInfo.cellIdentity =1;
  sib1.cellAccessRelatedInfo.csgIdentity=1;
  sib1.cellAccessRelatedInfo.csgIndication = false;
  sib1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity = 10;
  sib1.cellSelectionInfo.qQualMin = -34;
  sib1.cellSelectionInfo.qRxLevMin = -79;

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
  engPutVariable(ep, "SIB1", SIB1);

  engEvalString(ep,
		"rmc = lteRMCDL('R.12');"
		"rmc.CellRefP = 2;"
		"rmc.PDSCH.NLayers = 2;"
		"rmc.NCellID = 64;"
		"rmc.NFrame = 100;"
		"rmc.TotSubframes = 8*10;"
		"rmc.OCNGPDSCHEnable = 'On';"
		"rmc.PDSCH.RNTI = 61;"
		"rmc.SIB.Enable = 'On';"
		"rmc.SIB.DCIFormat = 'Format1A';"
		"rmc.SIB.AllocationType = 0;"
		"rmc.SIB.VRBStart = 0;"
		"rmc.SIB.VRBLength = 6;"
		"rmc.SIB.Gap = 0;"
		"rmc.SIB.Data = SIB1;"
		"trData = [1;0;0;1];"
		"[eNodeBOutput,txGrid,rmc] = lteRMCDLTool(rmc,trData);");

  return 0;
}
