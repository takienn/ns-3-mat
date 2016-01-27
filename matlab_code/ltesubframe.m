clear all

% general settings 
enb = {};
enb.CellRefP = 1;
enb.CyclicPrefix = 'Normal';
enb.Ng = 'Sixth';


% settings from ns-3
enb.NDLRB = 25;
enb.NCellID = 0;
enb.NSubframe = 0;
enb.CFI = 3; % currently fixed to 3 in Ns-3 
NFrame = 0;

% Pilots (RS)
rs = lteCellRS(enb);
ind1 = lteCellRSIndices(enb);


% PSS & SSS
pss = ltePSS(enb);
sss = lteSSS(enb);
ind2 = ltePSSIndices(enb);
ind3 = lteSSSIndices(enb);

% PBCH
mib = lteMIB(enb);
bchCoded = lteBCH(enb,mib);
pbchSymbols = ltePBCH(enb,bchCoded);
ind4 = ltePBCHIndices(enb);


% PDCCH
pdcchInfo = ltePDCCHInfo(enb);
cw = randi([0,1],pdcchInfo.MTot,1); % creates dummy data for pdcch
% alternatively we can do:
% ue = {};
% ue.PDCCHFormat = 1;  % 0, 1, 2, 3
% ue.RNTI = 0; % take from ns-3
% dciInfo = lteDCIInfo(enb);
% dcibits = zeros(dciInfo.Format1,1);
% cw = lteDCIEncode(ue,dcibits);
[pdcchSym,info] = ltePDCCH(enb,cw);
ind5 = ltePDCCHIndices(enb);


% PDSCH
tm = lteTestModel('1.1','5MHz');
tm.PDSCH.RNTI = 0;
tm.PDSCH.RV = 0;

prbset = (0:tm.NDLRB-1)';
[ind6,info] = ltePDSCHIndices(tm,tm.PDSCH,prbset);
trBlk  = randi([0,1],info.Gd,1);
cw = lteDLSCH(tm,tm.PDSCH,info.G,trBlk);
pdschSym = ltePDSCH(tm,tm.PDSCH,cw);





sf = lteResourceGrid(enb);
sf = sf(:);
sf(ind1) = rs;
sf(ind2) = pss;
sf(ind3) = sss;

sf(ind4) = pbchSymbols(1:240);
sf(ind5) = pdcchSym;
sf(ind6) = pdschSym;
