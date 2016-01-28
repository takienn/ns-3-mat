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
%tm = lteTestModel('1.1','5MHz');
%tm.PDSCH.RNTI = 0;
%tm.PDSCH.RV = 0;
enb.PDSCH.RNTI = 0;
enb.PDSCH.RV = 0;
enb.PDSCH.Modulation = 'QPSK';
enb.PDSCH.TxScheme = 'Port0';
prbset = (0:enb.NDLRB-1)';
[ind6,info] = ltePDSCHIndices(enb,enb.PDSCH,prbset);
trBlk  = randi([0,1],info.Gd,1);
cw = lteDLSCH(enb,enb.PDSCH,info.G,trBlk);
pdschSym = ltePDSCH(enb,enb.PDSCH,cw);





sf = lteResourceGrid(enb);
%sf = sf(:);
sf(ind1) = rs;
sf(ind2) = pss;
sf(ind3) = sss;

sf(ind4) = pbchSymbols(1:240);
sf(ind5) = pdcchSym;
sf(ind6) = pdschSym;
waveform = lteOFDMModulate(enb,sf);

% channel model 
% c = 3e8; 
% % Excess taps delay (according to 3GPP TS 36.104 Annex B.2)
% delays_pedestrianEPA = [0 30e-9 70e-9 90e-9 120e-9 190e-9 410e-9];
% delays_vehicularEVA = [0 30e-9 150e-9 310e-9 370e-9 710e-9 1090e-9 1730e-9 2510e-9];
% delays_urbanETU = [0 50e-9 120e-9 200e-9 230e-9 500e-9 1600e-9 2300e-9 5000e-9];
% Td = 100/c;
% 
% % Realtive power of taps (according to 3GPP TS 36.104 Annex B.2)
% power_pedestrianEPA = [0.0 -1.0 -2.0 -3.0 -8.0 -17.2 -20.8];
% power_vehicularEVA = [0.0 -1.5 -1.4 -3.6 -0.6 -9.1 -7.0 -12.0 -16.9];
% power_urbanETU = [-1.0 -1.0 -1.0 0.0 0.0 0.0 -3.0 -5.0 -7.0];
% 
% % DL EARFCN=500
% % fc = 2160e6; %
% 
% % UL  EARFCN=18100
% fc = 1930e6;
% 
% lambda = c/fc;
% v_km_h = 3.0;  % speed of mobile node
% v_m_s = v_km_h / 3.6;
% fd = v_m_s / lambda; % doppler shift
% 
% % when working with an FFT, the normalized frequency w is
% % w = 2 * pi * (f/fs) * t
% % hence the max normalized frequency w=2*pi corresponds to f = fs,
% % hence fs is also the max frequecy of our PowerSpectralDensity
% fs = 5e6; 
% 
% % sampling period must be determined corresponding to the sampling
% % frequency, because of the properties of an FFT
% % in other words, if ts = 1/fs, then the samples of the FFT will be
% % spaced by fs (which is what we want)
% ts = 1/fs; % sampling period (i.e., 1 subframe duration)
% 
% 
% % create the channel object
% %c = rayleighchan(ts, fd, delays_pedestrianEPA, power_pedestrianEPA);
% %c = rayleighchan(ts, fd, delays_vehicularEVA, power_vehicularEVA);
% c = rayleighchan(ts, fd, delays_urbanETU, power_urbanETU);
% %c.StorePathGains = 1;
% c.ResetBeforeFiltering = 0;
% c.NormalizePathGains = 1;
% y = filter(c,sf);   

chcfg.DelayProfile = 'EPA';
chcfg.NRxAnts = 1;
chcfg.DopplerFreq = 5;
chcfg.MIMOCorrelation = 'Low';
chcfg.SamplingRate = 5e6; %info.SamplingRate;
chcfg.Seed = 1;
chcfg.InitPhase = 'Random';
chcfg.ModelType = 'GMEDS';
chcfg.NTerms = 16;
chcfg.NormalizeTxAnts = 'On';
chcfg.NormalizePathGains = 'On';
chcfg.InitTime = 0;

% y = lteFadingChannel(chcfg,waveform);
y = waveform + 0.001*rand(size(waveform));



% decoder
cec.FreqWindow = 1;
cec.TimeWindow = 1;
cec.InterpType = 'cubic';
cec.PilotAverage = 'UserDefined';
cec.InterpWinSize = 3;
cec.InterpWindow = 'Causal';

rxgrid = lteOFDMDemodulate(enb, y);
[hest,noisest] = lteDLChannelEstimate(enb, cec, rxgrid);
rxpbchSym = rxgrid(ind4);
[bits,symbols,nfmod4,trblk,cellrefp] = ltePBCHDecode(enb, rxpbchSym, hest, noisest);

mib_decoded = trblk;

