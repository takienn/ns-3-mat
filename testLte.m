enb.NDLRB = 6;
enb.NCellID = 0;
enb.CellRefP = 2;
enb.CyclicPrefix = 'Normal';
enb.Ng = 'Sixth';
enb.PHICHDuration = 'Normal';
enb.NFrame = 0;
enb.NSubframe = 0;
enb.DuplexMode = 'FDD';
mib = lteMIB(enb);
bch = lteBCH(enb,mib);
pbch = ltePBCH(enb,bch);
pbchIndices = ltePBCHIndices(enb);
pss = ltePSS(enb);
sss = lteSSS(enb);
pssIndices = ltePSSIndices(enb);
sssIndices = lteSSSIndices(enb);

subframe = lteResourceGrid(enb);
subframe(pssIndices) = pss;
subframe(sssIndices) = sss;
subframe(pbchIndices) = pbch(pbchIndices);
txWave = lteOFDMModulate(enb,subframe);

chcfg.DelayProfile = 'EPA';
chcfg.NRxAnts = 2;
chcfg.DopplerFreq = 5;
chcfg.MIMOCorrelation = 'Low';
chcfg.SamplingRate = 1.92e6; %info.SamplingRate;
chcfg.Seed = 1;
chcfg.InitPhase = 'Random';
chcfg.ModelType = 'GMEDS';
chcfg.NTerms = 16;
chcfg.NormalizeTxAnts = 'On';
chcfg.NormalizePathGains = 'On';
chcfg.InitTime = 0;

rxWave = lteFadingChannel(chcfg,txWave);
rxSubframe = lteOFDMDemodulate(enb,rxWave);
cec.FreqWindow = 1;
cec.TimeWindow = 1;
cec.InterpType = 'cubic';
cec.PilotAverage = 'UserDefined';
cec.InterpWinSize = 3;
cec.InterpWindow = 'Causal';
[hest,noisest] = lteDLChannelEstimate(enb, rxSubframe);
rxPbch = rxSubframe(pbchIndices);
[bits,symbols,nfmod4,rxMib,cellrefp] = ltePBCHDecode(enb, rxPbch, hest, noisest);