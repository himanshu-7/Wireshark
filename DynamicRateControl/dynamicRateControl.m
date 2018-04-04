%% Waveform Configuration
cfgVHT = wlanVHTConfig;         
cfgVHT.ChannelBandwidth = 'CBW80'; % 40 MHz channel bandwidth
cfgVHT.MCS = 1;                    % QPSK rate-1/2
cfgVHT.APEPLength = 4096;          % APEP length in bytes

% Set random stream for repeatability of results
s = rng(21);

%% Channel Configuration
% In this example a TGac N-LOS channel model is used with delay profile
% Model-D. For Model-D when the distance between the transmitter and
% receiver is greater than or equal to 10 meters, the model is NLOS. This
% is described further in <matlab:doc('wlanTGacChannel') wlanTGacChannel>.

tgacChannel = wlanTGacChannel;
tgacChannel.DelayProfile = 'Model-A';
tgacChannel.ChannelBandwidth = cfgVHT.ChannelBandwidth;
tgacChannel.NumTransmitAntennas = 1;
tgacChannel.NumReceiveAntennas = 1;
tgacChannel.TransmitReceiveDistance = 20; % Distance in meters for NLOS
tgacChannel.RandomStream = 'mt19937ar with seed';
tgacChannel.Seed = 10;

% Set the sampling rate for the channel
sr = wlanSampleRate(cfgVHT);
tgacChannel.SampleRate = sr;

%% Rate Control Algorithm Parameters

rcaAttack = 1;  % Control the sensitivity when MCS is increasing
rcaRelease = 0; % Control the sensitivity when MCS is decreasing
threshold = [11 14 19 20 25 28 30 31 35]; 
snrUp = [threshold inf]+rcaAttack;
snrDown = [-inf threshold]-rcaRelease;
snrInd = cfgVHT.MCS; % Store the start MCS value

%% Simulation Parameters

numPackets = 100; % Number of packets transmitted during the simulation 
walkSNR = true; 

% Select SNR for the simulation

if walkSNR
    meanSNR = 22;   % Mean SNR
    amplitude = 14; % Variation in SNR around the average mean SNR value
    % Generate varying SNR values for each transmitted packet
    baseSNR = sin(linspace(1,10,numPackets))*amplitude+meanSNR;
    snrWalk = baseSNR(1); % Set the initial SNR value
    % The maxJump controls the maximum SNR difference between one
    % packet and the next 
    maxJump = 0.5;
else
    % Fixed mean SNR value for each transmitted packet. All the variability
    % in SNR comes from a time varying radio channel
    snrWalk = 22; %#ok<UNRCH>
    end

% To plot the equalized constellation for each spatial stream set
% displayConstellation to true
displayConstellation = false;
if displayConstellation
    ConstellationDiagram = comm.ConstellationDiagram; %#ok<UNRCH>
    ConstellationDiagram.ShowGrid = true;
    ConstellationDiagram.Name = 'Equalized data symbols';
end
 
% Define simulation variables
snrMeasured = zeros(1,numPackets);
MCS = zeros(1,numPackets);
ber = zeros(1,numPackets);
packetLength = zeros(1,numPackets);
constant_snr=1;
not_constant_snr=0;
BER_array= zeros(1,3);
i=1; same=0;j=1;
%snr_eg=[5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35,5,35];
%snr_eg=[15, 25, 9, 20, 35, 37, 31, 45, 40, 80,26, 23, 18, 20, 50, 22, 33, 17, 14, 25,28, 30, 34, 35, 17,17, 15, 21, 28, 29,20, 30, 76, 35, 14, 12, 11, 16, 22, 30,50, 10, 60, 70, 54, 50, 48, 30, 35, 39,25, 30, 76, 30, 14, 29, 18, 13, 22, 30,50, 55, 60, 70, 54, 50, 48, 30, 35, 39,26, 23, 18, 12, 21, 22, 35, 16,14, 25,30, 23, 18,35, 17, 17, 15, 21, 28, 29,20, 30, 76, 35, 14, 12, 11, 16, 22, 30,30, 23, 18, 17,17, 22, 11,35, 40, 39, 28, 30, 34, 35, 17, 17, 15,21, 28, 29,20, 30, 76, 35, 14, 12, 11, 16, 22,30,50,20, 25, 33, 54, 50, 30, 30, 35, 39,20, 30, 76, 35, 14, 12, 11, 16, 22, 30,23, 18,35, 17, 17, 29,20, 30,13, 12, 20, 28, 36, 10, 40, 45, 49, 50, 21, 28,17,17, 15, 21, 28, 29,20, 30, 76, 35, 14, 12,11, 16, 22, 30,50, 10, 60, 70, 54, 50, 48, 30, 35, 39,25,30, 76, 30, 14, 29, 18, 13, 22, 30,50, 55,60, 70, 54, 50, 48, 30, 35, 39,26, 23, 18, 12, 21, 22, 35,23, 18,35, 17, 17, 29,20, 30,13, 12, 20,28, 36, 10, 40, 45, 49, 50, 21, 28,76, 35, 14, 12, 11, 16, 22, 30,50, 10, 60, 70, 54, 50, 48, 17, 15,21, 28,29,20, 30, 76, 35, 14, 12, 11, 16, 22, 50, 22, 33, 17, 14, 25,28, 30, 17, 17, 15, 21, 28, 29,20,30, 76, 35, 14, 12, 11, 16, 22, 30,30, 23, 18, 17, 17, 22, 11,35, 40, 39, 28];
%% Processing Chain

for numPkt = 1:numPackets 
    if walkSNR
        % Generate SNR value per packet using random walk algorithm biased
        % towards the mean SNR
       snrWalk = 0.9*snrWalk+0.1*baseSNR(numPkt)+rand(1)*maxJump*2-maxJump;
      % snrWalk= snr_eg(numPkt);
    end
    
    % Generate a single packet waveform
    txPSDU = randi([0,1],8*cfgVHT.PSDULength,1,'int8');
    txWave = wlanWaveformGenerator(txPSDU,cfgVHT,'IdleTime',5e-4);
    
    % Receive processing, including SNR estimation
    y = processPacket(txWave,snrWalk,tgacChannel,cfgVHT);
    
    % Plot equalized symbols of data carrying subcarriers
    if displayConstellation && ~isempty(y.EstimatedSNR)
        release(ConstellationDiagram);
        ConstellationDiagram.ReferenceConstellation = helperReferenceSymbols(cfgVHT);
        ConstellationDiagram.Title = ['Packet ' int2str(numPkt)];
        ConstellationDiagram(y.EqDataSym(:));
        drawnow 
    end
    
    % Store estimated SNR value for each packet
    if isempty(y.EstimatedSNR) 
        snrMeasured(1,numPkt) = NaN;
    else
        snrMeasured(1,numPkt) = y.EstimatedSNR;
    end
    
    % Determine the length of the packet in seconds including idle time
    packetLength(numPkt) = y.RxWaveformLength/sr;
    
    % Calculate packet error rate (PER)
    if isempty(y.RxPSDU)
        % Set the PER of an undetected packet to NaN
        ber(numPkt) = NaN;
    else
        [~,ber(numPkt)] = biterr(y.RxPSDU,txPSDU);
    end

    % Compare the estimated SNR to the threshold, and adjust the MCS value
    % used for the next packet
    MCS(numPkt) = cfgVHT.MCS;% Store current MCS value
    increaseMCS = (mean(y.EstimatedSNR) > snrUp((snrInd==0)+snrInd));
    decreaseMCS = (mean(y.EstimatedSNR) <= snrDown((snrInd==0)+snrInd));
    temp=snrInd;
    % Capture 5 packets for determining which algorithm to switch to
    if(i<6)                                     
        BER_array(1,i)= ber(numPkt);
        i=i+1;
        
    else
        i=5;
        while (i~=1)
            for j=1:1:i-1
                if (BER_array(1,j) == BER_array(1,j+1))
                    same=same+1 ;%trending
                end
               i=i-1; 
            end
        end
        
        % If majority packets have bit error rate change the algorithm,
        % else keep tha same
        if (same<=2)                        
            not_constant_snr= 1;
            same=0;
            constant_snr=0;
        else 
            constant_snr=1;
            same=0;
            not_constant_snr=0;
        end
    end
    if (not_constant_snr) %algorithm 1 : use this algorithm if the channel has high fluctuations in thevalus of SNR
        if (increaseMCS)
            while (mean(y.EstimatedSNR)>snrUp((temp==0)+temp)) %get the perfect snrIndex it should be at. 
                temp= temp+1;
            end
            if (temp-snrInd >3) % if its too far apart, increment index by 2 instead of 1. get it there sooner. 
                snrInd=snrInd+3;
            else    
                snrInd= snrInd+1;
            end
        end
    
        if (decreaseMCS)
            while (mean(y.EstimatedSNR) <= snrDown((temp==0)+temp))
                temp= temp-1;
            end
            snrInd = temp;
        end
   
        if (decreaseMCS==0 && increaseMCS ==0)% to make sure that if its still within the interval, is it decreasing towards to lower threshold value (i.e if its within the prev interval threshold. then just dont takw chance and decrement MCS) 
            if (ber(numPkt)>0) %if this wasnt there, I was not taking the risk. if im in the same interval and was in the risky zone, I would decrement MCS. now I take the risk, sometimes it works (good cuz throughput increases) and sometime it'' throw BER.(thats when I decide to decrement, if I still stay in the same interval for the next packet)--> increases my throughput, keeping BER same.
                if(snrInd ~= 1)
                    snr_down= snrUp(snrInd-1);
                        if ( y.EstimatedSNR < snrUp(snrInd-1))
                            snrInd= snrInd-1;
                        end
                end
            end
        end
        if ((snrInd-1 == 9) && (cfgVHT.ChannelBandwidth == "CBW20"))     % If the channel is 20MHz do not switch to MCS9
            cfg.VHT.MCS = 8;
        else
            cfgVHT.MCS = snrInd-1;  
        end
    end
    
    if (constant_snr)                       % select this algorithm if the channel has small changes in SNR values
        if(mean(y.EstimatedSNR) < 12)       % Based on measured EstimatedSNR directly switch to the MCS value
            cfgVHT.MCS = 0;
        elseif (mean(y.EstimatedSNR) >= 12 && mean(y.EstimatedSNR) < 15)
            cfgVHT.MCS = 1; 
        elseif (mean(y.EstimatedSNR) >= 15 && mean(y.EstimatedSNR) < 20)
            cfgVHT.MCS = 2;  
        elseif (mean(y.EstimatedSNR) >= 20 && mean(y.EstimatedSNR) < 21)
            cfgVHT.MCS = 3; 
        elseif (mean(y.EstimatedSNR) >= 21 && mean(y.EstimatedSNR) < 26)
            cfgVHT.MCS = 4; 
        elseif (mean(y.EstimatedSNR) >= 26 && mean(y.EstimatedSNR) < 29)
            cfgVHT.MCS = 5; 
        elseif (mean(y.EstimatedSNR) >= 29 && mean(y.EstimatedSNR) < 31)
            cfgVHT.MCS = 6; 
        elseif (mean(y.EstimatedSNR) >= 31 && mean(y.EstimatedSNR) < 32)
            cfgVHT.MCS = 7;
        elseif (mean(y.EstimatedSNR) >= 32 && mean(y.EstimatedSNR < 36))
            cfgVHT.MCS = 8;
        elseif (mean(y.EstimatedSNR) >= 36)
            if (cfgVHT.ChannelBandwidth == "CBW20")  % if the channel is 20 MHz do not switch to MCS9
                cfgVHT.MCS = 8; 
            else
                cfgVHT.MCS = 9;
            end
        end 
    end
end
    %snrInd = snrInd+increaseMCS-decreaseMCS
   

%% Display and Plot Simulation Results
% Display and plot simulation results
disp(['Overall data rate: ' num2str(8*cfgVHT.APEPLength*(numPackets-numel(find(ber)))/sum(packetLength)/1e6) ' Mbps']);
disp(['Overall packet error rate: ' num2str(numel(find(ber))/numPackets)]);

plotResults(ber,packetLength,snrMeasured,MCS,cfgVHT);

% Restore default stream
rng(s);

%% Conclusion and Further Exploration

displayEndOfDemoMessage(mfilename)

function Y = processPacket(txWave,snrWalk,tgacChannel,cfgVHT)
    % Pass the transmitted waveform through the channel, perform
    % receiver processing, and SNR estimation.
    
    chanBW = cfgVHT.ChannelBandwidth; % Channel bandwidth
    % Set the following parameters to empty for an undetected packet
    estimatedSNR = [];
    eqDataSym = [];
    noiseVarVHT = [];
    rxPSDU = [];
    
    % Get the number of occupied subcarriers in VHT fields
    [vhtData,vhtPilots] = helperSubcarrierIndices(cfgVHT,'VHT');
    Nst_vht = numel(vhtData)+numel(vhtPilots);
    Nfft = helperFFTLength(cfgVHT); % FFT length
    
    % Pass the waveform through the fading channel model
    rxWave = tgacChannel(txWave);
    
    % Create an instance of the AWGN channel for each transmitted packet
    awgnChannel = comm.AWGNChannel;
    awgnChannel.NoiseMethod = 'Signal to noise ratio (SNR)';
    % Normalization
    awgnChannel.SignalPower = 1/tgacChannel.NumReceiveAntennas;
    % Account for energy in nulls
    awgnChannel.SNR = snrWalk-10*log10(Nfft/Nst_vht);
    
    % Add noise
    rxWave = awgnChannel(rxWave);
    rxWaveformLength = size(rxWave,1); % Length of the received waveform
    
    % Recover packet
    ind = wlanFieldIndices(cfgVHT); % Get field indices
    pktOffset = wlanPacketDetect(rxWave,chanBW); % Detect packet
    
    if ~isempty(pktOffset) % If packet detected
        % Extract the L-LTF field for fine timing synchronization
        LLTFSearchBuffer = rxWave(pktOffset+(ind.LSTF(1):ind.LSIG(2)),:);
    
        % Start index of L-LTF field
        finePktOffset = wlanSymbolTimingEstimate(LLTFSearchBuffer,chanBW);
     
        % Determine final packet offset
        pktOffset = pktOffset+finePktOffset;
        
        if pktOffset<15 % If synchronization successful
            % Extract L-LTF samples from the waveform, demodulate and
            % perform noise estimation
            LLTF = rxWave(pktOffset+(ind.LLTF(1):ind.LLTF(2)),:);
            demodLLTF = wlanLLTFDemodulate(LLTF,chanBW);

            % Estimate noise power in non-HT fields
            noiseVarVHT = helperNoiseEstimate(demodLLTF,chanBW,cfgVHT.NumSpaceTimeStreams,'Per Antenna');

            % Extract VHT-LTF samples from the waveform, demodulate and
            % perform channel estimation
            VHTLTF = rxWave(pktOffset+(ind.VHTLTF(1):ind.VHTLTF(2)),:);
            demodVHTLTF = wlanVHTLTFDemodulate(VHTLTF,cfgVHT);
            chanEstVHTLTF = wlanVHTLTFChannelEstimate(demodVHTLTF,cfgVHT);

            % Recover equalized symbols at data carrying subcarriers using
            % channel estimates from VHT-LTF
            [rxPSDU,~,eqDataSym] = wlanVHTDataRecover( ...
                rxWave(pktOffset + (ind.VHTData(1):ind.VHTData(2)),:), ...
                chanEstVHTLTF,mean(noiseVarVHT),cfgVHT);
            
            % SNR estimation per receive antenna
            powVHTLTF = mean(VHTLTF.*conj(VHTLTF));
            estSigPower = powVHTLTF-noiseVarVHT;
            estimatedSNR = 10*log10(mean(estSigPower./noiseVarVHT));
        end
    end
    
    % Set output
    Y = struct( ...
        'RxPSDU',           rxPSDU, ...
        'EqDataSym',        eqDataSym, ...
        'RxWaveformLength', rxWaveformLength, ...
        'NoiseVar',         noiseVarVHT, ...
        'EstimatedSNR',     estimatedSNR);
    
end

function plotResults(ber,packetLength,snrMeasured,MCS,cfgVHT)
    % Visualize simulation results

    figure('Outerposition',[50 50 900 700])
    subplot(4,1,1);
    plot(MCS);
    xlabel('Packet Number')
    ylabel('MCS')
    title('MCS selected for transmission')

    subplot(4,1,2);
    plot(snrMeasured);
    xlabel('Packet Number')
    ylabel('SNR')
    title('Estimated SNR')

    subplot(4,1,3);
    plot(find(ber==0),ber(ber==0),'x') 
    hold on; stem(find(ber>0),ber(ber>0),'or') 
    if any(ber)
        legend('Successful decode','Unsuccessful decode') 
    else
        legend('Successful decode') 
    end
    xlabel('Packet Number')
    ylabel('BER')
    title('Instantaneous bit error rate per packet')

    subplot(4,1,4);
    windowLength = 3; % Length of the averaging window
    movDataRate = movsum(8*cfgVHT.APEPLength.*(ber==0),windowLength)./movsum(packetLength,windowLength)/1e6;
    plot(movDataRate)
    xlabel('Packet Number')
    ylabel('Mbps')
    title(sprintf('Throughput over last %d packets',windowLength))
    
end