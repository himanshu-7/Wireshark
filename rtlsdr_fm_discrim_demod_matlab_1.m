% RTL-SDR(rx) FM Mono Non Coherent Discriminator Demodulator MATLAB Script
% - This script can be used to non-coherently demodulate an FM signal. The
%   DSP operations carried out here are identical to those in the
%   "rtlsdr_rx_fm_mono_bbox.slx" Simulink model
% - When the script runs, two Spectrum Analyzers will appear. 'Spectrum
%   Analyzer Modulated' shows the signal received by the RTL-SDR, and
%   'Spectrum Analyzer Demodulated' shows the demodulated information
%   signal
% - You may change the RTL-SDR tuner frequency by modifying the value set
%	in "rtlsdr_freq", the tuner gain by modifying "rtlsdr_gain", and the
%   region of the de-emphasis filter by modifying "deemph_region"
% - Set the simulation run time by modifying "sim_time". Any value you
%   enter represents the number of seconds the script will loop for
% - NOTE: to end simulation early, use |Ctrl| + |C|

% - To view file information without running the code, type:
%       mfileinfo rtlsdr_fm_discrim_demod_matlab
%   into the MATLAB comman window

function fm_demod

%% PARAMETERS (edit)
offline          =   0;
rtlsdr_id        = '0';                         % stick ID (zero as only one RTL_SDR dongle is connected.)
rtlsdr_fc        = 102.7e6;                     % tuner centre frequency in Hz
rtlsdr_gain      = 120;                         % tuner gain in dB
rtlsdr_fs        = 2.4e6;                       % tuner sampling rate
rtlsdr_ppm       = 0;                           % tuner parts per million correction
rtlsdr_frmlen    = 256*25;                      % output data frame size (multiple of 5)
rtlsdr_datatype  = 'single';                    % output data type
deemph_region 	 = 'eu';                        % set to either eu or us
audio_fs         = 48e3;                        % audio output sampling rate
sim_time         = 60;                          % simulation time in seconds
fs1              = 240e3;                       % the intermediate sample rate after first discriminator


%% CALCULATIONS (do not edit)
rtlsdr_frmtime = rtlsdr_frmlen/rtlsdr_fs;       % calculate time for 1 frame of data
if deemph_region == 'eu'                        % find de-emphasis filter coeff
    [num,den] = butter(1,3183.1/(audio_fs/2));
elseif deemph_region == 'us'
    [num,den] = butter(1,2122.1/(audio_fs/2));
else
    error('Invalid region for de-emphasis filter - must be either "eu" or "us"');
end


%% SYSTEM OBJECTS (do not edit)

 %check if running offline
 if offline == 1

    % link to an rtl-sdr data file
    obj_rtlsdr = import_rtlsdr_data(...
           'filepath', offline_filepath,...
          'frm_size', rtlsdr_frmlen,...
          'data_type',rtlsdr_datatype);

      % reduce sampling rate
      rtlsdr_fs = 240e3;

      obj_decmtr = dsp.FIRDecimator(...
        'DecimationFactor', 10,...
        'Numerator', fir1(64,0.07));

      obj_decmtr_1 = dsp.FIRDecimator(...
        'DecimationFactor', 5,...
        'Numerator', fir1(64,0.4));


 else

     % link to a physical rtl-sdr dongle
        obj_rtlsdr = comm.SDRRTLReceiver(...
        rtlsdr_id,...
        'CenterFrequency', rtlsdr_fc,...
        'EnableTunerAGC', false,...
        'TunerGain', rtlsdr_gain,...
        'SampleRate', rtlsdr_fs, ...
        'SamplesPerFrame', rtlsdr_frmlen,...
        'OutputDataType', rtlsdr_datatype,...
        'FrequencyCorrection', rtlsdr_ppm);

    % Decimation reduces the original sampling rate of a sequence to a
    % lower rate. The fir1 provides a window-based low pass FIR filter design.

    % fir decimator object - fs = 2.4MHz downto 240kHz with decimation factor of
    % N1=10
    obj_decmtr = dsp.FIRDecimator(...
        'DecimationFactor', 10,...
        'Numerator', fir1(64,0.02));
    % using the formula(fs' * 2 * pi )/ (fs) to obtain the LPF cutoff freq
    % of 0.02 in radian/sample

    % fir decimator object 2 - fs = 2.4MHz downto 48kHz with decimation factor of
    % N2=5
    obj_decmtr_1 = dsp.FIRDecimator(...
        'DecimationFactor', 5,...
        'Numerator', fir1(64,0.005));
    % using the formula(fs' * 2 * pi )/ (fs1) to obtain the LPF cutoff freq
    % 0.005 in radian/sample
end;

% iir de-emphasis filter
obj_deemph = dsp.IIRFilter(...
    'Numerator', num,...
    'Denominator', den);

% delay
obj_delay = dsp.Delay;

% audio output
obj_audio = dsp.AudioPlayer(audio_fs);

% spectrum analyzers
obj_spectrummod   = dsp.SpectrumAnalyzer(...
    'Name', 'Spectrum Analyzer Modulated',...
    'Title', 'Spectrum Analyzer Modulated',...
    'SpectrumType', 'Power density',...
    'FrequencySpan', 'Full',...
    'SampleRate', rtlsdr_fs);
 obj_spectrumdeci_1   = dsp.SpectrumAnalyzer(...
     'Name', 'Spectrum Analyzer of the ouput of first_decimator',...
     'Title', 'Spectrum Analyzer 1st decimator o/p',...
     'SpectrumType', 'Power density',...
     'FrequencySpan', 'Full',...
     'SampleRate', fs1);
obj_spectrumdemod = dsp.SpectrumAnalyzer(...
    'Name', 'Spectrum Analyzer Demodulated',...
    'Title', 'Spectrum Analyzer Demodulated',...
    'SpectrumType', 'Power density',...
    'FrequencySpan', 'Full',...
    'SampleRate', audio_fs);

%% SIMULATION

% if using RTL-SDR, check first if RTL-SDR is active
if offline == 0
    if ~isempty(sdrinfo(obj_rtlsdr.RadioAddress))
    else
        error(['RTL-SDR failure. Please check connection to ',...
            'MATLAB using the "sdrinfo" command.']);
    end
end

% reset run_time to 0 (secs)
run_time = 0;

% loop while run_time is less than sim_time
while run_time < sim_time

    % fetch a frame from obj_rtlsdr
    rtlsdr_data = step(obj_rtlsdr);
    % update 'modulated' spectrum analyzer window with new data.
    % step function calls the System object and runs the algorithm.
    step(obj_spectrummod, rtlsdr_data);

    %the output of the RTL-SDR block now goes into the i/p of the first
    %decimator block ( consists of an LPF and downsampler).
    %Inorder to keep out the band noise and interference from the
    %discriminator as much as possible.
    data_dec = step(obj_decmtr,rtlsdr_data);
    %spectrum analyzer at the output of the first decimator
    step(obj_spectrumdeci_1, data_dec);

    % implement frequency discriminator(combination of the delay,
    % conjugate, product and angular blocks). Discriminator is key to
    % demodulating FM.
    discrim_delay = step(obj_delay,data_dec);
    discrim_conj  = conj(data_dec);
    discrim_pd    = discrim_delay.*discrim_conj;
    discrim_arg   = angle(discrim_pd);

    %Spectrum analyzer for the output of the discriminator
    step(obj_spectrumdeci_1, discrim_arg);

    %the output of the decimator block now goes into the i/p of the second
    %decimator block
    data_dec = step(obj_decmtr_1,discrim_arg);
    %spectrum analyzer after the second discriminator block.
    step(obj_spectrumdiscrim_1, discrim_arg);

    %de-emphasis filter data
    data_deemph = step(obj_deemph,data_dec);

    % update 'demodulated' spectrum analyzer window with new data
    step(obj_spectrumdemod, data_deemph);

    % output demodulated signal to speakers
    step(obj_audio,data_deemph);

    % update run_time after processing another frame
    run_time = run_time + rtlsdr_frmtime;

end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Software, Simulation Examples and Design Exercises Licence Agreement  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  This license agreement refers to the simulation examples, design
%  exercises and files, and associated software MATLAB and Simulink
%  resources that accompany the book:
%
%    Title: Software Defined Radio using MATLAB & Simulink and the RTL-SDR
%    Published by Strathclyde Academic Media, 2015
%    Authored by Robert W. Stewart, Kenneth W. Barlee, Dale S.W. Atkinson,
%    and Louise H. Crockett
%
%  and made available as a download from www.desktopSDR.com or variously
%  acquired by other means such as via USB storage, cloud storage, disk or
%  any other electronic or optical or magnetic storage mechanism. These
%  files and associated software may be used subject to the terms of
%  agreement of the conditions below:
%
%    Copyright � 2015 Robert W. Stewart, Kenneth W. Barlee,
%    Dale S.W. Atkinson, and Louise H. Crockett. All rights reserved.
%
%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions are
%  met:
%
%   (1) Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%   (2) Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the
%       distribution.
%
%   (3) Neither the name of the copyright holder nor the names of its
%       contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
%
%   (4) In all cases, the software is, and all modifications and
%       derivatives of the software shall be, licensed to you solely for
%       use in conjunction with The MathWorks, Inc. products and service
%       offerings.
%
%  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%%  Audio Tracks used in Simulations Examples and Design Exercises
%
%  The music and vocal files used within the Examples files and software
%  within the book were variously written, arranged, performed, recorded
%  and produced by Garrey Rice, Adam Struth, Jamie Struth, Iain
%  Thistlethwaite and also Marshall Craigmyle who collectively, and
%  individually where appropriate, assert and retain all of their
%  copyright, performance and artistic rights. Permission to use and
%  reproduce this music is granted for all purposes associated with
%  MATLAB and Simulink software and the simulation examples and design
%  exercises files that accompany this book. Requests to use the music
%  for any other purpose should be directed to: info@desktopSDR.com. For
%  information on music track names, full credits, and links to the
%  musicians please refer to www.desktopSDR.com/more/audio.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
