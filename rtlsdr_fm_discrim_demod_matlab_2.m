% RTL-SDR(rx) FM Mono Non Coherent Discriminator Demodulator MATLAB Script
%Group- SharkHunterz (Priyanka Bhat and Himanshu Shah)


function fm_demod

%% PARAMETERS (edit)
rtlsdr_id        = '0';                         % stick ID (zero as only one RTL_SDR dongle is connected.)
rtlsdr_fc        = 102.7e6;                     % tuner centre frequency in Hz
rtlsdr_gain      = 50;                          % tuner gain in dB
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
        'Numerator', fir1(63,0.1));

    % fir decimator object 2 - fs = 2.4MHz downto 48kHz with decimation factor of
    % N2=5
    obj_decmtr_1 = dsp.FIRDecimator(...
        'DecimationFactor', 5,...
        'Numerator', fir1(63,0.125));

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

    if ~isempty(sdrinfo(obj_rtlsdr.RadioAddress))
    else
        error(['RTL-SDR failure. Please check connection to ',...
            'MATLAB using the "sdrinfo" command.']);
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


