clc;
close all;
clear all;
fs = 8000;
M = fs/2 + 1;
frameSize = 2048;

[B,A] = cheby2(4,20,[0.1 0.7]);
impulseResponseGenerator = dsp.IIRFilter('Numerator', [zeros(1,6) B], ...
    'Denominator', A);

FVT = fvtool(impulseResponseGenerator);  % Analyze the filter
FVT.Color = [1 1 1];
roomImpulseResponse = impulseResponseGenerator( ...
        (log(0.99*rand(1,M)+0.01).*sign(randn(1,M)).*exp(-0.002*(1:M)))');
roomImpulseResponse = roomImpulseResponse/norm(roomImpulseResponse)*4;
room = dsp.FIRFilter('Numerator', roomImpulseResponse');

fig = figure;
plot(0:1/fs:0.5, roomImpulseResponse);
xlabel('Time (s)');
ylabel('Amplitude');
title('Room Impulse Response');
fig.Color = [1 1 1];
[Fname1,Pname1] = uigetfile('nearspeech.wav','D:\Control_system_project\Acoustic-Echo-Canceller-master');

filename = strcat(Pname1,Fname1);

[v,Fs1] = audioread(filename);

player          = audioDeviceWriter('SupportVariableSizeInput', true, ...
                                    'BufferSize', 512, 'SampleRate', fs);
nearSpeechSrc   = dsp.SignalSource('Signal',v,'SamplesPerFrame',frameSize);
nearSpeechScope = timescope('SampleRate', fs, 'TimeSpanSource','Property',...
                    'TimeSpan', 35, 'TimeSpanOverrunAction', 'Scroll', ...
                    'YLimits', [-1.5 1.5], ...
                    'BufferLength', length(v), ...
                    'Title', 'Near-End Speech Signal', ...
                    'ShowGrid', true);

% Stream processing loop
while(~isDone(nearSpeechSrc))
    % Extract the speech samples from the input signal
    nearSpeech = nearSpeechSrc();
    % Send the speech samples to the output audio device
    player(nearSpeech);
    % Plot the signal
    nearSpeechScope(nearSpeech);
end
release(nearSpeechScope);
[Fname2,Pname2] = uigetfile('farspeech.wav','D:\Control_system_project\Acoustic-Echo-Canceller-master');

filename2 = strcat(Pname2,Fname2);
[x,Fs2] = audioread(filename2);
farSpeechSrc    = dsp.SignalSource('Signal',x,'SamplesPerFrame',frameSize);
farSpeechSink   = dsp.SignalSink;
farSpeechScope  = timescope('SampleRate', fs, 'TimeSpanSource','Property',...
                    'TimeSpan', 35, 'TimeSpanOverrunAction', 'Scroll', ...
                    'YLimits', [-0.5 0.5], ...
                    'BufferLength', length(x), ...
                    'Title', 'Far-End Speech Signal', ...
                    'ShowGrid', true);

% Stream processing loop
while(~isDone(farSpeechSrc))
    % Extract the speech samples from the input signal
    farSpeech = farSpeechSrc();
    % Add the room effect to the far-end speech signal
    farSpeechEcho = room(farSpeech);
    % Send the speech samples to the output audio device
    player(farSpeechEcho);
    % Plot the signal
    farSpeechScope(farSpeech);
    % Log the signal for further processing
    farSpeechSink(farSpeechEcho);
end
release(farSpeechScope);

reset(nearSpeechSrc);
farSpeechEchoSrc = dsp.SignalSource('Signal', farSpeechSink.Buffer, ...
                    'SamplesPerFrame', frameSize);
micSink         = dsp.SignalSink;
micScope        = timescope('SampleRate', fs,'TimeSpanSource','Property',...
                    'TimeSpan', 35, 'TimeSpanOverrunAction', 'Scroll',...
                    'YLimits', [-1 1], ...
                    'BufferLength', length(x), ...
                    'Title', 'Microphone Signal', ...
                    'ShowGrid', true);

% Stream processing loop
while(~isDone(farSpeechEchoSrc))
    % Microphone signal = echoed far-end + near-end + noise
    micSignal = farSpeechEchoSrc() + nearSpeechSrc() + ...
                0.001*randn(frameSize,1);
    % Send the speech samples to the output audio device
    player(micSignal);
    % Plot the signal
    micScope(micSignal);
    % Log the signal
    micSink(micSignal);
end
release(micScope);

%constuct frquency-domain apdaptive filter

echoCanceller    = dsp.FrequencyDomainAdaptiveFilter('Length', 2048, ...
                    'StepSize', 0.025, ...
                    'InitialPower', 0.01, ...
                    'AveragingFactor', 0.98, ...
                    'Method', 'Unconstrained FDAF');

AECScope1   = timescope(4, fs, ...
                'LayoutDimensions', [4,1],'TimeSpanSource','Property', ...
                'TimeSpan', 35, 'TimeSpanOverrunAction', 'Scroll', ...
                'BufferLength', length(x));

AECScope1.ActiveDisplay = 1;
AECScope1.ShowGrid      = true;
AECScope1.YLimits       = [-1.5 1.5];
AECScope1.Title         = 'Near-End Speech Signal';

AECScope1.ActiveDisplay = 2;
AECScope1.ShowGrid      = true;
AECScope1.YLimits       = [-1.5 1.5];
AECScope1.Title         = 'Microphone Signal';

AECScope1.ActiveDisplay = 3;
AECScope1.ShowGrid      = true;
AECScope1.YLimits       = [-1.5 1.5];
AECScope1.Title         = 'Output of Acoustic Echo Canceller mu=0.025';

AECScope1.ActiveDisplay = 4;
AECScope1.ShowGrid      = true;
AECScope1.YLimits       = [0 50];
AECScope1.YLabel        = 'ERLE (dB)';
AECScope1.Title         = 'Echo Return Loss Enhancement mu=0.025';

% Near-end speech signal
release(nearSpeechSrc);
nearSpeechSrc.SamplesPerFrame = frameSize;

% Far-end speech signal
release(farSpeechSrc);
farSpeechSrc.SamplesPerFrame = frameSize;

% Far-end speech signal echoed by the room
release(farSpeechEchoSrc);
farSpeechEchoSrc.SamplesPerFrame = frameSize;

%ERLE -- Echo return loss management
%smoothed measure of the amount (in dB) that the echo has been attenuated.


diffAverager = dsp.FIRFilter('Numerator', ones(1,1024));
farEchoAverager = clone(diffAverager);
setfilter(FVT,diffAverager);

micSrc = dsp.SignalSource('Signal', micSink.Buffer, ...
    'SamplesPerFrame', frameSize);

% Stream processing loop - adaptive filter step size = 0.025
while(~isDone(nearSpeechSrc))
    nearSpeech = nearSpeechSrc();
    farSpeech = farSpeechSrc();
    farSpeechEcho = farSpeechEchoSrc();
    micSignal = micSrc();
    % Apply FDAF
    [y,e] = echoCanceller(farSpeech, micSignal);
    % Send the speech samples to the output audio device
    player(e);
    % Compute ERLE
    erle = diffAverager((e-nearSpeech).^2)./ farEchoAverager(farSpeechEcho.^2);
    erledB = -10*log10(erle);
    % Plot near-end, far-end, microphone, AEC output and ERLE
    AECScope1(nearSpeech, micSignal, e, erledB);
end
release(AECScope1);

% changing the step size value in FDAF


reset(echoCanceller);
echoCanceller.StepSize = 0.04;

AECScope2 = clone(AECScope1);
AECScope2.ActiveDisplay = 3;
AECScope2.Title = 'Output of Acoustic Echo Canceller mu=0.04';
AECScope2.ActiveDisplay = 4;
AECScope2.Title = 'Echo Return Loss Enhancement mu=0.04';

reset(nearSpeechSrc);
reset(farSpeechSrc);
reset(farSpeechEchoSrc);
reset(micSrc);
reset(diffAverager);
reset(farEchoAverager);

% Stream processing loop - adaptive filter step size = 0.04
while(~isDone(nearSpeechSrc))
    nearSpeech = nearSpeechSrc();
    farSpeech = farSpeechSrc();
    farSpeechEcho = farSpeechEchoSrc();
    micSignal = micSrc();
    % Apply FDAF
    [y,e] = echoCanceller(farSpeech, micSignal);
    % Send the speech samples to the output audio device
    player(e);
    % Compute ERLE
    erle = diffAverager((e-nearSpeech).^2)./ farEchoAverager(farSpeechEcho.^2);
    erledB = -10*log10(erle);
    % Plot near-end, far-end, microphone, AEC output and ERLE
    AECScope2(nearSpeech, micSignal, e, erledB);
end

release(nearSpeechSrc);
release(farSpeechSrc);
release(farSpeechEchoSrc);
release(micSrc);
release(diffAverager);
release(farEchoAverager);
release(echoCanceller);
release(AECScope2);