clear
close all
clc

%% FREQ VARIABLES

%fFreq = 7458.62;
%fFreq = 2217.46;
fFreq = 311.13;
WAVEFOMRNUM = 512;
I2S_FREQ = 48000;
%error with fd?
%fd = (fFreq * WAVEFOMRNUM) / I2S_FREQ;
fd = I2S_FREQ / fFreq;

%% WAVEFORMGENS

%% DDS WAVEFORMS 512

sinaudio_data = zeros(512, 1);
triaudio_data = zeros(512, 1);
sawaudio_data = zeros(512, 1);
sqraudio_data = zeros(512, 1);

audio_amplitude = 1;
audio_pi = atan(1)*4;
sqr_percent = 50;


for u16Index = 0:(WAVEFOMRNUM-1)

        % sinus
        sinaudio_data(u16Index + 1) = ...
            audio_amplitude*(sin(2*audio_pi*u16Index /  WAVEFOMRNUM));

        %triangle
        if(u16Index <= ( WAVEFOMRNUM / 2))
            triaudio_data(u16Index + 1) = ...
                audio_amplitude * (u16Index /  (WAVEFOMRNUM/2)  );
        else
            triaudio_data(u16Index + 1) = ...
                audio_amplitude * ( 1 -  ...
                ( (u16Index-(WAVEFOMRNUM/2)) /  (WAVEFOMRNUM/2) ));
        end


        %sawtooth
        if(u16Index == (WAVEFOMRNUM-1))
            sawaudio_data(u16Index + 1) = 0;
        else
            sawaudio_data(u16Index + 1) = ...
                audio_amplitude*((u16Index+1) /  WAVEFOMRNUM);
        end

        % square
        if(u16Index <= round(WAVEFOMRNUM*sqr_percent/100))
            sqraudio_data(u16Index + 1) = audio_amplitude;
        else
            sqraudio_data(u16Index + 1) = 0;
        end
end


fDdsFd = (fFreq * WAVEFOMRNUM) / I2S_FREQ;
fDdsNum = 1;
AudioDataSin = zeros(512, 1);
AudioDataSaw = zeros(512, 1);
AudioDataSqr = zeros(512, 1);
AudioDataTri = zeros(512, 1);

for u16Index = 1:WAVEFOMRNUM
    idds = floor(fDdsNum) + 1;
    %gen waveform
    AudioDataSin(u16Index) = sinaudio_data(idds);
    AudioDataSaw(u16Index) = sawaudio_data(idds);
    AudioDataSqr(u16Index) = sqraudio_data(idds);
    AudioDataTri(u16Index) = triaudio_data(idds);
    fDdsNum = fDdsNum + fDdsFd;
    if fDdsNum >= WAVEFOMRNUM
      fDdsNum = fDdsNum - WAVEFOMRNUM;
    end
end

%% BLIT

add=100;
letsgo=0;
testarray = zeros(512, 1);
testarray2 = zeros(512, 1);
index = 1;
minus = 0;
countindex = 0;
startindex = 0;
while index < (WAVEFOMRNUM+1)
    if letsgo == add
        testarray(index) = 1;

        letsgo = letsgo - add;
        minus = -1/100;
        if countindex == 0
            startindex = index;
            countindex = countindex + 1;
        end
    end
    testarray2(index) = testarray(index) + minus;

    % if letsgo == add
    %     letsgo = letsgo - add;
    % end
    % if letsgo == 0
    %     testarray(index) = 1;
    % end
    % 
    % testarray(index) = (letsgo/100)-0.5;



    index = index + 1;
    letsgo = letsgo + 1;
    
    
end

meandcofftest = mean(testarray2);

%DC offset
%[yndcofftest1, fDcOfftest] = dcoff(100, fFreq, (I2S_FREQ / 100), testarray);

%yndcofftest2 = testarray(100:512) - 1/200;
%yndcofftest2 = testarray - 1/200;

%leaky integrator
Etest=0;
n1stelementtest = 0;
[yntest, n1stelementtest] = leakyint(Etest, n1stelementtest, testarray2);

for index=startindex:WAVEFOMRNUM
    yntest(index) = yntest(index) - 0.5;
end

%% BLIT
%arrfBlistNum = zeros(512, 1);
u32AudioAmpBlist = 1;
fBlistIndex = 1;
[ynblit, fBlistIndex] = blitgen(fd, fBlistIndex, u32AudioAmpBlist);

%% BIPOLAR BLIT
%arrfBlistNum = zeros(512, 1);
u32AudioAmpBlistbi = 1;
fBlistIndexbi = 1;
u8Flagbi = 1;
[ynblitbi, fbiBlistIndex] = biblitgen(fd/2, fBlistIndexbi, u32AudioAmpBlistbi, u8Flagbi);

%% BLIT & B-slave sawtooth

%meandcoff = mean(ynblit);

%DC offset
[yndcoffsaw, fDcOffsaw] = dcoff(fd, fFreq, I2S_FREQ, ynblit);


%leaky integrator
%Esaw=0.00005;
Esaw=0;
n1stelementsaw = 0;
[ynsaw, n1stelementsaw] = leakyint(Esaw, n1stelementsaw, yndcoffsaw);
%[ynsaw, n1stelementsaw] = leakyint(Esaw, n1stelementsaw, ynblit);

%-0.5 dc
for index = 1:WAVEFOMRNUM
    ynsaw(index) = ynsaw(index) - 0.5;
end

%% BLIT & B-slave squarewave

%leaky integrator
Esqr=0;
n1stelementsqr = 0;
[ynsqr, n1stelementsqr] = leakyint(Esqr, n1stelementsqr, ynblitbi);

%-0.5 dc
for index = 1:WAVEFOMRNUM
    ynsqr(index) = ynsqr(index) - 0.5;
end

%% BLIT & B-slave trianglewave

%leaky integrator
Etr=0;
%Etr=0.09;
n1stelementtr = 0;
[yntr, n1stelementtr] = leakyint(Etr, n1stelementtr, ynsqr);

%C amplification
%On 2217.46 Hz
%Ctr = 1/8.5;
%Ctr = 1;
%yntr = Ctr * yntr - 0.25;

%On 311.13 Hz
Ctr = 1/38;
yntr = Ctr * yntr - 0.5;


%% ***********
%% ***PLOTS***
%% ***********

%% Power spectral things

fs = I2S_FREQ;
m = 512;       % original sample length
n = pow2(nextpow2(m));  % transform length
%f = (0:n-1)*(fs/n)/10; ???/10???

%DDS
figure(1)
% DDS sinus waveform
subplot(4,2,1)
plot(AudioDataSin);
title('DDS sinus waveform')
xlabel('sample')
ylabel('amplitude')
% DDS sinus spectrum
y = fft(AudioDataSin(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(4,2,2)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('DDS sinus spectrum')
xlabel('Frequency')
ylabel('Magnitude')

% DDS sawtooth waveform
subplot(4,2,3)
plot(AudioDataSaw);
title('DDS sawtooth waveform')
xlabel('sample')
ylabel('amplitude')
% DDS sawtooth spectrum
y = fft(AudioDataSaw(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(4,2,4)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('DDS sawtooth spectrum')
xlabel('Frequency')
ylabel('Magnitude')

% DDS squarewave waveform
subplot(4,2,5)
plot(AudioDataSqr);
title('DDS squarewave waveform')
xlabel('sample')
ylabel('amplitude')
% DDS squarewave spectrum
y = fft(AudioDataSqr(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(4,2,6)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('DDS squarewave spectrum')
xlabel('Frequency')
ylabel('Magnitude')

% DDS trianglewave waveform
subplot(4,2,7)
plot(AudioDataTri);
title('DDS trianglewave waveform')
xlabel('sample')
ylabel('amplitude')
% DDS trianglewave spectrum
y = fft(AudioDataTri(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(4,2,8)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('DDS trianglewave spectrum')
xlabel('Frequency')
ylabel('Magnitude')


% BLIT
figure(2)
% BLIT waveform
subplot(2,2,1)
plot(testarray);
title('BLIT waveform')
xlabel('sample')
ylabel('amplitude')
% BLIT spectrum
y = fft(testarray(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;  
subplot(2,2,2)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('BLIT spectrum')
xlabel('Frequency')
ylabel('Magnitude')


%BLIT & B-slave waveform
subplot(2,2,3)
plot(ynblit);
title('BLIT & B-slave waveform')
xlabel('sample')
ylabel('amplitude')
%BLIT & B-slave spectrum
y = fft(ynblit(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(2,2,4)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('BLIT & B-slave spectrum')
xlabel('Frequency')
ylabel('Magnitude')

figure(3)
%Bipolar BLIT & B-slave waveform
subplot(1,2,1)
plot(ynblitbi);
title('Bipolar BLIT & B-slave waveform')
xlabel('sample')
ylabel('amplitude')
%Bipolar BLIT & B-slave spectrum
y = fft(ynblitbi(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(1,2,2)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('Bipolar BLIT & B-slave spectrum')
xlabel('Frequency')
ylabel('Magnitude')

figure(4)
%BLIT & B-slave sawtooth waveform
subplot(3,2,1)
plot(ynsaw);
title('BLIT & B-slave sawtooth waveform')
xlabel('sample')
ylabel('amplitude')
%BLIT & B-slave sawtooth spectrum
y = fft(ynsaw(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(3,2,2)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('BLIT & B-slave sawtooth spectrum')
xlabel('Frequency')
ylabel('Magnitude')

%BLIT & B-slave squarewave waveform
subplot(3,2,3)
plot(ynsqr);
title('BLIT & B-slave squarewave')
xlabel('sample')
ylabel('amplitude')
%BLIT & B-slave squarewave spectrum
y = fft(ynsqr(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(3,2,4)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('BLIT & B-slave sawtooth spectrum')
xlabel('Frequency')
ylabel('Magnitude')

%BLIT & B-slave trianglewave waveform
subplot(3,2,5)
plot(yntr);
title('BLIT & B-slave trianglewave')
xlabel('sample')
ylabel('amplitude')
%BLIT & B-slave trianglewave spectrum
y = fft(yntr(1:WAVEFOMRNUM, 1),n);        % DFT of signal
f = (0:n-1)*(fs/n);
power = abs(y)./n;      
subplot(3,2,6)
plot(f(2:floor(n/2)),power(2:floor(n/2)))
title('BLIT & B-slave sawtooth spectrum')
xlabel('Frequency')
ylabel('Magnitude')

%% **********
%% ***FUNC***
%% **********

%% FUNC BLIT GEN 

function [arri32BlistBuffer, fBlistIndex] = blitgen(fd, fBlistIndex, u32AudioAmpBlist)
    
    arri32BlistBuffer = zeros(512, 1);
    fBlistDelay = 1;
    WAVEFOMRNUM = 512;

    while fBlistIndex < (WAVEFOMRNUM + 2 + 1)
        
        i32BlistIndexFloor = floor(fBlistIndex); 
        fBlistNum = fBlistIndex - i32BlistIndexFloor;
    
        if ((i32BlistIndexFloor - 2 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 2 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBuffer(i32BlistIndexFloor - 2 + 1) = u32AudioAmpBlist * 1/6 * (2+(fBlistNum-2*fBlistDelay))^3;
            %arrfBlistNum(i32BlistIndexFloor - 2 + 1) = fBlistNum;
        end
    
        if ((i32BlistIndexFloor - 1 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 1 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBuffer(i32BlistIndexFloor - 1 + 1) = u32AudioAmpBlist * (2/3 - (fBlistNum-1*fBlistDelay)^2 - 1/2 * (fBlistNum-1*fBlistDelay)^3);
            %arrfBlistNum(i32BlistIndexFloor - 1 + 1) = fBlistNum;
        end
    
        if ((i32BlistIndexFloor - 0 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 0 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBuffer(i32BlistIndexFloor - 0 + 1) = u32AudioAmpBlist * (2/3 - (fBlistNum+0*fBlistDelay)^2 + 1/2 * (fBlistNum+0*fBlistDelay)^3);
            %arrfBlistNum(i32BlistIndexFloor - 0 + 1) = fBlistNum;
        end
        
        if ((i32BlistIndexFloor + 1 + 1) >= (0 + 1)) && ((i32BlistIndexFloor + 1 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBuffer(i32BlistIndexFloor + 1 + 1) = u32AudioAmpBlist * 1/6 * (2-(fBlistNum+1*fBlistDelay))^3;
            %arrfBlistNum(i32BlistIndexFloor + 1 + 1) = fBlistNum;
        end
        
        fBlistIndex = fBlistIndex + fd;
    
    end
    
    fBlistIndex = fBlistIndex - WAVEFOMRNUM;
end

%% FUNC BIPOLAR BLIT GEN
function [arri32BlistBufferbi, fBlistIndexbi] = biblitgen(fdbi, fBlistIndexbi, u32AudioAmpBlistbi, u8Flagbi)
    
    arri32BlistBufferbi = zeros(512, 1);
    fBlistDelay = 1;
    WAVEFOMRNUM = 512;

    while fBlistIndexbi < (WAVEFOMRNUM + 2 + 1)

        i32BlistIndexFloor = floor(fBlistIndexbi); 
        fBlistNum = fBlistIndexbi - i32BlistIndexFloor;
    
        if ((i32BlistIndexFloor - 2 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 2 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBufferbi(i32BlistIndexFloor - 2 + 1) = u8Flagbi * u32AudioAmpBlistbi * 1/6 * (2+(fBlistNum-2*fBlistDelay))^3;
            %arrfBlistNum(i32BlistIndexFloor - 2 + 1) = fBlistNum;
        end
    
        if ((i32BlistIndexFloor - 1 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 1 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBufferbi(i32BlistIndexFloor - 1 + 1) = u8Flagbi * u32AudioAmpBlistbi * (2/3 - (fBlistNum-1*fBlistDelay)^2 - 1/2 * (fBlistNum-1*fBlistDelay)^3);
            %arrfBlistNum(i32BlistIndexFloor - 1 + 1) = fBlistNum;
        end
    
        if ((i32BlistIndexFloor - 0 + 1) >= (0 + 1)) && ((i32BlistIndexFloor - 0 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBufferbi(i32BlistIndexFloor - 0 + 1) = u8Flagbi * u32AudioAmpBlistbi * (2/3 - (fBlistNum+0*fBlistDelay)^2 + 1/2 * (fBlistNum+0*fBlistDelay)^3);
            %arrfBlistNum(i32BlistIndexFloor - 0 + 1) = fBlistNum;
        end
        
        if ((i32BlistIndexFloor + 1 + 1) >= (0 + 1)) && ((i32BlistIndexFloor + 1 + 1) < (WAVEFOMRNUM + 1))
            arri32BlistBufferbi(i32BlistIndexFloor + 1 + 1) = u8Flagbi * u32AudioAmpBlistbi * 1/6 * (2-(fBlistNum+1*fBlistDelay))^3;
            %arrfBlistNum(i32BlistIndexFloor + 1 + 1) = fBlistNum;
        end
        
        fBlistIndexbi = fBlistIndexbi + fdbi;

        u8Flagbi = -1 * u8Flagbi;
    
    end
    
    fBlistIndexbi = fBlistIndexbi - WAVEFOMRNUM;
end


%% FUNC LEAKY INTEGRATOR
function [yn, n1stelement] = leakyint(E, n1stelement, xn)
    WAVEFOMRNUM = 512;
    yn = zeros(512, 1);
    
    for u16LeakyIndex = (0+1):WAVEFOMRNUM
        if (u16LeakyIndex == (0+1))
            yn(u16LeakyIndex) = xn(u16LeakyIndex) + (1-E) * n1stelement;
        else
            yn(u16LeakyIndex) = xn(u16LeakyIndex) + (1-E) * yn(u16LeakyIndex - 1);
        end

        if (u16LeakyIndex == WAVEFOMRNUM)   %WAVEFOMRNUM-1 in c
            n1stelement = yn(u16LeakyIndex);
        end
    end
end

%% FUNC DC OFFSET
function [yndcoff, fDcOff] = dcoff(fd, fFreq, I2S_FREQ, ynin)
    yndcoff = zeros(512, 1);
    WAVEFOMRNUM = 512;
    fDcOff = fFreq/I2S_FREQ;

    yndcoff = ynin - (1 / fd);
    
    %meanyin = mean(ynin);
    %yndcoff = ynin - meanyin;
    
    %yndcoff = ynin - fDcOff * 512 / fd;

    % for u16DCOffIndex = (0+1):WAVEFOMRNUM
    %     if (ynin(u16DCOffIndex)>0)
    %         yndcoff(u16DCOffIndex) = ynin(u16DCOffIndex) - fDcOff;
    %     else
    %         yndcoff(u16DCOffIndex) = 0;
    %     end
    % end
end