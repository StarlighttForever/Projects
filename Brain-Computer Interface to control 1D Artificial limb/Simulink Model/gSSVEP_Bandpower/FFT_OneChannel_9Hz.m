% FFT of One Channel
% M = h5read('11hz_SESSION32016.09.06_15.26.39.hdf5','/RawData/Samples');
M=load('SSVEP_bandpower_rawdata.mat');

Fs = 256;               % Sampling frequency
                      %Number of samples, L = 9186
figure (1);
for i=1:2
% y = M(i,1500:end);
y = M.y(i,2560:end);
%y = M(i,6494:7813);
% y = M(i,9756:10400);

% y2 = M(i,30208:32000);
% y3 = M(i,41728:43520);
% y4 = M(i,53504:55296);
% y5 = M(i,65280:67072);
% y = [y1,y2,y3,y4,y5];
% y = M(i,18432:19968);
L = length(y);  
NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
amplitude = 2*abs(Y1(1:NFFT/2+1));
power = (abs(Y1(1:NFFT/2+1)).^2);

% Plot single-sided amplitude spectrum.
%  figure(1);
%  subplot(3,2,i)
%  plot(frequency,amplitude); 
%  axis ([0 12 0 3]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('Magnitude')

%hold on;
% figure(1);
subplot(2,2,i)
plot(frequency,power); 
axis ([0 15 0 5]);
%title('Single-Sided Amplitude Spectrum of y(t)')
%title('14 Hz')

xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
grid on
end
% 
% figure (2);
% for i=1:8
% 
% % y1 = M(i,15872:17664);
% y = M(i,27648:29440);
% % y3 = M(i,39424:41216);
% % y4 = M(i,51200:52992);
% % y5 = M(i,62976:64768);
% % y = [y1,y2,y3,y4,y5];
% % y = M(i,18432:19968);
% L = length(y);  
% NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
% Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
% frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
% amplitude = 2*abs(Y1(1:NFFT/2+1));
% power = (abs(Y1(1:NFFT/2+1)).^2);
% 
% % Plot single-sided amplitude spectrum.
% %  figure(1);
% %  subplot(3,2,i)
% %  plot(frequency,amplitude); 
% %  axis ([0 12 0 3]);
% % %title('Single-Sided Amplitude Spectrum of y(t)')
% % title('14 Hz')
% % 
% % xlabel('Frequency (Hz)')
% % ylabel('Magnitude')
% 
% %hold on;
% % figure(1);
% subplot(4,2,i)
% plot(frequency,power); 
% axis ([0 15 0 5]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% %title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')
% grid on
% end
% 
% figure (3);
% for i=1:8
% 
% % y1 = M(i,15872:17664);
% % y2 = M(i,27648:29440);
% y = M(i,39424:41216);
% % y4 = M(i,51200:52992);
% % y5 = M(i,62976:64768);
% % y = [y1,y2,y3,y4,y5];
% % y = M(i,18432:19968);
% L = length(y);  
% NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
% Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
% frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
% amplitude = 2*abs(Y1(1:NFFT/2+1));
% power = (abs(Y1(1:NFFT/2+1)).^2);
% 
% % Plot single-sided amplitude spectrum.
% %  figure(1);
% %  subplot(3,2,i)
% %  plot(frequency,amplitude); 
% %  axis ([0 12 0 3]);
% % %title('Single-Sided Amplitude Spectrum of y(t)')
% % title('14 Hz')
% % 
% % xlabel('Frequency (Hz)')
% % ylabel('Magnitude')
% 
% %hold on;
% % figure(1);
% subplot(4,2,i)
% plot(frequency,power); 
% axis ([0 15 0 5]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% %title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')
% grid on
% end
% 
% figure (4);
% for i=1:8
% 
% % y1 = M(i,15872:17664);
% % y2 = M(i,27648:29440);
% % y3 = M(i,39424:41216);
% y = M(i,51200:52992);
% % y5 = M(i,62976:64768);
% % y = [y1,y2,y3,y4,y5];
% % y = M(i,18432:19968);
% L = length(y);  
% NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
% Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
% frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
% amplitude = 2*abs(Y1(1:NFFT/2+1));
% power = (abs(Y1(1:NFFT/2+1)).^2);
% 
% % Plot single-sided amplitude spectrum.
% %  figure(1);
% %  subplot(3,2,i)
% %  plot(frequency,amplitude); 
% %  axis ([0 12 0 3]);
% % %title('Single-Sided Amplitude Spectrum of y(t)')
% % title('14 Hz')
% % 
% % xlabel('Frequency (Hz)')
% % ylabel('Magnitude')
% 
% %hold on;
% % figure(1);
% subplot(4,2,i)
% plot(frequency,power); 
% axis ([0 15 0 5]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% %title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')
% grid on
% end
% 
% figure (5);
% for i=1:8
% 
% % y1 = M(i,15872:17664);
% % y2 = M(i,27648:29440);
% % y3 = M(i,39424:41216);
% % y4 = M(i,51200:52992);
% y = M(i,62976:64768);
% % y = [y1,y2,y3,y4,y5];
% % y = M(i,18432:19968);
% L = length(y);  
% NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
% Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
% frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
% amplitude = 2*abs(Y1(1:NFFT/2+1));
% power = (abs(Y1(1:NFFT/2+1)).^2);
% 
% % Plot single-sided amplitude spectrum.
% %  figure(1);
% %  subplot(3,2,i)
% %  plot(frequency,amplitude); 
% %  axis ([0 12 0 3]);
% % %title('Single-Sided Amplitude Spectrum of y(t)')
% % title('14 Hz')
% % 
% % xlabel('Frequency (Hz)')
% % ylabel('Magnitude')
% 
% %hold on;
% % figure(1);
% subplot(4,2,i)
% plot(frequency,power); 
% axis ([0 15 0 5]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% %title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')
% grid on
% end
% 
% figure (6);
% for i=1:8
% 
% y1 = M(i,15872:17664);
% y2 = M(i,27648:29440);
% y3 = M(i,39424:41216);
% y4 = M(i,51200:52992);
% y5 = M(i,62976:64768);
% y = [y1,y2,y3,y4,y5];
% % y = M(i,18432:19968);
% L = length(y);  
% NFFT = 2^nextpow2(L);   % Number of points on the fft, Nfft= 16384
% Y1 = fft(y,NFFT)/L;    % cal fft and Discard Half of Points
% frequency = Fs/2*linspace(0,1,NFFT/2+1); % linspace(0,1,3);x as a vector of 3 linearly spaced values between 0 and 1
% amplitude = 2*abs(Y1(1:NFFT/2+1));
% power = (abs(Y1(1:NFFT/2+1)).^2);
% 
% % Plot single-sided amplitude spectrum.
% %  figure(1);
% %  subplot(3,2,i)
% %  plot(frequency,amplitude); 
% %  axis ([0 12 0 3]);
% % %title('Single-Sided Amplitude Spectrum of y(t)')
% % title('14 Hz')
% % 
% % xlabel('Frequency (Hz)')
% % ylabel('Magnitude')
% 
% %hold on;
% % figure(1);
% subplot(4,2,i)
% plot(frequency,power); 
% axis ([0 15 0 5]);
% %title('Single-Sided Amplitude Spectrum of y(t)')
% %title('14 Hz')
% 
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')
% grid on
% end























% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nsamps = length(y);
% Do Fourier Transform
% Y1= abs(fft(y));            %Retain Magnitude
% Y1 = Y1(1:L/2);               %Discard Half of Points
% f = Fs*(0:L/2-1)/L;           %Prepare freq data for plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%