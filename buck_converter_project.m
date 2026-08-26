%% DC-DC Buck Converter: Design and Closed-Loop PI Control
% 24 V -> 12 V, 60 W, 50 kHz
% Course Project (EE360)
%
% This script:
%   1) sizes the passive components,
%   2) simulates an averaged buck converter,
%   3) applies a closed-loop PI controller,
%   4) introduces input-voltage and load disturbances,
%   5) calculates transient/ripple metrics, and
%   6) saves publication-ready plots.

clear; close all; clc;

%% 1. Specifications
Vin_nom = 24;          % V
Vout_ref = 12;         % V
Pout_rated = 60;       % W
fs = 50e3;             % Hz
Ts = 1/fs;             % s
nsub = 1000;            % PWM resolution: 0.1% duty-cycle step
Rnom = Vout_ref^2/Pout_rated;
L = 100e-6;            % H
C = 470e-6;            % F
ESR = 0.04;             % ohm, assumed
D_nom = Vout_ref/Vin_nom;

% PI controller (chosen for stable averaged closed-loop response)
Kp = 0.01;
Ki = 20;

fprintf('--- BUCK CONVERTER DESIGN ---\n');
fprintf('Nominal load resistance = %.3f ohm\n', Rnom);
fprintf('Nominal duty ratio      = %.3f\n', D_nom);

%% 2. Theoretical ripple estimates
DeltaIL = (Vin_nom - Vout_ref)*D_nom/(L*fs);
DeltaV_C = DeltaIL/(8*C*fs);
DeltaV_ESR = DeltaIL*ESR;
DeltaVout_pp = DeltaV_C + DeltaV_ESR;

fprintf('Inductor ripple (p-p)   = %.3f A\n', DeltaIL);
fprintf('Capacitive ripple       = %.3f mV\n', 1e3*DeltaV_C);
fprintf('ESR ripple              = %.3f mV\n', 1e3*DeltaV_ESR);
fprintf('Estimated Vout ripple   = %.3f mV p-p\n', 1e3*DeltaVout_pp);
fprintf('PWM switching simulation uses %d integration steps/switching period (0.1%% duty resolution).\n', nsub);

%% 3. Closed-loop averaged simulation
Tsim = 0.10;                    % 100 ms
N = round(Tsim/Ts);
dt = Ts/nsub;

t = (0:N-1)'*Ts;
iL = zeros(N,1);
vout = zeros(N,1);
duty = zeros(N,1);
iin = zeros(N,1);

% Initial conditions
iL(1) = 0;
vout(1) = 0;
integral_e = 0;

for k = 1:N-1
    tk = t(k);

    % Disturbances:
    % 50 ms: input falls from 24 V to 20 V
    % 75 ms: load changes from 2.4 ohm to 1.8 ohm
    Vin = Vin_nom;
    Rload = Rnom;

    if tk >= 0.050
        Vin = 20;
    end
    if tk >= 0.075
        Rload = 1.8;
    end

    % PI controller sampled once per switching period
    e = Vout_ref - vout(k);
    integral_e = integral_e + e*Ts;
    duty(k) = min(max(Kp*e + Ki*integral_e, 0.05), 0.95);

    % Anti-windup correction when saturation occurs
    unsat = Kp*e + Ki*integral_e;
    if unsat ~= duty(k)
        integral_e = integral_e - 0.25*e*Ts;
    end

    % PWM-switched buck model integrated with a fine internal time step
    x_i = iL(k);
    x_v = vout(k);

    for m = 1:nsub
        % Explicit PWM switching state: sawtooth carrier in [0,1]
        carrier = (m-1)/nsub;
        sw = double(carrier < duty(k));

        % Ideal buck switch + freewheel diode CCM model
        di = (sw*Vin - x_v)/L;
        dv = (x_i - x_v/Rload)/C;

        x_i = x_i + di*dt;
        x_v = x_v + dv*dt;
        x_i = max(x_i, 0);  % CCM approximation for this design
        x_v = max(x_v, 0);

        if m == 1
            iin(k) = sw*x_i;
        else
            iin(k) = iin(k) + sw*x_i;
        end
    end
    iin(k) = iin(k)/nsub;

    iL(k+1) = x_i;
    vout(k+1) = x_v;
end
duty(end) = duty(end-1);
iin(end) = duty(end)*iL(end);

%% 4. Performance metrics
idx_ss = t >= 0.090;
Vss = mean(vout(idx_ss));
Iss = mean(iL(idx_ss));
Pout = Vss^2/1.8;
% Average input power is estimated from the actual input-voltage
% disturbance profile rather than assuming 24 V throughout.
Vin_trace = Vin_nom*ones(size(t));
Vin_trace(t >= 0.050) = 20;
Pin_ideal = mean(Vin_trace(idx_ss).*iin(idx_ss));
eta_ideal = 100*Pout/max(Pin_ideal, eps);

% 10-90% rise time around initial startup
v10 = 0.1*Vout_ref;
v90 = 0.9*Vout_ref;
i10 = find(vout >= v10,1,'first');
i90 = find(vout >= v90,1,'first');
if ~isempty(i10) && ~isempty(i90)
    rise_time_ms = 1e3*(t(i90)-t(i10));
else
    rise_time_ms = NaN;
end

overshoot = max(0, (max(vout)-Vout_ref)/Vout_ref*100);
settle_band = 0.02*Vout_ref;
last_outside = find(abs(vout-Vout_ref)>settle_band,1,'last');
if isempty(last_outside)
    settling_ms = 0;
else
    settling_ms = 1e3*t(last_outside);
end

fprintf('\n--- CLOSED-LOOP RESULTS ---\n');
fprintf('Final output voltage    = %.3f V\n', vout(end));
fprintf('Final inductor current  = %.3f A\n', iL(end));
fprintf('Rise time (10-90%%)      = %.3f ms\n', rise_time_ms);
fprintf('Peak overshoot          = %.2f %%\n', overshoot);
fprintf('2%% settling time        = %.3f ms\n', settling_ms);
fprintf('Final duty ratio        = %.3f\n', duty(end));
fprintf('Output power after load step = %.2f W\n', Pout);
fprintf('Ideal-input efficiency indicator = %.2f %%\n', eta_ideal);

%% 5. Plots
figdir = fullfile(pwd,'figures');
if ~exist(figdir,'dir'), mkdir(figdir); end

f1 = figure('Color','w');
plot(t*1e3,vout,'LineWidth',1.5); hold on;
yline(Vout_ref,'--','Reference');
xline(50,'--','V_{in} step');
xline(75,'--','Load step');
grid on; xlabel('Time (ms)'); ylabel('Output Voltage (V)');
title('Closed-Loop Output Voltage');
legend('V_{out}','V_{ref}','Location','best');
saveas(f1,fullfile(figdir,'output_voltage.png'));

f2 = figure('Color','w');
plot(t*1e3,iL,'LineWidth',1.5);
grid on; xlabel('Time (ms)'); ylabel('Inductor Current (A)');
title('Inductor Current Response');
saveas(f2,fullfile(figdir,'inductor_current.png'));

f3 = figure('Color','w');
plot(t*1e3,duty,'LineWidth',1.5);
grid on; xlabel('Time (ms)'); ylabel('Duty Ratio');
title('PI Controller Duty-Ratio Command');
saveas(f3,fullfile(figdir,'duty_cycle.png'));

f4 = figure('Color','w');
plot(t*1e3,vout,'LineWidth',1.2); xlim([45 85]);
grid on; xlabel('Time (ms)'); ylabel('Output Voltage (V)');
title('Transient Response to Input and Load Changes');
saveas(f4,fullfile(figdir,'transient_response.png'));

fprintf('\nFigures saved to: %s\n',figdir);

