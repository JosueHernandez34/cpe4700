% lab04_analysis.m -- Lab 04 post-lab analysis in MATLAB (both wheels).
%
% Copy deadband.csv, step_response.csv and speed_control.csv from the Jetson
% into the folder that holds this file (or cd to them), then run:
%     lab04_analysis
% Results land in the struct `lab04` and in lab04.mat. The CSVs are not
% submitted -- only your plots and report.

clear; clc;
KP = 0.005;  KI = 0.020;           % gains currently in modules/speed_control.c

%% Deadband (offsets) ----------------------------------------------------
db = readtable('deadband.csv');
dbpos = nan(1, 2);  dbneg = nan(1, 2);
for w = 0:1
    p = db.deadband_duty(db.motor == w & strcmp(db.direction, 'positive'));
    n = db.deadband_duty(db.motor == w & strcmp(db.direction, 'negative'));
    if ~isempty(p), dbpos(w+1) = p(1);      end
    if ~isempty(n), dbneg(w+1) = abs(n(1)); end   % controller wants a magnitude
end
fprintf('Deadband (duty)   wheel0   wheel1\n');
fprintf('  positive        %.2f     %.2f\n', dbpos);
fprintf('  negative        %.2f     %.2f\n', dbneg);
fprintf('\nPaste into modules/speed_control.c:\n');
fprintf('static const double DEADBAND_POS[NUM_MOTORS] = { %.2f, %.2f };\n', dbpos);
fprintf('static const double DEADBAND_NEG[NUM_MOTORS] = { %.2f, %.2f };\n\n', dbneg);

%% Step response ---------------------------------------------------------
st = readtable('step_response.csv');
t  = st.time_ms / 1000;

figure('Name', 'Step response');
subplot(3,1,1); plot(t, st.duty_cycle); ylabel('duty cycle'); grid on;
title('Step response, both wheels');
subplot(3,1,2); plot(t, st.speed_0, t, st.speed_1);
ylabel('speed (edges/s)'); legend('wheel 0', 'wheel 1'); grid on;
subplot(3,1,3); plot(t, st.position_0, t, st.position_1);
ylabel('position (counts)'); xlabel('time (s)');
legend('wheel 0', 'wheel 1'); grid on;

% Windows from the Part 6 sequence: forward 1-2 s, reverse 4-5 s, forward 5-6 s.
windows = {'forward', 1.0, 2.0, +0.60; 'reverse', 4.0, 5.0, -0.60; ...
           'forward2', 5.0, 6.0, +0.60};
lab04.step = struct();
for w = 0:1
    speed = st.(sprintf('speed_%d', w));
    for k = 1:size(windows, 1)
        m  = t >= windows{k,2} & t < windows{k,3};
        ts = t(m);  s = speed(m);  duty = windows{k,4};
        ss = mean(s(end-99:end));           % last 100 ms = steady state
        sn = s * sign(ss);  mag = abs(ss);  % flip to direction of travel
        i10 = find(sn >= 0.1*mag, 1);  i90 = find(sn >= 0.9*mag, 1);
        out = find(abs(sn - mag) > 0.02*mag, 1, 'last');
        if isempty(out), settle = 0; else, settle = ts(out) - windows{k,2}; end
        r = struct('steady_state', ss, 'rise_s', ts(i90) - ts(i10), ...
                   'settle_s', settle, 'gain', ss / duty);
        lab04.step.(sprintf('wheel%d', w)).(windows{k,1}) = r;
        fprintf('wheel %d %-8s  ss=%8.1f  rise=%.3f s  settle=%.3f s  gain=%.1f\n', ...
                w, windows{k,1}, r.steady_state, r.rise_s, r.settle_s, r.gain);
    end
    rest = t < 1.0;  steady = t >= 1.5 & t < 2.0;
    lab04.noise.(sprintf('wheel%d', w)) = struct( ...
        'std_steady_fwd', std(speed(steady)), 'std_at_rest', std(speed(rest)));
    fprintf('wheel %d encoder noise: steady-forward std = %.2f, at-rest std = %.2f\n', ...
            w, std(speed(steady)), std(speed(rest)));
end

%% Triangular speed control ----------------------------------------------
sc = readtable('speed_control.csv');
figure('Name', 'Speed control');
for w = 0:1
    meas = sc.(sprintf('measured_speed_%d', w));
    subplot(2,1,w+1);
    plot(sc.time_ms/1000, sc.target_speed, sc.time_ms/1000, meas);
    ylabel(sprintf('wheel %d speed', w)); legend('target', 'measured'); grid on;
    rms = sqrt(mean((sc.target_speed - meas).^2));
    lab04.speed_control.(sprintf('wheel%d_rms_error', w)) = rms;
    fprintf('wheel %d RMS tracking error: %.2f\n', w, rms);
end
subplot(2,1,1); title('Triangular speed reference, 3 cycles');
subplot(2,1,2); xlabel('time (s)');
fprintf('cycles: %.2f\n', height(sc) / 2000);

%% Save ------------------------------------------------------------------
lab04.deadband = struct('positive', dbpos, 'negative', dbneg);
lab04.gains = struct('KP', KP, 'KI', KI);
save('lab04.mat', 'lab04', 'st', 'sc', 'db');
