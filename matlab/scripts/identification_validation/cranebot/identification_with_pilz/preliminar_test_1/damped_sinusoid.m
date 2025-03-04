function y = damped_sinusoid(b, T, t)
% Evaluates a exponentially damped sinusoid at the values in x
% with damping factor b, period T, amplitude a, and phase phi.
phi = pi/2;
a = 0.081;

y = a .* exp(-b .* t) .* sin(2 * pi / T .* t + phi);

end