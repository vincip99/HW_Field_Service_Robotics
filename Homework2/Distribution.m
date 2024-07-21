clear all
close all
clc

%% Exercise 1
% Symbolic variables
syms x [3 1]
assume(x, 'real');

%% vector fields, involutive distribution and annihilator 
G1 = [3*x(1); 0; -1];
% involutive or not
[r1, inv1, F1] = involutive(G1,x);
% Annihillator
null(G1')'

g1 = [1; 0; x(2)]; g2 = [0; 0; x(1)];
G2 = [g1, g2];
% involutive or not
[r2, inv2, F2] = involutive(G2,x);
% Annihillator
null(G2')'

g1 = [2*x(3); -1; 0]; g2 = [x(2); x(1); 1];
G3 = [g1, g2];
% involutive or not
[r3, inv3, F3] = involutive(G3,x);
% Annihillator
null(G3')'


%% Exercise 2
%% Kinematic model
% Omnidirectional mobile robot kinematic variables
syms l r
syms x y theta alpha beta gamma

assume(l, 'real');
assume(r, 'real');
assume(x, 'real');
assume(y, 'real');
assume(theta, 'real');
assume(alpha, 'real');
assume(beta, 'real');
assume(gamma, 'real');

% Pfaffian matrix
A = [sqrt(3)/2 * cos(theta) - 0.5 * sin(theta), 0.5 * cos(theta) + sqrt(3)/2 * sin(theta), l, r, 0, 0;
    sin(theta), -cos(theta), l, 0, r, 0;
    -sqrt(3)/2 * cos(theta) - 0.5 * sin(theta), 0.5 * cos(theta) - sqrt(3)/2 * sin(theta), l, 0, 0, r];

% Computing the null vectors of the pfaffian matrix
G = simplify(null(A));

% Computing the lie brackets for each column vector in G
h1 = simplify(lieBracket(G(:,1),G(:,2),[x y theta alpha beta gamma]));

h2 = simplify(lieBracket(G(:,2),G(:,3),[x y theta alpha beta gamma]));

h3 = simplify(lieBracket(G(:,1),G(:,3),[x y theta alpha beta gamma]));

% Computing the dimension of the distribution
delta = [G, h1, h2, h3];
rank(delta)


%% LIE BRACKET functions
% Function that compute the lie bracket between two vector fields
function h = lieBracket(f,g,x)
    
    % Jacobian of the two vector fields respect to the symbolic variables
    Jg = jacobian(g,x);
    Jf = jacobian(f,x);
    
    % Output vector field 
    h = Jg * f - Jf * g;
    
end

function [r, invol, F] = involutive(G,x)
    
    % if there is only one vector in the distribution lie bracket is 0
    if size(G,2) == 1
        B(:,1) = zeros(size(G));
    else % if there are more vector fields in the distribution
        % Loops to generate n*(n-1)/2 lie brackets (/2 due to the skew-symmetry)
        for i = 1 : size(G,2)
            % compute the lie bracket between the i-th vector and every j>i vector 
            for j = i+1 : size(G,2)
                % Lie bracket matrix
                B(:,i) = lieBracket(G(:,i), G(:,j), x);
            end
        end
    end
        
    % Matrix with vector fields and lie brackets
    F = [G, B];
    rg = rank(G);
    
    % Involutive 
    r = rank(F);
    
    % if F contains linearly dependemt vectors is involutive
    if r == rg % rank equal to the rank of the distribution
        invol = true;
    else
        invol = false;
    end

end



