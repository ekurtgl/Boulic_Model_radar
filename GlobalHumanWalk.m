%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Global Human Walk:
%
% Based on “A Global Human Walking Model with Real-time Kinematic
% Personification,” by R. Boulic, N. M. Thalmann, and D. Thalmann
% The Visual Computer, Vol.6, 1990, pp. 344–358.
% This model is based on biomechanical experimental data.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all
% human walking model
showplots = 'n'; % show translation and rotation of body segments
formove = 'n'; % forward walking
animove = 'y'; % display animation
genmovie = 'y'; % generate movie file
% relative velocity defined by average walking velocity normalized by the
% height from the toe to the hip: Ht
rv = 1.0; % relative velocity (from 0 to 3)
nt = 2048; % number of frames per cycle
if mod(nt,2) == 1
nt = nt+1;
end
numcyc = 3; % number of cycle
Height = 1.8;
[segment,seglength,T] = HumanWalkingModel(showplots, formove,...
animove, genmovie, Height, rv, nt, numcyc);
data = RadarReturnsFromWalkingHuman(segment,seglength);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







