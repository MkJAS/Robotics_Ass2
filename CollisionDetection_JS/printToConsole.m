function  printToConsole(message)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
switch message
    case 1
        cprintf('red', 'Possible collision detected! Aborting move!\n');
        beep
        cprintf('red', 'Waiting for obstacle to be removed\n');
    case 2
        cprintf('red', 'Workspace Breached! Waiting for clear!\n');
        beep
end