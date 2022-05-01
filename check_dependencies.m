% check for simulator on path
if isempty(which('simulator'))
    check_simulator = false ;
    warning(['The RTD simulator is not on your path! Please download it here: ',...
        'https://github.com/skousik/simulator'])
else
    disp('The simulator is on the MATLAB path')
    check_simulator = true ;
end

% check for RTD
try
    is_RTD_on_path()
    check_RTD = true ;
catch
    check_RTD = false ;
    warning(['The RTD parent repo is not on your path! Please download it here: ',...
        'https://github.com/skvaskov/RTD'])
end

% check for spotless
try
    x = msspoly('x') ;
    check_spotless = true ;
catch
    warning(['The spotless repo is not on your path! Please download ithere: ',...
        'https://github.com/spot-toolbox/spotless'])
    check_spotless = false ;
end

if check_simulator && check_RTD && check_spotless
    disp('RTD tutorial is ready to roll!')
else
    warning('RTD tutorial depencies are missing!')
end