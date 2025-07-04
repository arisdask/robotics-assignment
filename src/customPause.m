function customPause(nextStep, pauseFlag)
    % Conditional pause function with informative message
    %
    % Inputs:
    %   next_step  - String describing what will be executed next
    %   pause_flag - Numeric flag (0 = no pause, any other value = pause)
    
    % Input validation
    if nargin < 2
        error('customPause: Requires two arguments: next_step and pause_flag');
    end
    
    if ~ischar(nextStep) && ~isstring(nextStep)
        error('customPause: nextStep must be a string or character array');
    end
    
    if ~isnumeric(pauseFlag) || ~isscalar(pauseFlag)
        error('customPause: pauseFlag must be a numeric scalar');
    end
    
    % Only pause if pauseFlag is non-zero
    if pauseFlag ~= 0
        fprintf('\n--- PAUSE ---\n');
        fprintf('Next step: %s\n', nextStep);
        fprintf('Press any key to continue...\n');
        pause;
    end
end
