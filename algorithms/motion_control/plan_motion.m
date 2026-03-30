function [public_vars] = plan_motion(read_only_vars, public_vars)
    public_vars.motion_vector = [0, 0];
    
    %     % PLAN_MOTION - Task 5 
%     c = read_only_vars.counter;
% 
%     if c < 85
%         %rovně
%         public_vars.motion_vector = [0.8, 0.805]; 
% 
%     elseif c < 105
%         % doprava
%         public_vars.motion_vector = [0.2, 0.36]; 
% 
%     elseif c < 140  
%         % rovně
%         public_vars.motion_vector = [0.8, 0.8];
% 
%     elseif c < 160 
%          % doprava
%         public_vars.motion_vector = [0.2, 0.32];
% 
%      elseif c < 230  
%         % rovně
%          public_vars.motion_vector = [0.8, 0.8];
% 
%      elseif c < 250  
%         % doleva
%          public_vars.motion_vector = [0.32, 0.2];
% 
%      elseif c < 260  
%         % rovně
%          public_vars.motion_vector = [0.8, 0.8];
% 
%      elseif c < 283  
%         % doleva
%          public_vars.motion_vector = [0.32, 0.2];
% 
%       elseif c < 310  
%         % ROVNĚEEEEE
%          public_vars.motion_vector = [0.8, 0.8];
% 
%     elseif c < 820  
%         % ROVNĚEEEEE 2
%         public_vars.motion_vector = [0.8, 0.8];
% 
%     else
%         % brzdi ?
%         public_vars.motion_vector = [0, 0];
%     end
% end
end

