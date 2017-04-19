function Cnb = a2mat(att)
% Convert Euler angles to direction cosine matrix(DCM).
%
% Prototype: Cnb = a2mat(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Output: Cnb - DCM from body-frame to navigation-frame
%
% 2017/04/06 by Ren

   
    si = sin(att.e); sj = sin(att.n); sk = sin(att.u); 
    ci = cos(att.e); cj = cos(att.n); ck = cos(att.u);
%     Cnb = [ cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk;
%             cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck;
%            -ci*sj,           si,     ci*cj           ];% original rotation
%     matrix
    Cnb = [ cj*ck+si*sj*sk, ci*sk,  sj*ck-si*cj*sk;
            -cj*sk+si*sj*ck,  ci*ck,  -sj*sk-si*cj*ck;
           -ci*sj,           si,     ci*cj           ];%17/04/11 rotation matrix refer to Qin's "inertial navigation" 