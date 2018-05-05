function [] = senddata(a, b, c, d, e, s )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%while(1)
 
     % y = fscanf(s, 's', 30);
   % y = fgetl(s);
  %y =  fread(s,1,'char');
  
       commad = {a};
       xCord = {b};
       yCord = {c};
       zCord = {d};
       Charge = {e};
       
       
       
       replace_all = {'0'};
       x = jsonencode(table(commad, xCord, yCord, zCord, Charge)); % encode the parameters as JSON
       x = x(2:end-1); % send over the string
       
       fwrite(s,x); % write the string to the serial port


%end

end