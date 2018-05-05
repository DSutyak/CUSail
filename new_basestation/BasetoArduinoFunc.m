function [] = BasetoArduinoFunc( s, xCord, yCord, zCord, Charge)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

fopen(s);

x = '';

count = 0;
count = count + 1;
senddata(count, xCord, yCord, zCord, Charge, s); % send the data for input parameters

while(1)

    
  y = fscanf(s); % check the serial port
  disp(y) % display the values of the serial port
  pattern = "ACK"; 
  
  if(contains(y, pattern) == 1) % if we got an acknowledge signal from Arduino 
   
       count = count + 1;
       
       senddata(count, xCord, yCord, zCord, Charge, s); % send the next one
       

  else
  
      senddata(count, xCord, yCord, zCord, Charge, s); % send the previous one

      
  end
  
  

end

end


  