s = serial('/dev/tty.usbserial-DA01LE47', 'BaudRate', 9600);
%s.Terminator = '}'; 
%x = serial('COM5', 'BaudRate', 9600, 'Terminator', 'CR', 'StopBit', 1, 'Parity', 'None');
fopen(s);
%s.Timeout = 50;
%fopen(x);
%fprintf(s,'testn');


%s = serial('/dev/cu.usbserial-DA01LE47', 'BaudRate', 9600);

flushinput(s);
flushoutput(s);
%fwrite(s,'101010101');
x = 1;
x = 0;
x = '';

count = 0;
while(1)
 
     % y = fscanf(s, 's', 30);
   % y = fgetl(s);
  %y =  fread(s,1,'char');
  
  %{
       commad = {1};
       xCord = {300};
       yCord = {3};
       zCord = {90};
       ChargingLevel = {100};
       
       
       
       replace_all = {'0'};
       x = jsonencode(table(commad, xCord, yCord, zCord, ChargingLevel));
       x = x(2:end-1);
       x = strcat(x, 'n')
       
       fwrite(s,x); 
  
  %}
  
  
 % senddata(1, 300, 3, 90, 100, s);
%       x = ''; 

 %      pause(10);

      y = fscanf(s);
       
       
  
   if(contains(y, '}') == 1) 
    %  if(y == '}')   
        z = find(y == '}');
              
        x = strcat(x,y(1:z(1)));
        
        if(x(1) ~= '{')
        x = strcat('{', x);
        end
        
        disp(x);

        
        y = jsondecode(x);
        
        
        disp(y);
        
        flushinput(s);
        flushoutput(s);
        a = find(x == ':');
        b = find(x == ',');
        z = strcat('ACK', x(a(1)+1:b(1)-1));
        z = strcat(z, 'n');
        
       disp(z);
       fwrite(s, z);
       
       
       %{ 
       commad = {'1'};
       waypoits = {2};
       replace_all = {'0'};
           
       x = jsonencode(table(commad,waypoits,replace_all));
       x = x(2:end-1);
       x = strcat(x, 'n')
       fwrite(s,x); 
       
   %}
       
       
      
       %disp(y(1:z(1)));
        x = '';

      end
end
%   else
%       x = strcat(x,y);
   
%   end
   
%    count = count + 1;
    
   
   %{
   if(y == '}') 
       x = strcat(x, y);
       disp(x);
       x = '';
       
       
   else
   
      x = strcat(x, y);
      

       
   end
   
   %}
   
   %{
   if(fscanf(s,'s',1) == '}') 
       
       disp(x);
       x = ''; 
   else 
      
       x = strcat(x, fscanf(s,'s',1));
       
   end
   %}
       
   
   
       
   %pause(2);
   

pattern = '}';
% x = fscanf(s, 's', 40);
% if(contains(x,pattern)) 
%    
%     y = find(x, pattern);
%     z = x(1:y(1));
%     p = strcat(p, z);
%     disp(p);
%     p = x(y(1)+1:end);
% else
%     p = strcat(p,x);
% end



%s.BytesAvailableFcnMode = 'byte'; 




%{
character = fscanf(s, 's', 1);
if(character == pattern) 
    disp(p);
    p = '';
    
    
else
    
    p = strcat(p, character);
 
end
%}
    
%pause(1);

%x = x+1;
   

fclose(s);