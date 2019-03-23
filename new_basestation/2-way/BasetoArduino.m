s = serial('/dev/tty.usbserial-DA01LE47', 'BaudRate', 9600);

fopen(s);

flushinput(s);
flushoutput(s);

x = 1;
x = 0;
x = '';

count = 0;
 

  
  count = count + 1;
  senddata(count, 300, 3, 90, 100, s);
       x = ''; 


while(1)
  
  y = fscanf(s);
  disp(y)
  pattern = "ACK";
  
  if(contains(y, pattern) == 1) 
   
       count = count + 1;
       
       senddata(count, 300, 3, 90, 100, s);
       
       flushinput(s);
       flushoutput(s);
       

  else
  
      %pause(3);
      senddata(count, 300, 3, 90, 100, s);
      flushinput(s);
      flushoutput(s);

      
  end
  
  
end


   

fclose(s);