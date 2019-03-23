s = serial('COM6', 'BaudRate', 9600);
%x = serial('COM5', 'BaudRate', 9600, 'Terminator', 'CR', 'StopBit', 1, 'Parity', 'None');
fopen(s);
%fopen(x);
%fprintf(s,'testn');


%s = serial('/dev/cu.usbserial-DA01LE47', 'BaudRate', 9600);
    s.BytesAvailableFcnMode = 'terminator';
    s.Terminator = '}';

%fwrite(s,'101010101');
%x = 1;
while(1)
     
    while(s.BytesAvailable == 0)
         x = s.BytesAvailable;s
        disp('hello world');
    end
disp(x)
fwrite(s,'1010n');%send data
end


%pause(5);
disp(fscanf(s, '%s', 10));
%pause(1);
%fprintf(s,'cusailn');

%x = x+1;
%end

fclose(s);