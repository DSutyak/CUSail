s = serial('COM6', 'BaudRate', 9600);
%x = serial('COM5', 'BaudRate', 9600, 'Terminator', 'CR', 'StopBit', 1, 'Parity', 'None');
fopen(s);
%fopen(x);
%fprintf(s,'testn');





%while (1)
    disp(s.BytesAvailable);
    s.BytesAvailableFcnMode = 'terminator';
    s.Terminator = '}';
    %set(s, 'BytesAvailableFcn', @onTerminatorChar);
    %s.BytesAvailableFcn = @onTerminatorChar;
    s.BytesAvailableFcn = @(~,~)onTerminatorChar(s);

    %disp(1);
%end

function onTerminatorChar()
    disp("EEEEEEEEEEEE");
    msg = fscanf(s, '%s', s.BytesAvailable);
    disp(msg);
end


