function testSail3
%test the short course code with many inputs
n=1;
m=1;
j=1;
k=1;
for i=1:25
    for m=1:9
        %sail3(j, k, n);
        j=j+2;
        k=k+2;
    end
end
sail3(3, 1, 1);
end