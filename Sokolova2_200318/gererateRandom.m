%���������� ��������� �������� �� ����������� ���
function X = gererateRandom( range, count,classes, attribute)
%range - �������� ��������� ��������
%count - ���������� ���������� ��� i��� ������
%classes - ���������� �������
%attribute - ���������� ��������� 
%---------------------------------------------------------
%���������� �������������
%R = normrnd(MU,SIGMA,M,N) 
%���������� ������� M � N  ����� � ����������� 
%MU (�������������� ��������),SIGMA (���������)
%---------------------------------------------------------
A=unidrnd(range,classes,1)
%R = unidrnd(N,MM,NN) ���������� ������� MM � NN ����� �� [1,N]
%MM - ����� ����� 100
%NN ����� ��������

for j=1:classes
    for i=((j-1)*count+1):count*j
        X(((j-1)*count+1):count*j,:)=normrnd(0,2,count,attribute)+A(j,1);%
    end
end
end
