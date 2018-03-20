%Генерируем случайную величину по НОРМАЛЬНОМУ ЗРВ
function X = gererateRandom( range, count,classes, attribute)
%range - диапазон случайной величины
%count - количество наблюдений для iого класса
%classes - количество классов
%attribute - количество признаков 
%---------------------------------------------------------
%Нормальное распределение
%R = normrnd(MU,SIGMA,M,N) 
%возвращает матрицу M х N  чисел с параметрами 
%MU (математическое ожидание),SIGMA (дисперсия)
%---------------------------------------------------------
A=unidrnd(range,classes,1)
%R = unidrnd(N,MM,NN) возвращает матрицу MM х NN чисел из [1,N]
%MM - число строк 100
%NN число столбцов

for j=1:classes
    for i=((j-1)*count+1):count*j
        X(((j-1)*count+1):count*j,:)=normrnd(0,2,count,attribute)+A(j,1);%
    end
end
end
