function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 16-Mar-2018 14:36:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global range;
global count;
global classes;
global attribute;
global X;
global Y;%классифицирующая функция
global cv; %для разделения на множества Test и Train
global Xtrain;
global Ytrain;
global Xtest;
global Ytest;



 range      = str2num(get(handles.edit1,'String'))
 count      = str2num(get(handles.edit2,'String'))
 classes    = str2num(get(handles.edit3,'String'))
 attribute  = str2num(get(handles.edit4,'String'))
%----------------------------------------------------------
%Формирование матрицы исходных данных
X=gererateRandom(range,count,classes,attribute) 
X=normr(X);
%----------------------------------------------------------
%классифицирующая матрица
for j=1:classes
    for i=((j-1)*count+1):count*j
        Y(((j-1)*count+1):count*j,1)=j;%
    end
end
%----------------------------------------------------------
%рисуем график исходных данных
axes(handles.axes1)%Сделать оси текущими
plot3(X(:,1),X(:,2),X(:,3),'.')%Рисование графика
%----------------------------------------------------------
% В примере 40% данных случайным образом включаются в тестовый набор
cv = cvpartition((classes*count),'holdout',0.40)
%создает случайное разделение n наблюдений на обучающую выборку и тестирующую выборку. 
%Параметр p должен быть 0 < p < 1, в этом случае функция cvpartition 
%случайно выбирает примерно p*n наблюдений для тестирующей выборки.

%cv = 
%Hold-out cross validation partition
%             N: 300
%   NumTestSets: 1
%     TrainSize: 180
%      TestSize: 120

%----------------------------------------------------------
% Обучающее множество
%training(c) – возвращает логический вектор  
%для отличия номеров обучающей выборки (1) от номеров тестирующей выборки (0).
%X(training(cv),:) формирует подматрицу X из строк обучающей выборки.
Xtrain = X(training(cv),:)
Ytrain = Y(training(cv),:)
%----------------------------------------------------------
% ТЕСТОВОЕ МНОЖЕСТВО
%test(c) – возвращает логический вектор   для отличия номеров 
%тестирующей выборки (1) от номеров обучающей выборки (0).
Xtest = X(test(cv),:)
Ytest = Y(test(cv),:)
%----------------------------------------------------------
%ПОСТРОЕНИЕ ТАБЛИЦЫ ЧАСТОТ
%tabulate(x) создает таблицу частот данных из вектора x. 
%Информация в таблице организована следующим образом:
%1 столбец — уникальное значение x
%2 столбец — число повторений каждого значения
%3 столбец — процент каждого значения
tabulate(Ytrain)
tabulate(Ytest)
%--------------------------------------------------------------------------



% --- Executes on key press with focus on pushbutton1 and none of its controls.
function pushbutton1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on pushbutton2 and none of its controls.
function pushbutton2_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

%==========================================================================
% --- Executes on button press in SaveButton.
function SaveButton_Callback(hObject, eventdata, handles)
% hObject    handle to SaveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%СОХРАНЕНИЕ ДЕРЕВА В ФАЙЛ 
global X;
global Y;
global tree1;
save optimalTree.mat tree1;
save X.mat X Y;



% --- Executes on key press with focus on SaveButton and none of its controls.
function SaveButton_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to SaveButton (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

%==========================================================================
% --- Executes on button press in LoadButton.
function LoadButton_Callback(hObject, eventdata, handles)
% hObject    handle to LoadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global tree1;
global X;
global Y;
global Xtest;
global Ytest;
global Ytrain;
global Xtrain;
global cv;
global count;
global classes;
global YnewTrain;%предсказания по обучающей выборке
global YnewTest;%предсказания по тестовой выборке
global Ctrain;%количество правильно предсказанных
global Ctest;%количество правильно предсказанных
global C_ts;%процент правильно предсказанных 
global C_tr%процент правильно предсказанных

load optimalTree.mat tree1;
load X.mat X Y;
%----------------------------------------------------------
%рисуем график исходных данных
axes(handles.axes1)%Сделать оси текущими
plot3(X(:,1),X(:,2),X(:,3),'.')%Рисование графика
%----------------------------------------------------------
%ВЫВОД ДЕРЕВА ОПТИМАЛЬНОГО УРОВНЯ НА ЭКРАН
view(tree1,'mode','graph');
%----------------------------------------------------------
% ОБУЧАЮЩЕЕ МНОЖЕСТВО
%training(c) – возвращает логический вектор  
%для отличия номеров обучающей выборки (1) от номеров тестирующей выборки (0).
%X(training(cv),:) формирует подматрицу X из строк обучающей выборки.
%----------------------------------------------------------
% В примере 40% данных случайным образом включаются в тестовый набор
count      = str2num(get(handles.edit2,'String'))
classes    = str2num(get(handles.edit3,'String'))
cv         = cvpartition((classes*count),'holdout',0.40)

Xtrain = X(training(cv),:)
Ytrain = Y(training(cv),:)
%----------------------------------------------------------
% ТЕСТОВОЕ МНОЖЕСТВО
%test(c) – возвращает логический вектор   для отличия номеров 
%тестирующей выборки (1) от номеров обучающей выборки (0).
Xtest = X(test(cv),:)
Ytest = Y(test(cv),:)
%--------------------------------------------------------------------------
%ПРЕДСКАЗАНИЕ ЗНАЧЕНИЙ КЛАССИФИЦИРУЮЩЕГО ПРИЗНАКА ПО ДЕРЕВУ
%Для каждой стоки матрицы Xnew выдается классифицирующий признак в
%соответсвии с деревом, все значения объединяются в вектор Ynew
%Ynew=predict(tree,Xnew)
%Ynew = predict(tree,X)
YnewTrain = predict(tree1,Xtrain)%предсказания по обучающей
YnewTest = predict(tree1,Xtest)%предстаказания по тестовой
% количество верно классифицированных – дерево решений
Ctrain = confusionmat(Ytrain,YnewTrain);
Ctest = confusionmat(Ytest,YnewTest);
%Выводим в процентах
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100

function LoadButton_KeyPressFcn(hObject, eventdata, handles)


function ButtonTreeFit_Callback(hObject, eventdata, handles)
global X;
global Y; 
global BinarTree;%бинарное дерево
global Ynew;
global tree1;%оптимальное дерево
%Расчет бинарного дерева классификации наблюдений 
BinarTree = treefit(X,Y);
%отображение бинарного дерева
treedisp(BinarTree);
%----------------------------------------------------------





%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

function ButtonTreeFit_KeyPressFcn(hObject, eventdata, handles)

%--------------------------------------------------------------------------
function FitcTreeButton_Callback(hObject, eventdata, handles)
global tree;
global Ynew;
global tree1;

global Xtrain;
global Ytrain;

global Xtest;
global Ytest;

global YnewTrain;%предсказания по обучающей выборке
global YnewTest;%предсказания по тестовой выборке

global Ctrain;%количество правильно предсказанных
global Ctest;%количество правильно предсказанных

global C_ts;%процент правильно предсказанных 
global C_tr%процент правильно предсказанных

global X;
%ФОРМИРОВАНИЕ ДЕРЕВА КЛАССИФИКАЦИЙ
%tree = fitctree(x,y) возвращает бинарное дерево классификации tree, 
%основанное на входных переменных x и выходной переменной y. 
%В бинарном дереве каждый узел имеет не более двух потомков по значениям
%колонки матрицы x.
tree = fitctree(Xtrain,Ytrain)
%----------------------------------------------------------
%ВЫВОД ДЕРЕВА КЛАССИФИКАЦИЙ НА ЭКРАН
view(tree,'mode','graph');
%----------------------------------------------------------
%НАХОЖДЕНИЕ ОПТИМАЛЬНОГО УРОВНЯ СОКРАЩЕНИЯ ДЕРЕВА ЗА СЧЕТ МИНИМИЗАЦИИ
%ПОТЕРЬ
%Функция возвращает наилучший уровень сокращния дерева,
%все остальные выходные параметры не используются
[~,~,~, bestlevel] = cvLoss(tree,'SubTrees','All','TreeSize','min')
%-----------------------------------------------------------
%СОКРАЩЕНИЕ ДЕРЕВА ДО ОПТИМАЛЬНОГО УРОВНЯ
%получение сокращенного дерева
tree1 = prune(tree,'Level',bestlevel);
%----------------------------------------------------------
%ВЫВОД ДЕРЕВА ОПТИМАЛЬНОГО УРОВНЯ НА ЭКРАН
view(tree1,'mode','graph');
%----------------------------------------------------------
%ПРЕДСКАЗАНИЕ ЗНАЧЕНИЙ КЛАССИФИЦИРУЮЩЕГО ПРИЗНАКА ПО ДЕРЕВУ
%Для каждой стоки матрицы Xnew выдается классифицирующий признак в
%соответсвии с деревом, все значения объединяются в вектор Ynew
%Ynew=predict(tree,Xnew)
%Ynew = predict(tree,X)
YnewTrain = predict(tree1,Xtrain)%предсказания по обучающей
YnewTest  = predict(tree1,Xtest)%предстаказания по тестовой
% количество верно классифицированных – дерево решений
Ctrain = confusionmat(Ytrain,YnewTrain);
Ctest = confusionmat(Ytest,YnewTest);
%Выводим в процентах
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100


function FitcTreeButton_KeyPressFcn(hObject, eventdata, handles)


% --- Executes on button press in ButtonFizzyLogic.
function ButtonFizzyLogic_Callback(hObject, eventdata, handles)
%==========================================================================
%СИСТЕМА НЕЧЕТКОГО ВЫВОДА
%В этой системе мы не можем однозначно ответить принадлежит тот или иной
%элемент рассматриваемому нечеткому множеству
%задается функция, определяющая степень уверенности положительного ответа
%на подобный вопрос. Область допустимых значений этой функции от 0 до 1
%ЛИНГВИСТИЧЕСКАЯ ПЕРЕМЕННАЯ - переменная, значениями которой могут быть
%слова
%ТЕРМ МНОЖЕСТВО - множетсво всех возможных значений  лингвистической
%переменной 
%ТЕРМ - любой элемент терм множества
%--------------------------------------------------------------------------
%ФУНКЦИИ ПРИНАДЛЕЖНОСТИ
%1.Кусочно - линейные 
    %1.1. Треугольная
    %1.2. Трапециевидная
%2.Z образные и S образные
%--------------------------------------------------------------------------
%НЕЧЕТКИЕ ЛОГИЧЕСКИЕ ОПЕРАЦИИ
%A И B  =min(P(A),P(B))
%A ИЛИ B=max(P(A),P(B))
%НЕ A= 1-P(A)
%ИМПЛИКАЦИЯ A->B= max (1-P(A),P(B))
%--------------------------------------------------------------------------
%ФОРМАТ  НЕЧЕТКИХ ПРАВИЛ
%ЕСЛИ  посылка правила ТО заключение правила
% где посылка и заключение - это термы нечетких переменных
%--------------------------------------------------------------------------
%ФАЗИФИКАЦИЯ
% установка соответствия между значением входной переенной и значением
% функции принадлежности  соответсвуюшего ей терма. Это выбор функции
% нечеткой логики
%--------------------------------------------------------------------------
%АГРЕГИРОВАНИЕ
%процедура определения степени истинности высказываний. Если заключение 
%нечеткого продуктивного правила является  простым нечетким высказыванием, 
% то степень его истинности равна АЛГЕБРАИЧЕСКОМУ ПРОИЗВЕДЕНИЮ ВЕСОВОГО
% КОЭФФИЦИЕНТА И СТЕПЕНИ ИСТИННОСТИ ПОСЫЛКИ НЕЧЕТКОГО ПРАВИЛА 
%--------------------------------------------------------------------------
%СОЗДАНИЕ НОВОЙ СИСТЕМЫ НЕЧЕТКОГО ЛОГИЧЕСКОГО ВЫВОДА
global fis_name;
global fis_type;
global fisA;

fis_name='new_fuzzy_system';%имя системы нечеткого вывода
fis_type='sugeno';%тип системы нечеткого вывода
    %Допустимые значения:
    %'mamdani' - система типа Мамдани
    %'sugeno' - система типа Сугено 
fisA = newfis(fis_name,fis_type) 
%Добавление переменной в систему нечеткого логического вывода

fisA=addvar(fisA,'input', 'x1',[0 1])
fisA=addvar(fisA,'input', 'x2',[0 1])
fisA=addvar(fisA,'input', 'x3',[0 1])
fisA=addvar(fisA,'output', 'class',[1 3])
%--------------------------------------------------------------------------
%Добавление функции принадлежности к системе нечеткого вывода	
fisA=addmf(fisA, 'input', 1, 'L', 'trimf', [0 0.2760585 0.552117])
fisA=addmf(fisA, 'input', 1, 'H', 'trimf', [0.552117 0.7760585 1])

fisA=addmf(fisA, 'input', 2, 'L', 'trimf', [0 0.2690865 0.538173])
fisA=addmf(fisA, 'input', 2, 'Average', 'trimf', [0.538173 0.5457505 0.553328])
fisA=addmf(fisA, 'input', 2, 'H', 'trimf', [0.553328 0.776664 1])

fisA=addmf(fisA, 'input', 3, 'L', 'trimf', [0 0.2729365 0.545873])
fisA=addmf(fisA, 'input', 3, 'H', 'trimf', [0.545873 0.7729365 1])

fisA=addmf(fisA, 'output', 1, 'A', 'constant', 1)
fisA=addmf(fisA, 'output', 1, 'B', 'constant', 2)
fisA=addmf(fisA, 'output', 1, 'C', 'constant', 3)
%--------------------------------------------------------------------------
%Правила
r1='if x1 is L then class is C                              ';
r2='if x1 is H and x3 is L then class is C                  ';
r3='if x1 is H and x3 is H and x2 is L then class is C      ';
r4='if x1 is H and x3 is L and x2 is Average then class is A';
r5='if x1 is H and x3 is L and x2 is H then class is B      ';
inrulelist=char(r1,r2,r3,r4,r5)
fisA = parsrule (fisA, inrulelist) %ввод правил в нечеткую базу знаний %char(rule1,rule2,rule3);
%--------------------------------------------------------------------------
%Запись системы нечеткого логического вывода
writefis(fisA,'myfis.fis');
%--------------------------------------------------------------------------   
%Загрузка системы нечеткого логического вывода 
fisA=readfis('myfis.fis')
%--------------------------------------------------------------------------
%Выполнение нечеткого логического вывода
output=evalfis([0.7892 0.5482 0.7169],fisA) %class 1
%--------------------------------------------------------------------------
plotfis(fisA)

% --- Executes on key press with focus on ButtonFizzyLogic and none of its controls.
function ButtonFizzyLogic_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to ButtonFizzyLogic (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in ButtonShowLoadingTree.
function ButtonShowLoadingTree_Callback(hObject, eventdata, handles)
%ВЫВОД ДЕРЕВА КЛАССИФИКАЦИЙ НА ЭКРАН ПОСЛЕ ЕГО ЗАГРУЗКИ
global tree1;
view(tree1,'mode','graph');


% --- Executes on key press with focus on ButtonShowLoadingTree and none of its controls.
function ButtonShowLoadingTree_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to ButtonShowLoadingTree (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global x1; 
global x2;
global x3;
global fisA;
x1      = str2num(get(handles.edit5,'String'))
x2      = str2num(get(handles.edit6,'String'))
x3      = str2num(get(handles.edit7,'String'))
output=evalfis([x1 x2 x3],fisA) %class 1
set(handles.text6,'String',num2str(output));



% --- Executes on key press with focus on pushbutton9 and none of its controls.
function pushbutton9_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
global fisA;
global Xtest;
global Ytest;
global Ytrain;
global Xtrain;
Ytrain
Ytest
outputTest=evalfis(Xtest,fisA) %
outputTrain=evalfis(Xtrain,fisA) %
% количество верно классифицированных – дерево решений
Ctrain = confusionmat(Ytrain,outputTrain);
Ctest = confusionmat(Ytest,outputTest);
%Выводим в процентах
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100

%ЭКСПЕРТНАЯ СИСТЕМА
function pushbutton11_Callback(hObject, eventdata, handles)
global fisA;
%Прием выбранных значений входных параметров
r3=get(handles.radiobutton3,'Value');
r4=get(handles.radiobutton4,'Value');

r5=get(handles.radiobutton5,'Value');
r6=get(handles.radiobutton6,'Value');
r7=get(handles.radiobutton7,'Value');

r8=get(handles.radiobutton8,'Value');
r9=get(handles.radiobutton9,'Value');
%Фаззификация
if (r3==1)
    x1=0.2761 %берем среднее - вершину треугольной функции
end    
if (r4==1)
    x1=0.7761 
end  
%-----------------------------------
if (r5==1)
    x2=0.2691
end 
if (r6==1)
    x2=0.5458
end  
if (r7==1)
    x2=0.7767
end  
%-----------------------------------
if (r8==1)
    x3=0.2729
end  
if (r9==1)
    x3=0.7729
end 
%загрузка системы нечеткого вывода из файла
%не будем загружать

%выполнение нечеткого вывода для заданных значений входных переменных
y=evalfis([x1 x2 x3],fisA) %class 1
set(handles.text13,'String',num2str(y));




% --- Executes on key press with focus on pushbutton11 and none of its controls.
function pushbutton11_KeyPressFcn(hObject, eventdata, handles)
