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
global Y;%���������������� �������
global cv; %��� ���������� �� ��������� Test � Train
global Xtrain;
global Ytrain;
global Xtest;
global Ytest;



 range      = str2num(get(handles.edit1,'String'))
 count      = str2num(get(handles.edit2,'String'))
 classes    = str2num(get(handles.edit3,'String'))
 attribute  = str2num(get(handles.edit4,'String'))
%----------------------------------------------------------
%������������ ������� �������� ������
X=gererateRandom(range,count,classes,attribute) 
X=normr(X);
%----------------------------------------------------------
%���������������� �������
for j=1:classes
    for i=((j-1)*count+1):count*j
        Y(((j-1)*count+1):count*j,1)=j;%
    end
end
%----------------------------------------------------------
%������ ������ �������� ������
axes(handles.axes1)%������� ��� ��������
plot3(X(:,1),X(:,2),X(:,3),'.')%��������� �������
%----------------------------------------------------------
% � ������� 40% ������ ��������� ������� ���������� � �������� �����
cv = cvpartition((classes*count),'holdout',0.40)
%������� ��������� ���������� n ���������� �� ��������� ������� � ����������� �������. 
%�������� p ������ ���� 0 < p < 1, � ���� ������ ������� cvpartition 
%�������� �������� �������� p*n ���������� ��� ����������� �������.

%cv = 
%Hold-out cross validation partition
%             N: 300
%   NumTestSets: 1
%     TrainSize: 180
%      TestSize: 120

%----------------------------------------------------------
% ��������� ���������
%training(c) � ���������� ���������� ������  
%��� ������� ������� ��������� ������� (1) �� ������� ����������� ������� (0).
%X(training(cv),:) ��������� ���������� X �� ����� ��������� �������.
Xtrain = X(training(cv),:)
Ytrain = Y(training(cv),:)
%----------------------------------------------------------
% �������� ���������
%test(c) � ���������� ���������� ������   ��� ������� ������� 
%����������� ������� (1) �� ������� ��������� ������� (0).
Xtest = X(test(cv),:)
Ytest = Y(test(cv),:)
%----------------------------------------------------------
%���������� ������� ������
%tabulate(x) ������� ������� ������ ������ �� ������� x. 
%���������� � ������� ������������ ��������� �������:
%1 ������� � ���������� �������� x
%2 ������� � ����� ���������� ������� ��������
%3 ������� � ������� ������� ��������
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
%���������� ������ � ���� 
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
global YnewTrain;%������������ �� ��������� �������
global YnewTest;%������������ �� �������� �������
global Ctrain;%���������� ��������� �������������
global Ctest;%���������� ��������� �������������
global C_ts;%������� ��������� ������������� 
global C_tr%������� ��������� �������������

load optimalTree.mat tree1;
load X.mat X Y;
%----------------------------------------------------------
%������ ������ �������� ������
axes(handles.axes1)%������� ��� ��������
plot3(X(:,1),X(:,2),X(:,3),'.')%��������� �������
%----------------------------------------------------------
%����� ������ ������������ ������ �� �����
view(tree1,'mode','graph');
%----------------------------------------------------------
% ��������� ���������
%training(c) � ���������� ���������� ������  
%��� ������� ������� ��������� ������� (1) �� ������� ����������� ������� (0).
%X(training(cv),:) ��������� ���������� X �� ����� ��������� �������.
%----------------------------------------------------------
% � ������� 40% ������ ��������� ������� ���������� � �������� �����
count      = str2num(get(handles.edit2,'String'))
classes    = str2num(get(handles.edit3,'String'))
cv         = cvpartition((classes*count),'holdout',0.40)

Xtrain = X(training(cv),:)
Ytrain = Y(training(cv),:)
%----------------------------------------------------------
% �������� ���������
%test(c) � ���������� ���������� ������   ��� ������� ������� 
%����������� ������� (1) �� ������� ��������� ������� (0).
Xtest = X(test(cv),:)
Ytest = Y(test(cv),:)
%--------------------------------------------------------------------------
%������������ �������� ����������������� �������� �� ������
%��� ������ ����� ������� Xnew �������� ���������������� ������� �
%����������� � �������, ��� �������� ������������ � ������ Ynew
%Ynew=predict(tree,Xnew)
%Ynew = predict(tree,X)
YnewTrain = predict(tree1,Xtrain)%������������ �� ���������
YnewTest = predict(tree1,Xtest)%�������������� �� ��������
% ���������� ����� ������������������ � ������ �������
Ctrain = confusionmat(Ytrain,YnewTrain);
Ctest = confusionmat(Ytest,YnewTest);
%������� � ���������
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100

function LoadButton_KeyPressFcn(hObject, eventdata, handles)


function ButtonTreeFit_Callback(hObject, eventdata, handles)
global X;
global Y; 
global BinarTree;%�������� ������
global Ynew;
global tree1;%����������� ������
%������ ��������� ������ ������������� ���������� 
BinarTree = treefit(X,Y);
%����������� ��������� ������
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

global YnewTrain;%������������ �� ��������� �������
global YnewTest;%������������ �� �������� �������

global Ctrain;%���������� ��������� �������������
global Ctest;%���������� ��������� �������������

global C_ts;%������� ��������� ������������� 
global C_tr%������� ��������� �������������

global X;
%������������ ������ �������������
%tree = fitctree(x,y) ���������� �������� ������ ������������� tree, 
%���������� �� ������� ���������� x � �������� ���������� y. 
%� �������� ������ ������ ���� ����� �� ����� ���� �������� �� ���������
%������� ������� x.
tree = fitctree(Xtrain,Ytrain)
%----------------------------------------------------------
%����� ������ ������������� �� �����
view(tree,'mode','graph');
%----------------------------------------------------------
%���������� ������������ ������ ���������� ������ �� ���� �����������
%������
%������� ���������� ��������� ������� ��������� ������,
%��� ��������� �������� ��������� �� ������������
[~,~,~, bestlevel] = cvLoss(tree,'SubTrees','All','TreeSize','min')
%-----------------------------------------------------------
%���������� ������ �� ������������ ������
%��������� ������������ ������
tree1 = prune(tree,'Level',bestlevel);
%----------------------------------------------------------
%����� ������ ������������ ������ �� �����
view(tree1,'mode','graph');
%----------------------------------------------------------
%������������ �������� ����������������� �������� �� ������
%��� ������ ����� ������� Xnew �������� ���������������� ������� �
%����������� � �������, ��� �������� ������������ � ������ Ynew
%Ynew=predict(tree,Xnew)
%Ynew = predict(tree,X)
YnewTrain = predict(tree1,Xtrain)%������������ �� ���������
YnewTest  = predict(tree1,Xtest)%�������������� �� ��������
% ���������� ����� ������������������ � ������ �������
Ctrain = confusionmat(Ytrain,YnewTrain);
Ctest = confusionmat(Ytest,YnewTest);
%������� � ���������
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100


function FitcTreeButton_KeyPressFcn(hObject, eventdata, handles)


% --- Executes on button press in ButtonFizzyLogic.
function ButtonFizzyLogic_Callback(hObject, eventdata, handles)
%==========================================================================
%������� ��������� ������
%� ���� ������� �� �� ����� ���������� �������� ����������� ��� ��� ����
%������� ���������������� ��������� ���������
%�������� �������, ������������ ������� ����������� �������������� ������
%�� �������� ������. ������� ���������� �������� ���� ������� �� 0 �� 1
%��������������� ���������� - ����������, ���������� ������� ����� ����
%�����
%���� ��������� - ��������� ���� ��������� ��������  ���������������
%���������� 
%���� - ����� ������� ���� ���������
%--------------------------------------------------------------------------
%������� ��������������
%1.������� - �������� 
    %1.1. �����������
    %1.2. ��������������
%2.Z �������� � S ��������
%--------------------------------------------------------------------------
%�������� ���������� ��������
%A � B  =min(P(A),P(B))
%A ��� B=max(P(A),P(B))
%�� A= 1-P(A)
%���������� A->B= max (1-P(A),P(B))
%--------------------------------------------------------------------------
%������  �������� ������
%����  ������� ������� �� ���������� �������
% ��� ������� � ���������� - ��� ����� �������� ����������
%--------------------------------------------------------------------------
%�����������
% ��������� ������������ ����� ��������� ������� ��������� � ���������
% ������� ��������������  ��������������� �� �����. ��� ����� �������
% �������� ������
%--------------------------------------------------------------------------
%�������������
%��������� ����������� ������� ���������� ������������. ���� ���������� 
%��������� ������������� ������� ��������  ������� �������� �������������, 
% �� ������� ��� ���������� ����� ��������������� ������������ ��������
% ������������ � ������� ���������� ������� ��������� ������� 
%--------------------------------------------------------------------------
%�������� ����� ������� ��������� ����������� ������
global fis_name;
global fis_type;
global fisA;

fis_name='new_fuzzy_system';%��� ������� ��������� ������
fis_type='sugeno';%��� ������� ��������� ������
    %���������� ��������:
    %'mamdani' - ������� ���� �������
    %'sugeno' - ������� ���� ������ 
fisA = newfis(fis_name,fis_type) 
%���������� ���������� � ������� ��������� ����������� ������

fisA=addvar(fisA,'input', 'x1',[0 1])
fisA=addvar(fisA,'input', 'x2',[0 1])
fisA=addvar(fisA,'input', 'x3',[0 1])
fisA=addvar(fisA,'output', 'class',[1 3])
%--------------------------------------------------------------------------
%���������� ������� �������������� � ������� ��������� ������	
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
%�������
r1='if x1 is L then class is C                              ';
r2='if x1 is H and x3 is L then class is C                  ';
r3='if x1 is H and x3 is H and x2 is L then class is C      ';
r4='if x1 is H and x3 is L and x2 is Average then class is A';
r5='if x1 is H and x3 is L and x2 is H then class is B      ';
inrulelist=char(r1,r2,r3,r4,r5)
fisA = parsrule (fisA, inrulelist) %���� ������ � �������� ���� ������ %char(rule1,rule2,rule3);
%--------------------------------------------------------------------------
%������ ������� ��������� ����������� ������
writefis(fisA,'myfis.fis');
%--------------------------------------------------------------------------   
%�������� ������� ��������� ����������� ������ 
fisA=readfis('myfis.fis')
%--------------------------------------------------------------------------
%���������� ��������� ����������� ������
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
%����� ������ ������������� �� ����� ����� ��� ��������
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
% ���������� ����� ������������������ � ������ �������
Ctrain = confusionmat(Ytrain,outputTrain);
Ctest = confusionmat(Ytest,outputTest);
%������� � ���������
C_ts = bsxfun(@rdivide,Ctest,sum(Ctest,2)) * 100
C_tr = bsxfun(@rdivide,Ctrain,sum(Ctrain,2))*100

%���������� �������
function pushbutton11_Callback(hObject, eventdata, handles)
global fisA;
%����� ��������� �������� ������� ����������
r3=get(handles.radiobutton3,'Value');
r4=get(handles.radiobutton4,'Value');

r5=get(handles.radiobutton5,'Value');
r6=get(handles.radiobutton6,'Value');
r7=get(handles.radiobutton7,'Value');

r8=get(handles.radiobutton8,'Value');
r9=get(handles.radiobutton9,'Value');
%������������
if (r3==1)
    x1=0.2761 %����� ������� - ������� ����������� �������
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
%�������� ������� ��������� ������ �� �����
%�� ����� ���������

%���������� ��������� ������ ��� �������� �������� ������� ����������
y=evalfis([x1 x2 x3],fisA) %class 1
set(handles.text13,'String',num2str(y));




% --- Executes on key press with focus on pushbutton11 and none of its controls.
function pushbutton11_KeyPressFcn(hObject, eventdata, handles)
