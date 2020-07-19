function varargout = CalibrationUI(varargin)
% CALIBRATIONUI MATLAB code for CalibrationUI.fig
%      CALIBRATIONUI, by itself, creates a new CALIBRATIONUI or raises the existing
%      singleton*.
%
%      H = CALIBRATIONUI returns the handle to a new CALIBRATIONUI or the handle to
%      the existing singleton*.
%
%      CALIBRATIONUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALIBRATIONUI.M with the given input arguments.
%
%      CALIBRATIONUI('Property','Value',...) creates a new CALIBRATIONUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CalibrationUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CalibrationUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CalibrationUI

% Last Modified by GUIDE v2.5 02-Apr-2019 20:11:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CalibrationUI_OpeningFcn, ...
                   'gui_OutputFcn',  @CalibrationUI_OutputFcn, ...
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


% --- Executes just before CalibrationUI is made visible.
function CalibrationUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CalibrationUI (see VARARGIN)

% Choose default command line output for CalibrationUI
handles.output = hObject;
%handles.laser = [];
handles.udp = [];
global lidarplottimer;
global cameraplottimer;
global validateplottimer;
lidarplottimer = timer('TimerFcn',{@lasertimer_Callback,handles}, 'Period',0.5,'executionmode','fixedrate');
cameraplottimer = timer('TimerFcn',{@cameratimer_Callback,handles}, 'Period',0.2,'executionmode','fixedrate');
validateplottimer = timer('TimerFcn',{@validatetimer_Callback,handles}, 'Period',0.2,'executionmode','fixedrate');

lasernode = LaserNode;
set( handles.pushbutton_playlidarscan, 'UserData', lasernode);

camnode = CamNode;
set( handles.pushbutton_playcamera, 'UserData', camnode );

dataelement = CalibDataElement();
set( handles.pushbutton_ok1, 'UserData',dataelement);
set( handles.pushbutton_ok2, 'UserData',dataelement);

handles.plateboard = PlateBoard();
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CalibrationUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CalibrationUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function lasertimer_Callback(hObject,eventdata, handles)
%handles.laser = rossubscriber( '/scan' );
lasernode = get(handles.pushbutton_playlidarscan,'UserData');
lasernode.scandata = receive(lasernode.laser,10);
lasernode.scandata.RangeMin = 0.0;

lasernode.xy = readCartesian(lasernode.scandata);
loops = 6;
for i=1:loops
    lasernode.scandata = receive(lasernode.laser,10);
    lasernode.scandata.RangeMin = 0.0;
    lasernode.xy = lasernode.xy + readCartesian(lasernode.scandata);
end
lasernode.xy = lasernode.xy ./(loops+1);

plot( handles.lidar_axes, lasernode.xy(:,1),lasernode.xy(:,2));
%scatter( handles.lidar_axes, lasernode.xy(:,1),lasernode.xy(:,2));
set( handles.lidar_axes, 'XLim',[0,1.2]);
set( handles.lidar_axes, 'YLim',[-0.8,0.8]);

function cameratimer_Callback( hObject, eventdata, handles )
camnode = get( handles.pushbutton_playcamera, 'UserData');
if camnode.imgmsg == []
    msgbox( 'Please Test The Camera First' );
else
    camnode.imgmsg = receive( camnode.imgsubscriber,10 );
    camnode.formattedimg = readImage( camnode.imgmsg );
    %axes( handles.cam_axes );
    imshow( camnode.formattedimg, 'parent', handles.cam_axes );
    set( handles.pushbutton_playcamera, 'UserData', camnode ); 
end
%guidata( hObject, handles );

function validatetimer_Callback( hObject, eventdata, handles)
lasernode = get(handles.pushbutton_playlidarscan,'UserData');
lasernode.scandata = receive(lasernode.laser,10);
lasernode.scandata.RangeMin = 0.0;
lasernode.xy = readCartesian(lasernode.scandata);

plot( handles.lidar_axes, lasernode.xy(:,1),lasernode.xy(:,2));
set( handles.lidar_axes, 'XLim',[-0,1.2]);
set( handles.lidar_axes, 'YLim',[-0.8,0.8]);

lidar_points = [1000*lasernode.xy,ones(length(lasernode.xy),1)]'; % meter to milimeter
%-----------------
disp('lidar_points_raw')
lidar_points(:,1:20)

lidar_x_mask = (lidar_points(1,:)<1200)&(lidar_points(1,:)>0);
lidar_y_mask = abs(lidar_points(2,:))<500;
lidar_mask = lidar_x_mask&lidar_y_mask;

lidar_points =[ lidar_points(1,lidar_mask);lidar_points(2,lidar_mask)];



disp('lidarpoints')
disp(size(lidar_points));

lidar_points = [lidar_points;ones(1,length(lidar_points)) ];

disp('lidar_points_refine')
lidar_points(:,1:20)

disp('lidarpoints')
disp(size(lidar_points));
%-----------------
Rt = get( handles.edit_calibrationresult,'UserData' );

disp('Rt')
disp(size(Rt));

img_points = (Rt * lidar_points)';

disp('img_points')
disp(size(img_points));

img_uv = round( img_points(:,1:2) );
camnode = get( handles.pushbutton_playcamera, 'UserData');

camnode.imgmsg = receive( camnode.imgsubscriber,10 );
camnode.formattedimg = readImage( camnode.imgmsg );
img_size = size( camnode.formattedimg ); %[h,w,depth]

% u:height v:width
img_u_mask = (img_uv(:,1)>0)&(img_uv(:,1)< img_size(2) );
img_v_mask = (img_uv(:,2)>0)&(img_uv(:,2)< img_size(1) );
img_uv_mask = img_u_mask&img_v_mask;
disp('********************')
u = img_uv( img_uv_mask,1 );
v = img_uv( img_uv_mask,2 );
%u = img_uv(:,1);
%v = img_uv(:,2);
rect_size = 2;
for i=1:length(v)
    v(i)
    u(i)
    if v(i)-rect_size < 0
        left = 1;
    else 
        left = abs(v(i)-rect_size)+1;
    end
    if v(i)+rect_size > img_size(1)
        right = img_size(1);
    else 
        right = v(i)+rect_size;
    end
    if u(i)-rect_size < 0
        bottom = 1;
    else 
        bottom = abs(u(i)-rect_size)+1;
    end
    if u(i)+rect_size > img_size(2)
        top = img_size(2);
    else 
        top = u(i)+rect_size;
    end

    camnode.formattedimg(left:right,bottom:top,1)=255;
    camnode.formattedimg(left:right,bottom:top,2)=0;
    camnode.formattedimg(left:right,bottom:top,3)=0;
end

imshow( camnode.formattedimg, 'parent', handles.cam_axes );

%plot( handles.cam_axes,img_uv(img_uv_mask,1), img_uv(img_uv_mask,2),'o');
%scatter( img_uv(img_uv_mask,1), img_uv(img_uv_mask,2), 10 ,'filled','red','parent',handles.cam_axes);
set( handles.pushbutton_playcamera, 'UserData', camnode ); 


function edit_hostIP_Callback(hObject, eventdata, handles)
% hObject    handle to edit_hostIP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_hostIP as text
%        str2double(get(hObject,'String')) returns contents of edit_hostIP as a double


% --- Executes during object creation, after setting all properties.
function edit_hostIP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_hostIP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_connectmaster.
function pushbutton_connectmaster_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_connectmaster (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
masterIP_addr = get( handles.edit_hostIP, 'String');
% Check if is a valid IP
ipcell = strsplit( masterIP_addr,'.' );

if length( ipcell ) <4
    warndlg('Not a valid IP address, length is too short');
    return
end
for i=1:4
    tmp = str2num(ipcell{i} );
    if isempty( tmp )
        warndlg('Not a valid IP address, all should be numbers');
        return
    else 
        if tmp>255
            warndlg('IP address can not be larger than 255');
            return
        end
    end
end
masterIP_port =  get( handles.edit_port, 'String');
masterIP_addr = ['http://',masterIP_addr,':',masterIP_port];
if isempty( str2num(masterIP_port ) )
    warndlg('Port must be a number, 0<port<65535');
end
msgbox('Matlab ROS: Please wait for some seconds','Connecting to Master');
rosinit( masterIP_addr );
msgbox('Matlab ROS UP!','Connected to Master');
    

function edit_port_Callback(hObject, eventdata, handles)
% hObject    handle to edit_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_port as text
%        str2double(get(hObject,'String')) returns contents of edit_port as a double


% --- Executes during object creation, after setting all properties.
function edit_port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_testlidar.
function pushbutton_testlidar_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_testlidar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
topiclist = rostopic('list');
scan_flag = false;

for i=1:1:length( topiclist )
    if ~strcmp(topiclist{i},'/scan')
        scan_flag = true;
    end
end
if scan_flag 
    msgbox( 'Lidar Works Fine!');
    lasernode = get(handles.pushbutton_playlidarscan,'UserData');
    lasernode.laser = rossubscriber( '/scan' );
    lasernode.scandata = receive(lasernode.laser, 10);
    lasernode.xy = readCartesian(lasernode.scandata);
    set( handles.pushbutton_playlidarscan, 'UserData',lasernode);
    plot( handles.lidar_axes, lasernode.xy(:,1),lasernode.xy(:,2) );
    set( handles.lidar_axes, 'XLim',[-8,8]);
    set( handles.lidar_axes, 'YLim',[-8,8]);
else
    msgbox( 'No /scan topic,Please Check Lidar Node');
end

guidata( hObject, handles );

% --- Executes on button press in pushbutton_testcam.
function pushbutton_testcam_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_testcam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
camnode = get( handles.pushbutton_playcamera, 'UserData');
camnode.imgmsg = rosmessage('sensor_msgs/Image');
camnode.imgsubscriber = rossubscriber( '/usb_cam/image_raw' );
camnode.imgmsg = receive( camnode.imgsubscriber ,10);
camnode.formattedimg = readImage( camnode.imgmsg );

imshow( camnode.formattedimg ,'parent',handles.cam_axes );
set( handles.pushbutton_playcamera, 'UserData', camnode );

% --- Executes on button press in pushbutton_disconnectmaster.
function pushbutton_disconnectmaster_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_disconnectmaster (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
rosshutdown
msgbox( 'Matlab ROS shutdown');

% --- Executes on button press in pushbutton_playlidarscan.
function pushbutton_playlidarscan_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_playlidarscan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lidarplottimer;
if strcmp(lidarplottimer.Running ,'off')
    start(lidarplottimer);
end
guidata(hObject, handles);

% --- Executes on button press in pushbutton_playcamera.
function pushbutton_playcamera_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_playcamera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cameraplottimer;
if strcmp( cameraplottimer.Running , 'off')
    start( cameraplottimer );
end
guidata(hObject, handles);

% --- Executes on button press in pushbutton_stoptocalibrate1.
function pushbutton_stoptocalibrate1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_stoptocalibrate1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lidarplottimer;
if strcmp( lidarplottimer.Running , 'on')
    stop(lidarplottimer);
end

% --- Executes on button press in pushbutton_stoptocalibrate2.
function pushbutton_stoptocalibrate2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_stoptocalibrate2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global lidarplottimer;
if strcmp( lidarplottimer.Running , 'on')
    stop(lidarplottimer);
end

% --- Executes on button press in pushbutton_finalcalibration.
function pushbutton_finalcalibration_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_finalcalibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pose_1 = get( handles.pushbutton_ok1, 'UserData' );
pose_2 = get( handles.pushbutton_ok2, 'UserData');

% pose one:p1 pose two:p2 
pXY = [
    pose_1.lidar_point_start;
    pose_1.lidar_point_end;
    pose_1.lidar_point_p1;
    pose_1.lidar_point_p2;
    
    pose_2.lidar_point_start;
    pose_2.lidar_point_end;
    pose_2.lidar_point_p1;
    pose_2.lidar_point_p2;
]

pUV = [
    pose_1.cam_point_start;
    pose_1.cam_point_end;
    pose_1.cam_point_p1;
    pose_1.cam_point_p2;

    pose_2.cam_point_start;
    pose_2.cam_point_end;
    pose_2.cam_point_p1;
    pose_2.cam_point_p2;
]

point_num = length(pXY);
U = zeros( 3*point_num, 9);
size(U)
for i=1:3
    row_base = (i-1)*point_num+1;
    col_base = (i-1)*3+1;
    
    U( row_base:row_base+7, col_base+0 ) = pXY(:,1);
    U( row_base:row_base+7, col_base+1 ) = pXY(:,2);
    U( row_base:row_base+7, col_base+2 ) = ones( point_num, 1);
end

Y = [ pUV(:,1); pUV(:,2); ones( point_num, 1 )];

X = (U'*U)\(U'*Y);
Rt = [ X(1:3)';X(4:6)';X(7:9)'];

set( handles.edit_calibrationresult,'UserData', Rt );
set( handles.edit_calibrationresult,'String', {mat2str(Rt(1,1:3)),mat2str(Rt(2,1:3)),mat2str(Rt(3,1:3))} );


function edit_calibrationresult_Callback(hObject, eventdata, handles)
% hObject    handle to edit_calibrationresult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_calibrationresult as text
%        str2double(get(hObject,'String')) returns contents of edit_calibrationresult as a double


% --- Executes during object creation, after setting all properties.
function edit_calibrationresult_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_calibrationresult (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_select1.
function pushbutton_select1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_select1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dataelement = get( handles.pushbutton_ok1, 'UserData' );
waitforkbhit();
dataelement.lidar_point_start = ginput(1);
waitforkbhit();
dataelement.lidar_point_end = ginput(1);

axes( handles.lidar_axes );
hold on;
plot( handles.lidar_axes, [dataelement.lidar_point_start(1,1),dataelement.lidar_point_end(1,1)],...
    [dataelement.lidar_point_start(1,2),dataelement.lidar_point_end(1,2)],'r','LineWidth',1 );

waitforkbhit();
dataelement.lidar_point_p1 = ginput(1);
waitforkbhit();
dataelement.lidar_point_p2 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_p1(1,1),dataelement.lidar_point_p2(1,1)],...
    [dataelement.lidar_point_p1(1,2),dataelement.lidar_point_p2(1,2)],'g' ,'LineWidth',2);

waitforkbhit();
dataelement.lidar_point_k1 = ginput(1);
waitforkbhit();
dataelement.lidar_point_k2 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_k1(1,1),dataelement.lidar_point_k2(1,1)],...
    [dataelement.lidar_point_k1(1,2),dataelement.lidar_point_k2(1,2)],'m', 'LineWidth',3 );

waitforkbhit();
dataelement.lidar_point_k3 = ginput(1);
waitforkbhit();
dataelement.lidar_point_k4 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_k3(1,1),dataelement.lidar_point_k4(1,1)],...
    [dataelement.lidar_point_k3(1,2),dataelement.lidar_point_k4(1,2)],'c', 'LineWidth',3  );
% 采集完所有点，将雷达数据转换成mm
dataelement = dataelement.meter2mm();

% 计算斜率以及其中一点，求出曲线方程,ginput获得的单位为m，先要换算成mm
disp('line:');

line_1 = sqrt( sum( (abs(dataelement.lidar_point_k1 - dataelement.lidar_point_k2)).^2 ) )
line_2 = sqrt( sum( (abs(dataelement.lidar_point_k3 - dataelement.lidar_point_k4)).^2 ) )
handles.plateboard.judgesign( line_1 , line_2 );

obliquelen = sqrt( sum( (abs(dataelement.lidar_point_end - dataelement.lidar_point_start)).^2 ) )
horizontallen = handles.plateboard.Width - 2*handles.plateboard.Outerboarder
handles.plateboard.computeslope( obliquelen, horizontallen );

cutline = sqrt( sum( (abs(dataelement.lidar_point_p1 - dataelement.lidar_point_p2)).^2 ) );
handles.plateboard.computepoint( cutline, 2 );

disp('slope')
slope = string( handles.plateboard.Slope )
intercept = string( abs( handles.plateboard.Intercept ) )
tosend = slope + ' ' + intercept;
fwrite( handles.udp, char(tosend) );
hold off
set( handles.pushbutton_ok1, 'UserData', dataelement);
% 延时0.5s 让显示器显示
pause(0.5);
global cameraplottimer;
if strcmp(cameraplottimer.Running ,'on')
    stop(cameraplottimer);
end

camnode = get( handles.pushbutton_playcamera, 'UserData');
camnode.imgmsg = rosmessage('sensor_msgs/Image');
camnode.imgsubscriber = rossubscriber( '/usb_cam/image_raw' );
camnode.imgmsg = receive( camnode.imgsubscriber ,10);
camnode.formattedimg = readImage( camnode.imgmsg );
disp('image size:')
size( camnode.formattedimg )

imshow( camnode.formattedimg ,'parent',handles.cam_axes );
set( handles.pushbutton_playcamera, 'UserData', camnode );

waitforkbhit();
dataelement.cam_point_start = ginput(1);
waitforkbhit();
dataelement.cam_point_end = ginput(1);
waitforkbhit();
dataelement.cam_point_p1 = ginput(1);
waitforkbhit();
dataelement.cam_point_p2 = ginput(1);

set( handles.pushbutton_ok1, 'UserData', dataelement );

guidata( hObject, handles );

% --- Executes on button press in pushbutton_reselect1.
function pushbutton_reselect1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reselect1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes( handles.lidar_axes );
hold off

% --- Executes on button press in pushbutton_ok1.
function pushbutton_ok1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ok1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_select2.
function pushbutton_select2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_select2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
dataelement = get( handles.pushbutton_ok2, 'UserData' );
waitforkbhit();
dataelement.lidar_point_start = ginput(1);
waitforkbhit();
dataelement.lidar_point_end = ginput(1);

axes( handles.lidar_axes );
hold on;
plot( handles.lidar_axes, [dataelement.lidar_point_start(1,1),dataelement.lidar_point_end(1,1)],...
    [dataelement.lidar_point_start(1,2),dataelement.lidar_point_end(1,2)],'r', 'LineWidth',1 );

waitforkbhit();
dataelement.lidar_point_p1 = ginput(1);
waitforkbhit();
dataelement.lidar_point_p2 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_p1(1,1),dataelement.lidar_point_p2(1,1)],...
    [dataelement.lidar_point_p1(1,2),dataelement.lidar_point_p2(1,2)],'g','LineWidth',2 );

waitforkbhit();
dataelement.lidar_point_k1 = ginput(1);
waitforkbhit();
dataelement.lidar_point_k2 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_k1(1,1),dataelement.lidar_point_k2(1,1)],...
    [dataelement.lidar_point_k1(1,2),dataelement.lidar_point_k2(1,2)],'m' , 'LineWidth',3);

waitforkbhit();
dataelement.lidar_point_k3 = ginput(1);
waitforkbhit();
dataelement.lidar_point_k4 = ginput(1);

plot( handles.lidar_axes, [dataelement.lidar_point_k3(1,1),dataelement.lidar_point_k4(1,1)],...
    [dataelement.lidar_point_k3(1,2),dataelement.lidar_point_k4(1,2)],'c', 'LineWidth',3 );
% 采集完所有点，将雷达数据转换成mm
dataelement = dataelement.meter2mm();

% 计算斜率以及其中一点，求出曲线方程,ginput获得的单位为m，先要换算成mm
line_1 = sqrt( sum( (abs(dataelement.lidar_point_k1 - dataelement.lidar_point_k2)).^2 ) );
line_2 = sqrt( sum( (abs(dataelement.lidar_point_k3 - dataelement.lidar_point_k4)).^2 ) );
disp('line')
line_1
line_2
if line_1 < line_2
    handles.plateboard.Slope_flag = -1
end
handles.plateboard.judgesign( line_1 , line_2 );
handles.plateboard.Slope_flag

% 计算斜率以及其中一点，求出曲线方程,ginput获得的单位为m，先要换算成mm
obliquelen = sqrt( sum( (abs(dataelement.lidar_point_end - dataelement.lidar_point_start)).^2 ) )
horizontallen = handles.plateboard.Width - 2*handles.plateboard.Outerboarder
handles.plateboard.computeslope( obliquelen, horizontallen );

cutline = sqrt( sum( (abs(dataelement.lidar_point_p1 - dataelement.lidar_point_p2)).^2 ) )
handles.plateboard.computepoint( cutline, 2 );

slope = string( abs(handles.plateboard.Slope) )
intercept = string( abs( handles.plateboard.Intercept ) )
tosend = slope + ' ' + intercept;
fwrite( handles.udp, char(tosend) );
hold off
set( handles.pushbutton_ok2, 'UserData', dataelement);
% 延时0.5s 让显示器显示
pause(0.5);
global cameraplottimer;
if strcmp(cameraplottimer.Running ,'on')
    stop(cameraplottimer);
end

camnode = get( handles.pushbutton_playcamera, 'UserData');
camnode.imgmsg = rosmessage('sensor_msgs/Image');
camnode.imgsubscriber = rossubscriber( '/usb_cam/image_raw' );
camnode.imgmsg = receive( camnode.imgsubscriber ,10);
camnode.formattedimg = readImage( camnode.imgmsg );

imshow( camnode.formattedimg ,'parent',handles.cam_axes );
set( handles.pushbutton_playcamera, 'UserData', camnode );

waitforkbhit();
dataelement.cam_point_start = ginput(1);
waitforkbhit();
dataelement.cam_point_end = ginput(1);
waitforkbhit();
dataelement.cam_point_p1 = ginput(1);
waitforkbhit();
dataelement.cam_point_p2 = ginput(1);

set( handles.pushbutton_ok2, 'UserData', dataelement );

guidata( hObject, handles );

% --- Executes on button press in pushbutton_reselect2.
function pushbutton_reselect2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reselect2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_ok2.
function pushbutton_ok2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ok2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit_displayserver_Callback(hObject, eventdata, handles)
% hObject    handle to edit_displayserver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_displayserver as text
%        str2double(get(hObject,'String')) returns contents of edit_displayserver as a double


% --- Executes during object creation, after setting all properties.
function edit_displayserver_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_displayserver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_connectdisplayserver.
function pushbutton_connectdisplayserver_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_connectdisplayserver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
DisplayServer_addr = get( handles.edit_displayserver, 'String');
% Check if is a valid IP
ipcell = strsplit( DisplayServer_addr,'.' );

if length( ipcell ) <4
    warndlg('Not a valid IP address, length is too short');
    return
end
for i=1:4
    tmp = str2num(ipcell{i} );
    if isempty( tmp )
        warndlg('Not a valid IP address, all should be numbers');
        return
    else 
        if tmp>255
            warndlg('IP address can not be larger than 255');
            return
        end
    end
end

handles.udp = udp(DisplayServer_addr,5555);
fopen( handles.udp);
fwrite( handles.udp,'0.5 0');
pause(0.5);
fwrite( handles.udp,'0 0');
guidata(hObject , handles);


% --- Executes on button press in pushbutton_closeudp.
function pushbutton_closeudp_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_closeudp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if strcmp( handles.udp.Status , 'open')
    fclose( handles.udp);
end
guidata(hObject, handles);

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.udp) && ( strcmp( handles.udp.Status , 'open') )
    fclose( handles.udp);
end
rosshutdown;
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on button press in pushbutton_validate.
function pushbutton_validate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_validate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global validateplottimer;
if strcmp(validateplottimer.Running,'on')
    stop(validateplottimer);
    set(handles.pushbutton_validate,'String','Validation');
elseif strcmp(validateplottimer.Running,'off')
    start(validateplottimer);
    set(handles.pushbutton_validate,'String','Stop');
end


% --- Executes on button press in pushbutton_save.
function pushbutton_save_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Rt = get( handles.edit_calibrationresult,'UserData');
if isempty( Rt )
    return 
end
FileName = uiputfile('Rt.mat','Save Rt Matrix');
if FileName == 0
    return
end
save( FileName , 'Rt' );

% --- Executes on button press in pushbutton_load.
function pushbutton_load_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

FileName = uigetfile('calib_last.mat','Get Rt Matrix');
if FileName == 0
    return
end
tmp = load(FileName);
Rt = tmp.Rt;
set( handles.edit_calibrationresult,'UserData', Rt);
set( handles.edit_calibrationresult,'String', {mat2str(Rt(1,1:3)),mat2str(Rt(2,1:3)),mat2str(Rt(3,1:3))} );
