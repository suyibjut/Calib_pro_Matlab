classdef CalibDataElement
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties        
        lidar_point_start
        lidar_point_end
        lidar_point_p1
        lidar_point_p2
        
        lidar_point_k1
        lidar_point_k2
        lidar_point_k3
        lidar_point_k4
        lidar_unit
        
        cam_point_start
        cam_point_end
        cam_point_p1
        cam_point_p2
        cam_point_k1
        cam_point_k2
        cam_point_k3
        cam_point_k4        
        
    end
    
    methods
        function obj = CalibDataElement( )
            obj.lidar_unit = 'm';
            obj.lidar_point_start=[];
            obj.lidar_point_end=[];
            obj.lidar_point_p1 = [];
            obj.lidar_point_p2 =[];
            obj.lidar_point_k1 =[];
            obj.lidar_point_k2 =[];
            obj.lidar_point_k3 =[];
            obj.lidar_point_k4 =[];
            
            obj.cam_point_start =[];
            obj.cam_point_end =[];
            obj.cam_point_p1 = [];
            obj.cam_point_p2 = [];
            obj.cam_point_k1 = [];
            obj.cam_point_k2 = [];
            obj.cam_point_k3 = [];
            obj.cam_point_k4 = [];
            
        end
        function obj=meter2mm(obj)  % 该函数一定要再采集完所有数据后使用
            obj.lidar_unit = 'mm';
            obj.lidar_point_start = 1000 .* obj.lidar_point_start;
            obj.lidar_point_end = 1000 .* obj.lidar_point_end;
            obj.lidar_point_p1 =  1000 .* obj.lidar_point_p1;
            obj.lidar_point_p2 = 1000 .* obj.lidar_point_p2;

            obj.lidar_point_k1 = 1000 .* obj.lidar_point_k1;
            obj.lidar_point_k2 = 1000 .* obj.lidar_point_k2;
            obj.lidar_point_k3 = 1000 .* obj.lidar_point_k3;
            obj.lidar_point_k4 = 1000 .* obj.lidar_point_k4;
        end
    end
    
end

