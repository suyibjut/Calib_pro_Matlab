classdef PlateBoard <handle
    properties
        Width
        Height
        Outerboarder
        Rectwidth
        Trianglespace
        Triangleheight  %直角三角形高
        Triangleangle    %直角三角形短边对角（degree）
        
        Slope
        Angle %rad
        Intercept
        Point 
        Slope_flag   % 1 or -1
    end
    
    methods
        function obj = PlateBoard( w, h )
            if nargin >0              
                obj.Width = w;
                obj.Height = h;
            else 
                % Default plate is 400mm X 200mm 
                obj.Width = 460.94;
                obj.Height = 230;
            end
            obj.Outerboarder = 15;
            obj.Rectwidth = 40;
            obj.Triangleheight = 200;
            obj.Triangleangle = 30;
            obj.Trianglespace = 40;
            obj.Slope_flag = 1;
        end

        function obj=judgesign( obj, line_1, line_2 )
            if line_1 > line_2 
                disp('Positive slope')
                obj.Slope_flag = 1;
            else 
                disp('Negative slope');
                obj.Slope_flag = -1;
            end
        end
        
        function computeslope(  obj, obliquelen , horizontallen)
            obj.Slope = obj.Slope_flag * sqrt(obliquelen^2-horizontallen^2)/horizontallen;
            obj.Angle = acos( horizontallen/obliquelen );
        end

        function  computepoint(  obj, cutline , num )
            if num>2 || num <0
                prompt = {'Enter a valid num 1 or 2'};
                dlg_title = 'Input';
                defaultans = {'1'};
                num_lines = 1;
                answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
                num = str2double( answer{1} );
            end
            if isempty(obj.Angle)
                prompt = {'Enter oblique length(mm):','Enter horizontal length(mm):'};
                dlg_title = 'Input';
                num_lines = 1;
                defaultans = {'400','400'};
                answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
                obliquelen = str2double( answer{1} );
                horizontallen = str2double( answer{2} );
                obj.computeslope( obliquelen , horizontallen);
            end
            h_tribottom = obj.Triangleheight * tan(obj.Triangleangle/180.0*pi);
            h_cutline = cutline * cos( obj.Angle ) -(num-1)*(h_tribottom+obj.Trianglespace);
            y = obj.Outerboarder + obj.Height * ( 1-h_cutline/h_tribottom );
            x = (obj.Outerboarder + obj.Rectwidth + obj.Trianglespace) + h_cutline+(num-1)*(h_tribottom+obj.Trianglespace);
            obj.Point = [x,y];   
            obj.Intercept = y - obj.Slope*x;
            
        end
    end

end