classdef boundDetection
   methods
        function delete(obj)
            % obj is always scalar
        end
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Optimize contour
        %
        % Description   :   The function gets the outer contoure of the
        %                   point cloud and optimze them to obtain only
        %                   vertical and horizontal lanes
        % 
        % Inputs        :   1. 2D data points
        %                   2. Scaling factor
        %
        % Outputs       :   1. Optimized data
        % =========================================================================
        function [opti_data] = optimize_data_pos(obj, data, scale)
            len = length(data);
            for i = 1:5
                for j = 1:len-2
                    diff = abs(data(j)-data(j+1));
                    diff2 = abs(data(j)-data(j+2));
                    if diff < scale
                        if diff2 < scale
                            if data(j) < data(j+1) && data(j) < data(j+2)
                                data(j) = data(j+1);
                            else
                                data(j+1) = data(j);
                            end
                        else
                            data(j) = data(j+1);
                        end
                    end
                end
            end
            opti_data = data;   
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Set corner points
        %
        % Description   :   The function selects the points that belong to the
        %                   corners of the bound and stores them in an array
        % 
        % Inputs        :   1. x-corrdinates
        %                   2. y-corrdinates
        %                   3. Threshold value
        %                   4. Margin
        %
        % Outputs       :   1. Selected data points (corners)
        %                   2. Differences between x-values
        %                   3. Differences between y-values
        % =========================================================================
        function [data, div_x, div_y] = set_corner_points(obj, x,y, th, margin)
            arr = horzcat(x,y);
            %arr = unique(arr,'rows');
            x = arr(:,1);
            y = arr(:,2);
        
            len = length(arr);
        
            div_x = abs(diff(x));
            div_y = abs(diff(y));
        
            data = [];
            
            for i = 1:len-1
                data = vertcat(data, arr(i,:));
                if (div_x(i) > th) && (div_y(i) > th)
                    if div_x(i) > margin
                        data = vertcat(data, [arr(i+1,1),arr(i,2)]);
                    else
                        data = vertcat(data, [arr(i,1),arr(i+1,2)]);
                    end 
                end
            end
            data = vertcat(data, arr(len,:));
            
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Remove data points
        %
        % Description   :   The function removes data points that are lying
        %                   to close together to keep only the corner
        %                   values
        %
        % Inputs        :   1. Data points (x,y)
        %
        % Outputs       :   1. Filter data points (x,y)
        % =========================================================================
        function [data] = remove_data_points(obj, data_in)
            len = length(data_in);
            data = [];
        
            row_x = 0;
            row_y = 0;
        
            for i = 1:len
                if (data_in(i,1) ~= row_x) || (data_in(i,2) ~= row_y)
                    data = vertcat(data, data_in(i,:));
                    row_x = data_in(i,1);
                    row_y = data_in(i,2);
                end
            end
        end
   end
end