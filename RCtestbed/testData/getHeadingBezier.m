function [heading, isDone] = getHeadingBezier(curPosLLH_lat, curPosLLH_lon, heading)
    persistent delWayLatBez_m delWayLonBez_m waypointInd
    global newWaypoints_lat newWaypoints_lon
    global LATPERM LONPERM delHeadingMax MINTURNRAD

    if (isempty(delWayLatBez_m))
        waypointInd = 1;
        delWayLatBez_m = (newWaypoints_lat(waypointInd+1) - newWaypoints_lat(waypointInd)) / LATPERM;
        delWayLonBez_m = (newWaypoints_lon(waypointInd+1) - newWaypoints_lon(waypointInd)) / LONPERM;
        delWayMag = sqrt(delWayLatBez_m*delWayLatBez_m + delWayLonBez_m*delWayLonBez_m);
        delWayLatBez_m = delWayLatBez_m / delWayMag;
        delWayLonBez_m = delWayLonBez_m / delWayMag;
    end
    
    isDone = false;
    
    isFeasible = false;
    while (~isFeasible && ~isDone)
        delLat_m = (newWaypoints_lat(waypointInd) - curPosLLH_lat) / LATPERM;
        delLon_m = (newWaypoints_lon(waypointInd) - curPosLLH_lon) / LONPERM;
        delForward_m = (delLon_m*cos(heading) + delLat_m*sin(heading));
        delRight_m = (delLon_m*sin(heading) - delLat_m*cos(heading));
        
        canAdvance = (delLat_m*delWayLatBez_m + delLon_m*delWayLonBez_m) < 0;

    %     isFeasible?
        dirDel = atan2(delLat_m, delLon_m);
        dirWay = atan2(delWayLatBez_m, delWayLonBez_m);
        dirWayInBodyCoords = dirDel-dirWay;
        c1 = [MINTURNRAD*sign(delRight_m), 0];
        c2 = [delRight_m, delForward_m] + ...
             [cos(dirWayInBodyCoords-pi/2), sin(dirWayInBodyCoords-pi/2)] * MINTURNRAD * sign(delRight_m);
        isFeasible = sum((c1-c2).^2) >= MINTURNRAD.*2;

        if (~isFeasible || canAdvance)
            waypointInd = waypointInd + 1;
            if (waypointInd > length(newWaypoints_lat))
                isDone = true;
            elseif (waypointInd == length(newWaypoints_lat))
                delWayLatBez_m = (newWaypoints_lat(waypointInd) - newWaypoints_lat(waypointInd-1)) / LATPERM;
                delWayLonBez_m = (newWaypoints_lon(waypointInd) - newWaypoints_lon(waypointInd-1)) / LONPERM;
            else
                delWayLatBez_m = (newWaypoints_lat(waypointInd+1) - newWaypoints_lat(waypointInd)) / LATPERM;
                delWayLonBez_m = (newWaypoints_lon(waypointInd+1) - newWaypoints_lon(waypointInd)) / LONPERM;
            end
%             delWayMag = sqrt(delWayLat_m*delWayLat_m + delWayLon_m*delWayLon_m);
%             delWayLat_m = delWayLat_m / delWayMag;
%             delWayLon_m = delWayLon_m / delWayMag;
        end
    end
    
%     desHeading = 999*sign(-delRight_m);
%     
%     delHeading = mod(desHeading - heading + pi, 2*pi) - pi;
%     if (abs(delHeading) > delHeadingMax)
%         delHeading = delHeadingMax * sign(delHeading);
%     end
    heading = heading - delHeadingMax*sign(delRight_m);
    
end