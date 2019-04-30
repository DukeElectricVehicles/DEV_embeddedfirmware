function [heading, isDone] = getHeading(curPosLLH_lat, curPosLLH_lon, heading)
    persistent delWayLat_m delWayLon_m waypointInd
    global newWaypoints_lat newWaypoints_lon
    global LATPERM LONPERM delHeadingMax

    if (isempty(delWayLat_m))
        waypointInd = 1;
        delWayLat_m = (newWaypoints_lat(waypointInd+1) - newWaypoints_lat(waypointInd)) / LATPERM;
        delWayLon_m = (newWaypoints_lon(waypointInd+1) - newWaypoints_lon(waypointInd)) / LONPERM;
        delWayMag = sqrt(delWayLat_m*delWayLat_m + delWayLon_m*delWayLon_m);
        delWayLat_m = delWayLat_m / delWayMag;
        delWayLon_m = delWayLon_m / delWayMag;
    end
    
    isDone = false;
    delLat_m = (newWaypoints_lat(waypointInd) - curPosLLH_lat) / LATPERM;
	delLon_m = (newWaypoints_lon(waypointInd) - curPosLLH_lon) / LONPERM;
% 	setLat_m = delLat_m;
% 	setLon_m = delLon_m;
    
	proj = (delLat_m*delWayLat_m) + (delLon_m*delWayLon_m);
    
    if (proj < 0)
        waypointInd = waypointInd + 1;
        if (waypointInd > length(newWaypoints_lat))
            isDone = true;
        elseif (waypointInd == length(newWaypoints_lat))
            delWayLat_m = (newWaypoints_lat(waypointInd) - newWaypoints_lat(waypointInd-1)) / LATPERM;
            delWayLon_m = (newWaypoints_lon(waypointInd) - newWaypoints_lon(waypointInd-1)) / LONPERM;
        else
            delWayLat_m = (newWaypoints_lat(waypointInd+1) - newWaypoints_lat(waypointInd)) / LATPERM;
            delWayLon_m = (newWaypoints_lon(waypointInd+1) - newWaypoints_lon(waypointInd)) / LONPERM;
        end
        delWayMag = sqrt(delWayLat_m*delWayLat_m + delWayLon_m*delWayLon_m);
        delWayLat_m = delWayLat_m / delWayMag;
        delWayLon_m = delWayLon_m / delWayMag;
    end
    
    desHeading = atan2(delLat_m, delLon_m);
    delHeading = mod(desHeading - heading + pi, 2*pi) - pi;
    if (abs(delHeading) > delHeadingMax)
        delHeading = delHeadingMax * sign(delHeading);
    end
    heading = heading + delHeading;
    
end