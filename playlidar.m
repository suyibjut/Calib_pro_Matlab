function playlidar( h )
    axes(plothandle);
    axis([-1,1,-1,1]);
    scandata = receive( h.laser );
    xy = readCartesian( scandata );
    plot( h.lidar_axes, xy(:,1),xy(:,2));
end