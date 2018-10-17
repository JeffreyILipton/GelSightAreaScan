function pts = makeWaypoints(origin,xysize,delta)
    ns = xysize/delta;

    pts = zeros(4,ns(1)*ns(2));
    for i = 1:ns(1)
        x = (i-1)*delta+origin(1);
        for j = 1:ns(2)
            y = (j-1)*delta+origin(2);
            k = (i-1)*ns(2)+(j-1)+1;
            pts(:,k)=[x,y,origin(3),1];
        end
    end
end