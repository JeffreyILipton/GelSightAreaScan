function pts = makeWaypoints(origin,xysize,delta)
    ns = floor(xysize/delta);

    pts = zeros(4,ns(1)*ns(2));
    for i = 1:ns(1)
        x = (i-1)*delta+origin(1);
        for j = 1:ns(2)
            if mod(i,2)==0
                y = (j-1)*delta+origin(2);
            else
                y = (ns(2)-j)*delta+origin(2);
            end
            k = (i-1)*ns(2)+(j-1)+1;
            pts(:,k)=[x,origin(3),y,1];
        end
    end
end