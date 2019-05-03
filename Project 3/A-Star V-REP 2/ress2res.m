
function md = ress2res(ress)

k = 0;

for td = -100:ress:100 % percent diffrence in speed
   [x,y] = xy(td);
   figure(1)
   plot(x,y,'-');
   hold on
   plot(x(end),y(end),'o');
   axis equal
   if td > -90
       d(k) = sqrt((x(end)-x_old)^2+(y(end)-y_old)^2);
   end
   x_old = x(end);
   y_old = y(end);
   k = k+1;
end

md = min(d);