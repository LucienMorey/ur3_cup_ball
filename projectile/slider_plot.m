function myslider
x = 0:10;
hplot = plot(x,0*x);
h = uicontrol('style','slider','units','pixel','position',[20 20 300 20]);
addlistener(h,'ContinuousValueChange',@(hObject, event) makeplot(hObject, event,x,hplot));
function makeplot(hObject,event,x,hplot)
n = get(hObject,'Value') * 90;
disp(n)
set(hplot,'ydata',x*n);
drawnow;