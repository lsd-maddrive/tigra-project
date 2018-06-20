function my_fig

    vector = [0];
    cntr = 1;
    num = 0;
    
    f=figure('visible','off','position',...
        [0 0 640 480]);
    slhan=uicontrol('style','slider', 'units','normalized','position',[0 1-.05 1 .05],...
        'min',0,'max',10,'callback',@callbackfn);
    hsttext=uicontrol('style','text', 'units','normalized',...
        'position',[0 1-.05-.07 1 .05]);
    set(hsttext,'visible','on','string',num2str(num))
    axes('units','normalized','position',[.05 .05 .9 .9-.05-.05], 'PlotBoxAspectRatioMode', 'auto');
    movegui(f,'center')
    set(f,'visible','on');
    
    function callbackfn(source,eventdata)
        num=get(slhan,'value');
        set(hsttext,'visible','on','string',num2str(num))
        % x=linspace(0,4*pi);

        %ax=gca;
        %ax.XLim=[0 2*pi]
    end

    while length(vector) < 100
        vector = [vector cntr];
        cntr = cntr + 1;
        
        y=sin(num*vector);
        plot(gca, y);
        
        pause(0.01)
    end
    
end