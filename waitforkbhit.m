function waitforkbhit()
    wkb = 0;
    while ~wkb
        wkb = waitforbuttonpress;
    end
end