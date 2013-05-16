
set xlabel "measured"
set ylabel "predicted"


set term x11 0 persist
set title "Axis 0"
plot [] [-250:250] "calib_0.out" using 1:2 notitle,  (0.882935)*x+(-13.901711) notitle


set term x11 1 persist
set title "Axis 1"
plot [] [-250:250] "calib_1.out" using 1:2 notitle,  (-0.718480)*x+(-17.068955) notitle


set term x11 2 persist
set title "Axis 2"
plot [] [-250:250] "calib_2.out" using 1:2 notitle,  (-0.205529)*x+(2.667526) notitle

