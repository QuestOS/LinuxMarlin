;M109 S208.000000
;Sliced at: Thu 01-10-2015 19:14:11
;Basic settings: Layer height: 0.2064 Walls: 0.8 Fill: 15
;Print time: #P_TIME#
;Filament used: #F_AMNT#m #F_WGHT#g
;Filament cost: #F_COST#
;M190 S70 ;Uncomment to add your own bed temperature line
;M109 S208 ;Uncomment to add your own temperature line
;G21        ;metric values --RW-- Currently not implemented
G90        ;absolute positioning
M82        ;set extruder to absolute mode
M107       ;start with the fan off
G28 X0 Y0 Z0 ;move X/Y/Z to min endstops
;G28 Z0     ;move Z to min endstops
G29
